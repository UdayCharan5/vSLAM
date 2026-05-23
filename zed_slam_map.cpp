//zed_slam_map
//  zed_slam_map.cpp  —  Live ZED SLAM spatial mapping (ZEDfu-style)
//
//  Architecture:
//    • ZED SDK positional tracking  → T_world_camera per frame (free, no extra compute)
//    • SGBM+WLS stereo depth        → your existing high-quality depth pipeline
//    • Open3D TSDF VoxelBlockGrid   → volumetric map fusion (GPU-accelerated)
//    • Marching Cubes / Poisson     → watertight mesh extraction from volume
//    • OpenGL                       → live render of the growing global map
//
//  Key difference from zed_rviz.cpp:
//    OLD: buildMesh() makes a LOCAL mesh for the current frame only.
//         Moving the camera throws away old geometry.
//    NEW: tsdf_volume_.Integrate() ACCUMULATES all frames into a single
//         global volume. The world is built up as you walk through it.
//
//  Build dependencies (add to CMakeLists.txt):
//    find_package(Open3D REQUIRED)
//    find_package(Eigen3 REQUIRED)
//    target_link_libraries(your_target Open3D::Open3D Eigen3::Eigen)
//
//  Controls:
//    Left drag  = orbit camera
//    Scroll     = zoom
//    WASD       = pan the view centre
//    Q/E        = move centre up/down
//
//  TUNE section below for map resolution, memory, and keyframe rate.

#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>

#define GLFW_INCLUDE_NONE
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// Open3D — TSDF + point cloud
#include <open3d/Open3D.h>
#include <open3d/t/geometry/VoxelBlockGrid.h>
#include <open3d/core/Tensor.h>
#include <open3d/core/Device.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <fstream>
#include <cmath>
#include <iostream>
#include <chrono>

using namespace sl;

// ═══════════════════════════════════════════════════════════════════════════════
// TUNE — Map parameters
// ═══════════════════════════════════════════════════════════════════════════════
static constexpr float VOXEL_SIZE      = 0.04f;  // 4 cm — good for rooms/corridors
static constexpr float TSDF_TRUNC      = 0.16f;  // 4× voxel (typical)
static constexpr int   BLOCK_COUNT     = 40000;  // ~300 MB VRAM at 4 cm
static constexpr int   KEYFRAME_EVERY  = 3;      // integrate every N frames
static constexpr int   EXTRACT_EVERY   = 1;     // re-mesh every N keyframes
static constexpr float DEPTH_MIN       = 0.3f;
static constexpr float DEPTH_MAX       = 8.0f;
static constexpr int   PC_STEP         = 3;

// ═══════════════════════════════════════════════════════════════════════════════
// Coordinate convention
// ═══════════════════════════════════════════════════════════════════════════════
inline void toWorld(float px, float py, float pz, float& X, float& Y, float& Z)
{
    X = -px * 1e-3f;
    Y = -py * 1e-3f;
    Z =  pz * 1e-3f;
}

// sl::Transform (row-major 4×4 float) → Eigen::Matrix4f
inline Eigen::Matrix4f slToEigen(const sl::Transform& t)
{
    Eigen::Matrix4f M;
    for(int r=0;r<4;++r)
        for(int c=0;c<4;++c)
            M(r,c) = t.m[r*4+c];
    return M;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Camera orbit + pan for exploring the accumulated map
// ═══════════════════════════════════════════════════════════════════════════════
struct Camera3D {
    float yaw=0.f, pitch=-25.f, dist=5.f;
    float cx=0, cy=0, cz=2;
    double last_mx=-1, last_my=-1;
    bool dragging=false;
} g_cam;

void scroll_cb(GLFWwindow*,double,double dy)
    { g_cam.dist-=(float)dy*0.3f; g_cam.dist=std::max(0.5f,g_cam.dist); }
void mouse_btn_cb(GLFWwindow*,int btn,int action,int)
    { if(btn==GLFW_MOUSE_BUTTON_LEFT) g_cam.dragging=(action==GLFW_PRESS); }
void cursor_cb(GLFWwindow*,double x,double y){
    if(g_cam.dragging&&g_cam.last_mx>=0){
        g_cam.yaw  +=(float)(x-g_cam.last_mx)*0.4f;
        g_cam.pitch+=(float)(y-g_cam.last_my)*0.4f;
        g_cam.pitch=std::max(-89.f,std::min(89.f,g_cam.pitch));
    }
    g_cam.last_mx=x; g_cam.last_my=y;
}
void key_cb(GLFWwindow*,int key,int,int action,int){
    const float spd=0.1f;
    if(action==GLFW_PRESS||action==GLFW_REPEAT){
        if(key==GLFW_KEY_W) g_cam.cz-=spd;
        if(key==GLFW_KEY_S) g_cam.cz+=spd;
        if(key==GLFW_KEY_A) g_cam.cx-=spd;
        if(key==GLFW_KEY_D) g_cam.cx+=spd;
        if(key==GLFW_KEY_Q) g_cam.cy+=spd;
        if(key==GLFW_KEY_E) g_cam.cy-=spd;
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// Shared data: SLAM thread → render thread
// ═══════════════════════════════════════════════════════════════════════════════
struct GlobalMesh {
    std::vector<float>            tris;      // 18 floats/triangle [x,y,z,r,g,b]×3
    std::vector<Eigen::Vector3f>  trajectory;
    std::atomic<bool>             fresh{false};
    std::mutex                    mtx;
} g_mesh;

// ═══════════════════════════════════════════════════════════════════════════════
// Render helpers
// ═══════════════════════════════════════════════════════════════════════════════
void drawMesh(const std::vector<float>& tris)
{
    if(tris.empty()) return;
    // Solid faces
    glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    glBegin(GL_TRIANGLES);
    for(size_t i=0;i<tris.size();i+=18){
        for(int v=0;v<3;++v){
            size_t b=i+v*6;
            glColor4f(tris[b+3],tris[b+4],tris[b+5],0.75f);
            glVertex3f(tris[b],tris[b+1],tris[b+2]);
        }
    }
    glEnd();
    glDisable(GL_BLEND);
    // Wireframe overlay
    glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
    glLineWidth(0.4f);
    glBegin(GL_TRIANGLES);
    for(size_t i=0;i<tris.size();i+=18){
        for(int v=0;v<3;++v){
            size_t b=i+v*6;
            glColor3f(tris[b+3]*0.3f,tris[b+4]*0.3f,tris[b+5]*0.3f+0.1f);
            glVertex3f(tris[b],tris[b+1],tris[b+2]);
        }
    }
    glEnd();
    glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
}

void drawTrajectory(const std::vector<Eigen::Vector3f>& traj)
{
    if(traj.size()<2) return;
    glLineWidth(2.5f);
    glBegin(GL_LINE_STRIP);
    for(size_t i=0;i<traj.size();++i){
        float t=(float)i/(float)traj.size();
        glColor3f(1.f, 0.9f-t*0.7f, 0.1f);   // yellow→orange along path
        glVertex3f(traj[i].x(),traj[i].y(),traj[i].z());
    }
    glEnd();
}

// Convert Open3D TriangleMesh → flat float vector
void o3dMeshToTris(const open3d::geometry::TriangleMesh& mesh,
                   std::vector<float>& out)
{
    out.clear();
    const bool hasCol = mesh.HasVertexColors();
    out.reserve(mesh.triangles_.size()*18);
    for(const auto& tri : mesh.triangles_){
        for(int v=0;v<3;++v){
            int idx=(int)tri[v];
            const auto& p=mesh.vertices_[idx];
            out.push_back((float)p.x());
            out.push_back((float)p.y());
            out.push_back((float)p.z());
            if(hasCol){
                const auto& c=mesh.vertex_colors_[idx];
                out.push_back((float)c.x());
                out.push_back((float)c.y());
                out.push_back((float)c.z());
            } else {
                out.push_back(0.55f); out.push_back(0.75f); out.push_back(0.90f);
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// ZedSlamNode
// ═══════════════════════════════════════════════════════════════════════════════
class ZedSlamNode : public rclcpp::Node {
public:
    ZedSlamNode() : Node("zed_slam_mapper")
    {
        publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("zed/point_cloud",10);
        odom_pub_  = create_publisher<nav_msgs::msg::Odometry>("zed/odom",10);
        pose_pub_  = create_publisher<geometry_msgs::msg::PoseStamped>("zed/pose",10);

        // ── Open ZED ──────────────────────────────────────────────────────
        InitParameters ip;
        ip.camera_resolution = RESOLUTION::HD720;
        ip.camera_fps        = 30;
        ip.depth_mode        = DEPTH_MODE::PERFORMANCE; // needed for tracking; SGBM still used for TSDF
        ip.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
        ip.coordinate_units  = UNIT::METER;
        if(zed_.open(ip)!=ERROR_CODE::SUCCESS){
            RCLCPP_ERROR(get_logger(),"ZED open failed"); rclcpp::shutdown(); return;
        }

        // ── Positional tracking ───────────────────────────────────────────
        // This is the core addition over zed_rviz.cpp.
        // The ZED SDK runs its own visual-inertial odometry (VIO) internally
        // using the stereo pair + IMU.  We call zed_.getPosition() each frame
        // to get a 4×4 world pose for free — no external SLAM library needed.
        sl::PositionalTrackingParameters ptp;
        ptp.enable_area_memory = true;   // loop closure + drift correction
        ptp.enable_imu_fusion  = true;   // fuse IMU if present
        if(zed_.enablePositionalTracking(ptp)==ERROR_CODE::SUCCESS){
            tracking_ok_=true;
            RCLCPP_INFO(get_logger(),"Positional tracking enabled (area memory ON)");
        } else {
            RCLCPP_WARN(get_logger(),"Tracking unavailable — map in camera-local frame");
        }

        // ── Stereo calibration (identical to zed_rviz.cpp) ────────────────
        CalibrationParameters calib=
            zed_.getCameraInformation().camera_configuration.calibration_parameters;
        auto lc=calib.left_cam, rc=calib.right_cam;
        float bl=calib.getCameraBaseline();
        fx_=lc.fx; fy_=lc.fy; cx_=lc.cx; cy_=lc.cy;

        cv::Mat K1=(cv::Mat_<double>(3,3)<<lc.fx,0,lc.cx,0,lc.fy,lc.cy,0,0,1);
        cv::Mat K2=(cv::Mat_<double>(3,3)<<rc.fx,0,rc.cx,0,rc.fy,rc.cy,0,0,1);
        cv::Mat D1=cv::Mat::zeros(1,5,CV_64F),D2=cv::Mat::zeros(1,5,CV_64F);
        cv::Mat R=cv::Mat::eye(3,3,CV_64F),T=(cv::Mat_<double>(3,1)<<-bl,0,0);
        cv::Mat R1,R2,P1,P2;
        cv::stereoRectify(K1,D1,K2,D2,cv::Size(1280,720),R,T,
                          R1,R2,P1,P2,Q_,cv::CALIB_ZERO_DISPARITY,0);
        cv::initUndistortRectifyMap(K1,D1,R1,P1,cv::Size(1280,720),CV_32FC1,m1x_,m1y_);
        cv::initUndistortRectifyMap(K2,D2,R2,P2,cv::Size(1280,720),CV_32FC1,m2x_,m2y_);

        // ── SGBM+WLS (identical to zed_rviz.cpp) ─────────────────────────
        int bs=9,nd=128;
        lm_=cv::StereoSGBM::create(0,nd,bs,8*3*bs*bs,32*3*bs*bs,1,0,7,150,2,
                                    cv::StereoSGBM::MODE_SGBM_3WAY);
        rm_=cv::ximgproc::createRightMatcher(lm_);
        wls_=cv::ximgproc::createDisparityWLSFilter(lm_);
        wls_->setLambda(12000); wls_->setSigmaColor(1.5);
        lg_.create(720,1280,CV_8U); rg_.create(720,1280,CV_8U);

        // ── TSDF volume ───────────────────────────────────────────────────
        initTSDF();

        timer_=create_wall_timer(std::chrono::milliseconds(33),
            std::bind(&ZedSlamNode::tick,this));
        RCLCPP_INFO(get_logger(),
            "SLAM mapper ready | voxel=%.3fm | kf_every=%d | extract_every=%d",
            VOXEL_SIZE, KEYFRAME_EVERY, EXTRACT_EVERY);
    }

    ~ZedSlamNode(){
        // Cancel timer first so no new frames arrive while we save
        if(timer_) timer_->cancel();
        zed_.disablePositionalTracking();
        zed_.close();
        cv::destroyAllWindows();
        saveFinalMap();       // PLY point cloud + mesh (unchanged)
        saveFinalMapCSV();    // NEW: full coloured CSV for CloudCompare
    }

private:
    // ── TSDF setup ────────────────────────────────────────────────────────────
    void makeTSDF(open3d::core::Device& dev)
    {
        tsdf_ = std::make_unique<open3d::t::geometry::VoxelBlockGrid>(
            std::vector<std::string>{"tsdf","weight","color"},
            std::vector<open3d::core::Dtype>{
                open3d::core::Float32,
                open3d::core::Float32,
                open3d::core::Float32},   // Float32 [0,1] — apt CPU kernel requires all-Float32
            std::vector<open3d::core::SizeVector>{{1},{1},{3}},
            VOXEL_SIZE, 16, BLOCK_COUNT, dev);
    }

    void initTSDF()
    {
        try {
            device_ = std::make_shared<open3d::core::Device>("CUDA:0");
            makeTSDF(*device_);
            RCLCPP_INFO(get_logger(), "TSDF on CUDA GPU");
            return;
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "CUDA TSDF unavailable (%s) — CPU fallback", e.what());
        } catch (...) {
            RCLCPP_WARN(get_logger(), "CUDA TSDF unavailable — CPU fallback");
        }
        device_ = std::make_shared<open3d::core::Device>("CPU:0");
        makeTSDF(*device_);
        RCLCPP_INFO(get_logger(), "TSDF on CPU");
    }

    // ── Main loop ─────────────────────────────────────────────────────────────
    void tick()
    {
        if(zed_.grab()!=ERROR_CODE::SUCCESS) return;

        // 1. Get world pose
        Eigen::Matrix4f T_wc = Eigen::Matrix4f::Identity();
        if(tracking_ok_){
            sl::Pose p;
            if(zed_.getPosition(p,sl::REFERENCE_FRAME::WORLD)==POSITIONAL_TRACKING_STATE::OK){
                T_wc=slToEigen(p.pose_data);
                publishPose(p);
                Eigen::Vector3f pos=T_wc.block<3,1>(0,3);
                std::lock_guard<std::mutex> lk(g_mesh.mtx);
                g_mesh.trajectory.push_back(pos);
                if(g_mesh.trajectory.size()>5000)
                    g_mesh.trajectory.erase(g_mesh.trajectory.begin());
            }
        }

        // 2. Stereo depth (your proven pipeline)
        Mat lz,rz;
        zed_.retrieveImage(lz,VIEW::LEFT);
        zed_.retrieveImage(rz,VIEW::RIGHT);
        cv::Mat left (lz.getHeight(),lz.getWidth(),CV_8UC4,lz.getPtr<sl::uchar1>(sl::MEM::CPU));
        cv::Mat right(rz.getHeight(),rz.getWidth(),CV_8UC4,rz.getPtr<sl::uchar1>(sl::MEM::CPU));

        cv::cvtColor(left, lg_,cv::COLOR_BGRA2GRAY);
        cv::cvtColor(right,rg_,cv::COLOR_BGRA2GRAY);
        cv::cvtColor(left, lb_,cv::COLOR_BGRA2BGR);
        cv::remap(lg_,lr_,m1x_,m1y_,cv::INTER_LINEAR);
        cv::remap(rg_,rr_,m2x_,m2y_,cv::INTER_LINEAR);
        cv::remap(lb_,lrb_,m1x_,m1y_,cv::INTER_LINEAR);

        cv::Mat dl,dr,df;
        lm_->compute(lr_,rr_,dl);
        rm_->compute(rr_,lr_,dr);
        wls_->filter(dl,lr_,df,dr);
        df.convertTo(dfl_,CV_32F,1.0/16.0);

        cv::Mat dfl_med; cv::medianBlur(dfl_,dfl_med,3);
        cv::reprojectImageTo3D(dfl_med,pts3D_raw_,Q_,true,CV_32F);

        if(pts3D_m_.empty()) pts3D_m_.create(pts3D_raw_.size(),CV_32FC3);
        for(int r=0;r<pts3D_raw_.rows;++r){
            const cv::Vec3f* s=pts3D_raw_.ptr<cv::Vec3f>(r);
            cv::Vec3f* d=pts3D_m_.ptr<cv::Vec3f>(r);
            for(int c=0;c<pts3D_raw_.cols;++c){
                float X,Y,Z; toWorld(s[c][0],s[c][1],s[c][2],X,Y,Z);
                d[c]={X,Y,Z};
            }
        }

        // EMA temporal smoothing
        const float A=0.25f;
        if(pts3D_smooth_.empty()) pts3D_m_.copyTo(pts3D_smooth_);
        else {
            for(int r=0;r<pts3D_m_.rows;++r){
                const cv::Vec3f* cur=pts3D_m_.ptr<cv::Vec3f>(r);
                cv::Vec3f* sm=pts3D_smooth_.ptr<cv::Vec3f>(r);
                for(int c=0;c<pts3D_m_.cols;++c){
                    float Z=cur[c][2];
                    if(!std::isfinite(Z)||Z<DEPTH_MIN||Z>DEPTH_MAX) sm[c][2]*=0.85f;
                    else {
                        sm[c][0]=A*cur[c][0]+(1-A)*sm[c][0];
                        sm[c][1]=A*cur[c][1]+(1-A)*sm[c][1];
                        sm[c][2]=A*cur[c][2]+(1-A)*sm[c][2];
                    }
                }
            }
        }

        // 3. Keyframe-based TSDF integration
        ++frame_;
        if(frame_%KEYFRAME_EVERY==0){
            integrateTSDF(pts3D_raw_,lrb_,T_wc);
            ++kf_count_;
            if(kf_count_%EXTRACT_EVERY==0) extractMesh();
        }

        // 4. Point cloud publish
        cv::resize(pts3D_smooth_,pts3D_pub_,cv::Size(),1.0/PC_STEP,1.0/PC_STEP,cv::INTER_NEAREST);
        cv::resize(lrb_,lrb_pub_,cv::Size(),1.0/PC_STEP,1.0/PC_STEP,cv::INTER_LINEAR);
        publishPC();

        // 5. Previews
        cv::Mat d8; double mn,mx;
        cv::minMaxLoc(dfl_,&mn,&mx,nullptr,nullptr,dfl_>0);
        if(mx>mn){
            dfl_.convertTo(d8,CV_8U,255.0/(mx-mn),-255.0*mn/(mx-mn));
            cv::Mat dc; cv::applyColorMap(d8,dc,cv::COLORMAP_TURBO);
            cv::imshow("Left",lr_); cv::imshow("Depth",dc);
        }
        cv::waitKey(1);
    }

    // ── TSDF integration ──────────────────────────────────────────────────────
    void integrateTSDF(const cv::Mat& pts3D, const cv::Mat& colour,
                       const Eigen::Matrix4f& T_wc)
    {
        const int H=pts3D.rows, W=pts3D.cols;

        // Depth: Z channel of pts3D_raw_ (metres, +Z forward, standard pinhole)
        cv::Mat depth_m(H, W, CV_32F);
        for(int r = 0; r < H; ++r){
            const cv::Vec3f* p = pts3D.ptr<cv::Vec3f>(r);
            float* d = depth_m.ptr<float>(r);
            for(int c = 0; c < W; ++c){
                float Z = p[c][2];
                d[c] = (std::isfinite(Z) && Z > DEPTH_MIN && Z < DEPTH_MAX) ? Z : 0.f;
            }
        }

        // Colour: Float32 [0,1] RGB — apt CPU kernel requires Float32 (not UInt8)
        cv::Mat rgb_u8, rgb_f32;
        cv::cvtColor(colour, rgb_u8, cv::COLOR_BGR2RGB);
        rgb_u8.convertTo(rgb_f32, CV_32FC3, 1.0f / 255.0f);

        open3d::core::Tensor depth_cpu(depth_m.ptr<float>(), {H,W,1},
            open3d::core::Float32, open3d::core::Device("CPU:0"));
        open3d::core::Tensor color_cpu(rgb_f32.ptr<float>(), {H,W,3},
            open3d::core::Float32, open3d::core::Device("CPU:0"));

        open3d::t::geometry::Image depth_img(depth_cpu.To(*device_));
        open3d::t::geometry::Image color_img(color_cpu.To(*device_));

        // Intrinsics: Float64 CPU
        open3d::core::Tensor K = open3d::core::Tensor::Eye(3,
            open3d::core::Float64, open3d::core::Device("CPU:0"));
        K[0][0]=(double)fx_; K[1][1]=(double)fy_;
        K[0][2]=(double)cx_; K[1][2]=(double)cy_;

        // Extrinsics: T_cam_world = T_wc^{-1}, Float64 CPU
        Eigen::Matrix4d T_cw = T_wc.cast<double>().inverse();
        open3d::core::Tensor ext(T_cw.data(), {4,4},
            open3d::core::Float64, open3d::core::Device("CPU:0"));

        try {
            auto block_coords = tsdf_->GetUniqueBlockCoordinates(
                depth_img, K, ext, 1.0f, DEPTH_MAX, TSDF_TRUNC);
            tsdf_->Integrate(block_coords, depth_img, color_img,
                             K, ext, 1.0f, DEPTH_MAX);
        } catch(const std::exception& e){
            RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,
                "TSDF::Integrate: %s",e.what());
        }
    }

    // ── Mesh extraction: marching cubes directly on TSDF (fast + colored) ──────
    void extractMesh()
    {
        auto t0 = std::chrono::steady_clock::now();
        try {
            auto tmesh_cpu = tsdf_->ExtractTriangleMesh()
                                   .To(open3d::core::Device("CPU:0"));

            auto verts_t  = tmesh_cpu.GetVertexPositions();   // (N,3) Float32
            auto tris_idx = tmesh_cpu.GetTriangleIndices();   // (M,3) Int32 or Int64
            const int64_t NT = tris_idx.GetShape(0);
            if(NT == 0) return;

            bool has_color = tmesh_cpu.HasVertexColors();
            open3d::core::Tensor colors_t;
            if(has_color) colors_t = tmesh_cpu.GetVertexColors(); // (N,3) Float32

            const float* V = verts_t.GetDataPtr<float>();
            const float* C = has_color ? colors_t.GetDataPtr<float>() : nullptr;

            // Handle Int32 or Int64 triangle index buffer
            std::vector<int64_t> idx_buf;
            const int64_t* TI;
            if(tris_idx.GetDtype() == open3d::core::Int64){
                TI = tris_idx.GetDataPtr<int64_t>();
            } else {
                const int32_t* src = tris_idx.GetDataPtr<int32_t>();
                idx_buf.resize(NT*3);
                for(int64_t i=0;i<NT*3;++i) idx_buf[i]=(int64_t)src[i];
                TI = idx_buf.data();
            }

            // Pack [x,y,z,r,g,b]×3 per triangle.
            // TSDF frame: +X right, +Y down, +Z forward (camera/OpenCV).
            // Viewer frame: +Y up — flip Y and Z for correct upright display.
            std::vector<float> out;
            out.reserve(NT * 18);
            for(int64_t t=0; t<NT; ++t){
                for(int v=0; v<3; ++v){
                    int64_t vi = TI[t*3+v];
                    out.push_back( V[vi*3+0]);        // X unchanged
                    out.push_back(-V[vi*3+1]);        // Y flipped
                    out.push_back(-V[vi*3+2]);        // Z flipped
                    if(C){ out.push_back(C[vi*3+0]); out.push_back(C[vi*3+1]); out.push_back(C[vi*3+2]); }
                    else  { out.push_back(0.6f);      out.push_back(0.8f);      out.push_back(0.9f); }
                }
            }

            long ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now()-t0).count();
            RCLCPP_INFO(get_logger(),"Mesh: %ld verts %ld tris (%ld ms) KF#%d",
                (long)verts_t.GetShape(0),(long)NT,ms,kf_count_);

            std::lock_guard<std::mutex> lk(g_mesh.mtx);
            g_mesh.tris = std::move(out);
            g_mesh.fresh.store(true);
        } catch(const std::exception& e){
            RCLCPP_WARN(get_logger(),"Mesh extract: %s",e.what());
        }
    }

    // ── Publish ───────────────────────────────────────────────────────────────
    void publishPC()
    {
        sensor_msgs::msg::PointCloud2 msg;
        msg.header.stamp=get_clock()->now(); msg.header.frame_id="map";
        msg.height=pts3D_pub_.rows; msg.width=pts3D_pub_.cols; msg.is_dense=false;
        sensor_msgs::PointCloud2Modifier mod(msg);
        mod.setPointCloud2Fields(4,
            "x",1,sensor_msgs::msg::PointField::FLOAT32,
            "y",1,sensor_msgs::msg::PointField::FLOAT32,
            "z",1,sensor_msgs::msg::PointField::FLOAT32,
            "rgb",1,sensor_msgs::msg::PointField::FLOAT32);
        mod.resize(pts3D_pub_.rows*pts3D_pub_.cols);
        sensor_msgs::PointCloud2Iterator<float>   ix(msg,"x"),iy(msg,"y"),iz(msg,"z");
        sensor_msgs::PointCloud2Iterator<uint8_t> ir(msg,"rgb");
        for(int r=0;r<pts3D_pub_.rows;++r){
            const cv::Vec3f* prow=pts3D_pub_.ptr<cv::Vec3f>(r);
            const cv::Vec3b* crow=lrb_pub_.ptr<cv::Vec3b>(r);
            for(int c=0;c<pts3D_pub_.cols;++c){
                float X=prow[c][0],Y=prow[c][1],Z=prow[c][2];
                bool ok=std::isfinite(Z)&&Z>DEPTH_MIN&&Z<DEPTH_MAX&&
                        std::isfinite(X)&&std::isfinite(Y)&&
                        X*X+Y*Y<(Z*1.2f)*(Z*1.2f);
                *ix=ok?X:NAN; *iy=ok?Y:NAN; *iz=ok?Z:NAN;
                const cv::Vec3b& bgr=crow[c];
                ir[0]=bgr[2]; ir[1]=bgr[1]; ir[2]=bgr[0];
                ++ix;++iy;++iz;++ir;
            }
        }
        publisher_->publish(msg);
    }

    void publishPose(sl::Pose p)
    {
        auto now=get_clock()->now();
        geometry_msgs::msg::PoseStamped ps;
        ps.header.stamp=now; ps.header.frame_id="map";
        auto tr=p.getTranslation(); auto ori=p.getOrientation();
        ps.pose.position.x=tr.x; ps.pose.position.y=tr.y; ps.pose.position.z=tr.z;
        ps.pose.orientation.x=ori.x; ps.pose.orientation.y=ori.y;
        ps.pose.orientation.z=ori.z; ps.pose.orientation.w=ori.w;
        pose_pub_->publish(ps);
        nav_msgs::msg::Odometry od;
        od.header=ps.header; od.child_frame_id="zed_camera";
        od.pose.pose=ps.pose;
        odom_pub_->publish(od);
    }

    void saveFinalMap()
    {
        if(!tsdf_){ RCLCPP_WARN(get_logger(),"TSDF gone, skip PLY save"); return; }
        RCLCPP_INFO(get_logger(),"Saving final PLY maps ...");
        try {
            auto tpcd = tsdf_->ExtractPointCloud();
            auto lpcd = tpcd.To(open3d::core::Device("CPU:0")).ToLegacy();
            open3d::io::WritePointCloud("final_map.ply", lpcd);
            RCLCPP_INFO(get_logger(),"Saved final_map.ply (%zu pts)", lpcd.points_.size());
        } catch(const std::exception& e){
            RCLCPP_WARN(get_logger(),"PLY point cloud save failed: %s", e.what());
        }
        try {
            auto lmesh = tsdf_->ExtractTriangleMesh()
                               .To(open3d::core::Device("CPU:0"))
                               .ToLegacy();
            open3d::io::WriteTriangleMesh("final_mesh.ply", lmesh);
            RCLCPP_INFO(get_logger(),"Saved final_mesh.ply (%zu tris)", lmesh.triangles_.size());
        } catch(const std::exception& e){
            RCLCPP_WARN(get_logger(),"PLY mesh save failed: %s", e.what());
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // saveFinalMapCSV — extracts the complete fused TSDF point cloud and writes
    // it as a CloudCompare-ready CSV with true RGB colours.
    //
    // Coordinate convention (matches CloudCompare / standard right-hand Y-up):
    //   CSV_X =  tsdf_X          (right)
    //   CSV_Y = -tsdf_Y          (TSDF is Y-down → flip to Y-up)
    //   CSV_Z = -tsdf_Z          (TSDF is Z-fwd  → flip for CC right-hand)
    //
    // RGB: written as 0–255 integers AND as 0.0–1.0 floats side-by-side.
    //   When you open in CloudCompare:
    //     • File → Open → final_map.csv
    //     • Separator: comma   |   tick "Contains header"
    //     • Assign  col0→X  col1→Y  col2→Z
    //               col3→R(0-255)  col4→G(0-255)  col5→B(0-255)
    //       OR      col6→R(float)  col7→G(float)  col8→B(float)
    //   Both sets are included so either importer path works.
    //   Do NOT assign the colour columns as "Scalar field" — assign them as
    //   "Red", "Green", "Blue" (or "Red (0-1)" etc.) to get real colour.
    //
    // The function also applies a small statistical outlier filter (radius search
    // with min neighbours threshold) to remove isolated noise points before saving.
    // ─────────────────────────────────────────────────────────────────────────
    void saveFinalMapCSV()
    {
        if(!tsdf_){ RCLCPP_WARN(get_logger(),"TSDF gone, skip CSV save"); return; }
        RCLCPP_INFO(get_logger(),"Extracting final point cloud for CSV ...");

        open3d::geometry::PointCloud lpcd;
        try {
            auto tpcd = tsdf_->ExtractPointCloud();
            lpcd = tpcd.To(open3d::core::Device("CPU:0")).ToLegacy();
        } catch(const std::exception& e){
            RCLCPP_WARN(get_logger(),"CSV: point cloud extraction failed: %s", e.what());
            return;
        }

        if(lpcd.points_.empty()){
            RCLCPP_WARN(get_logger(),"CSV: extracted point cloud is empty, nothing to save");
            return;
        }
        RCLCPP_INFO(get_logger(),"CSV: extracted %zu raw points", lpcd.points_.size());

        // ── Colour quality: estimate normals + apply simple white-balance ────
        // The TSDF colour is already fused from many frames, but can be
        // slightly dark/grey due to averaging.  We apply a per-channel
        // brightness stretch (auto white-balance) so colours look vivid.
        bool has_color = lpcd.HasColors();
        if(has_color && !lpcd.colors_.empty()){
            // Find per-channel min/max over a random 5 % sample for speed
            double rmin=1, rmax=0, gmin=1, gmax=0, bmin=1, bmax=0;
            size_t step = std::max<size_t>(1, lpcd.colors_.size()/50000);
            for(size_t i=0; i<lpcd.colors_.size(); i+=step){
                double r=lpcd.colors_[i].x();
                double g=lpcd.colors_[i].y();
                double b=lpcd.colors_[i].z();
                if(r<rmin) rmin=r; if(r>rmax) rmax=r;
                if(g<gmin) gmin=g; if(g>gmax) gmax=g;
                if(b<bmin) bmin=b; if(b>bmax) bmax=b;
            }
            // Clamp stretch range to avoid blowing out uniform-colour scenes
            double rr = (rmax-rmin)>0.05 ? (rmax-rmin) : 1.0;
            double gr = (gmax-gmin)>0.05 ? (gmax-gmin) : 1.0;
            double br = (bmax-bmin)>0.05 ? (bmax-bmin) : 1.0;
            // Apply stretch in-place
            for(auto& c : lpcd.colors_){
                c.x() = std::min(1.0, std::max(0.0, (c.x()-rmin)/rr));
                c.y() = std::min(1.0, std::max(0.0, (c.y()-gmin)/gr));
                c.z() = std::min(1.0, std::max(0.0, (c.z()-bmin)/br));
            }
            RCLCPP_INFO(get_logger(),"CSV: auto white-balance applied");
        }

        // ── Statistical outlier removal (removes isolated noise specks) ─────
        // Keep points that have at least 5 neighbours within 3× voxel radius.
        try {
            auto [clean, _] = lpcd.RemoveStatisticalOutliers(
                /*nb_neighbors=*/20, /*std_ratio=*/2.0);
            size_t removed = lpcd.points_.size() - clean->points_.size();
            lpcd = *clean;
            RCLCPP_INFO(get_logger(),
                "CSV: outlier removal: %zu pts kept, %zu removed",
                lpcd.points_.size(), removed);
        } catch(const std::exception& e){
            RCLCPP_WARN(get_logger(),"CSV: outlier removal failed (%s), saving raw", e.what());
        }

        // ── Write CSV ────────────────────────────────────────────────────────
        std::ofstream f("final_map.csv");
        if(!f.is_open()){
            RCLCPP_WARN(get_logger(),"CSV: cannot open final_map.csv for writing");
            return;
        }

        // Header — CloudCompare ASCII wizard: tick "Contains header",
        // separator = comma, then assign columns as described above.
        f << "X,Y,Z,R_255,G_255,B_255,R_f,G_f,B_f\n";
        f << std::fixed;
        f.precision(6);

        const bool hc = lpcd.HasColors();
        size_t written = 0;

        for(size_t i=0; i<lpcd.points_.size(); ++i){
            const auto& p = lpcd.points_[i];

            // Skip NaN/Inf points
            if(!std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z()))
                continue;

            // Coordinate flip: TSDF Y-down Z-fwd  →  CloudCompare Y-up
            float cx =  (float)p.x();
            float cy = -(float)p.y();   // Y-down → Y-up
            float cz = -(float)p.z();   // Z-fwd  → Z-back (right-hand Y-up)

            // Colour (0–1 float from TSDF colour channel)
            float rf=0.5f, gf=0.5f, bf=0.5f;
            if(hc){
                rf = std::min(1.f, std::max(0.f, (float)lpcd.colors_[i].x()));
                gf = std::min(1.f, std::max(0.f, (float)lpcd.colors_[i].y()));
                bf = std::min(1.f, std::max(0.f, (float)lpcd.colors_[i].z()));
            }
            // Integer versions (0–255)
            int ri = (int)std::round(rf * 255.f);
            int gi = (int)std::round(gf * 255.f);
            int bi = (int)std::round(bf * 255.f);

            f << cx << ','
              << cy << ','
              << cz << ','
              << ri << ',' << gi << ',' << bi << ','
              << rf << ',' << gf << ',' << bf << '\n';
            ++written;
        }

        f.close();
        //RCLCPP_INFO(get_logger(),
        //    "Saved final_map.csv (%zu points, RGB columns 3-5 are 0-255 ints, "
        //    "cols 6-8 are 0.0-1.0 floats)", written);
    }

    // ── Members ───────────────────────────────────────────────────────────────
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr   publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr         odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    Camera zed_;
    bool   tracking_ok_=false;
    float  fx_,fy_,cx_,cy_;

    cv::Ptr<cv::StereoSGBM>                   lm_;
    cv::Ptr<cv::StereoMatcher>                rm_;
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_;
    cv::Mat Q_,m1x_,m1y_,m2x_,m2y_;
    cv::Mat lg_,rg_,lb_,lr_,rr_,lrb_,dfl_;
    cv::Mat pts3D_raw_,pts3D_m_,pts3D_smooth_;
    cv::Mat pts3D_pub_,lrb_pub_;

    std::shared_ptr<open3d::core::Device>              device_;
    std::unique_ptr<open3d::t::geometry::VoxelBlockGrid> tsdf_;

    int frame_=0, kf_count_=0;
};

// ═══════════════════════════════════════════════════════════════════════════════
// main
// ═══════════════════════════════════════════════════════════════════════════════
int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);

    if(!glfwInit()){std::cerr<<"GLFW init failed\n";return 1;}
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR,2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR,1);
    GLFWwindow* win=glfwCreateWindow(1280,720,"ZED SLAM Map",nullptr,nullptr);
    if(!win){glfwTerminate();return 1;}
    glfwMakeContextCurrent(win);
    glewExperimental = GL_TRUE;
    if(glewInit()!=GLEW_OK){std::cerr<<"GLEW init failed\n";return 1;}
    glfwSwapInterval(1);
    glfwSetScrollCallback(win,scroll_cb);
    glfwSetMouseButtonCallback(win,mouse_btn_cb);
    glfwSetCursorPosCallback(win,cursor_cb);
    glfwSetKeyCallback(win,key_cb);
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.05f,0.05f,0.10f,1.f);

    auto node=std::make_shared<ZedSlamNode>();
    std::thread ros_thread([&]{ rclcpp::spin(node); });

    std::vector<float>           render_tris;
    std::vector<Eigen::Vector3f> render_traj;

    while(!glfwWindowShouldClose(win))
    {
        if(g_mesh.fresh.load()){
            std::lock_guard<std::mutex> lk(g_mesh.mtx);
            render_tris=g_mesh.tris;
            render_traj=g_mesh.trajectory;
            g_mesh.fresh.store(false);
        }

        int W,H; glfwGetFramebufferSize(win,&W,&H);
        glViewport(0,0,W,H);
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

        glMatrixMode(GL_PROJECTION); glLoadIdentity();
        float asp=(H>0)?(float)W/H:1.f;
        float tn=0.1f*std::tan(60.f*3.14159f/360.f);
        glFrustum(-tn*asp,tn*asp,-tn,tn,0.1f,100.f);

        glMatrixMode(GL_MODELVIEW); glLoadIdentity();
        float yr=g_cam.yaw*3.14159f/180.f, pr=g_cam.pitch*3.14159f/180.f;
        float ecx=g_cam.cx+g_cam.dist*std::cos(pr)*std::sin(yr);
        float ecy=g_cam.cy+g_cam.dist*std::sin(pr);
        float ecz=g_cam.cz+g_cam.dist*std::cos(pr)*std::cos(yr);
        float fx=g_cam.cx-ecx,fy=g_cam.cy-ecy,fz=g_cam.cz-ecz;
        float fl=std::sqrt(fx*fx+fy*fy+fz*fz); fx/=fl;fy/=fl;fz/=fl;
        float sx=-fz,sy=0,sz=fx;
        float sl2=std::sqrt(sx*sx+sz*sz); sx/=sl2;sz/=sl2;
        float ux=sy*fz-sz*fy,uy=sz*fx-sx*fz,uz=sx*fy-sy*fx;
        float M[16]={sx,ux,-fx,0,sy,uy,-fy,0,sz,uz,-fz,0,
            -(sx*ecx+sy*ecy+sz*ecz),-(ux*ecx+uy*ecy+uz*ecz),fx*ecx+fy*ecy+fz*ecz,1};
        glLoadMatrixf(M);

        // World axes
        glLineWidth(2.f);
        glBegin(GL_LINES);
          glColor3f(1,0,0);glVertex3f(0,0,0);glVertex3f(1,0,0);
          glColor3f(0,1,0);glVertex3f(0,0,0);glVertex3f(0,1,0);
          glColor3f(0,0,1);glVertex3f(0,0,0);glVertex3f(0,0,1);
        glEnd();

        // Ground grid
        glLineWidth(0.5f); glColor3f(0.2f,0.2f,0.3f);
        glBegin(GL_LINES);
        for(int i=-10;i<=10;++i){
            glVertex3f((float)i,0,-10);glVertex3f((float)i,0,10);
            glVertex3f(-10,0,(float)i);glVertex3f(10,0,(float)i);
        }
        glEnd();

        drawTrajectory(render_traj);
        drawMesh(render_tris);

        glfwSetWindowTitle(win,(
            std::string("ZED SLAM Map | ")+
            std::to_string(render_tris.size()/18)+" tris | "+
            "WASD=pan  Drag=orbit  Scroll=zoom").c_str());

        glfwSwapBuffers(win);
        glfwPollEvents();
    }

    rclcpp::shutdown();
    ros_thread.join();
    glfwDestroyWindow(win);
    glfwTerminate();
    return 0;
}
