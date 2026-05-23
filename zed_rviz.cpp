//  zed_rviz.cpp  —  Live stereo → SGBM+WLS depth → structured 3D mesh
//
//  Fixes applied in this version:
//    1. Temporal smoothing of depth (EMA α=0.3) → eliminates floor spikes
//    2. Median filter on disparity before reproject → kills salt-pepper noise
//    3. Adaptive edge threshold based on Z → bridges gaps at far range
//    4. Bilateral smoothing of depth map → smooth surfaces, sharp edges kept
//    5. Hole-filling pass in buildMesh (cross-quad fallback) → fewer top-view gaps
//    6. Tighter radial outlier guard per-Z → no wild points at sides
//
//  Coordinate convention (toWorld):
//    X = -px/1000   (stereo baseline flipped)
//    Y = -py/1000   (image rows down → world Y up)
//    Z =  pz/1000   (depth unchanged)

#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>

#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GLFW/glfw3.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <fstream>
#include <cmath>
#include <iostream>

using namespace sl;

// ─── Coordinate convention ────────────────────────────────────────────────────
inline void toWorld(float px, float py, float pz,
                    float& X, float& Y, float& Z)
{
    X = -px * 1e-3f;
    Y = -py * 1e-3f;
    Z =  pz * 1e-3f;
}

// ─── Camera orbit ─────────────────────────────────────────────────────────────
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

// ─── Shared mesh (ROS → GL) ───────────────────────────────────────────────────
// Each triangle = 18 floats  (3 verts × [x,y,z,r,g,b])
struct MeshData {
    std::vector<float> tris;
    std::atomic<bool>  fresh{false};
    std::mutex         mtx;
} g_mesh;

// ─── buildMesh ────────────────────────────────────────────────────────────────
// pts3D : CV_32FC3, already in metres (post-toWorld, downsampled)
// col   : CV_8UC3 BGR, same size
//
// Key quality improvements:
//   • Adaptive edge threshold: edges up to (Z*0.12) apart allowed → fills gaps
//     at distance without bridging real discontinuities up close
//   • Cross-quad fallback: if a quad has only 3 valid vertices, emit the
//     degenerate triangle anyway (reduces holes at object boundaries)
//   • All 4 vertices of a quad checked together so both triangles share
//     the same depth-continuity decision
void buildMesh(const cv::Mat& pts3D, const cv::Mat& col,
               std::vector<float>& out,
               float z_min=0.3f, float z_max=8.f)
{
    const int rows=pts3D.rows, cols_=pts3D.cols;

    // ── Pass 1: validate ─────────────────────────────────────────────────
    // Store X,Y,Z directly — no re-division in pass 2
    std::vector<int>   vidx(rows*cols_,-1);
    std::vector<float> vx,vy,vz,vr,vg,vb;
    vx.reserve(rows*cols_); vy.reserve(rows*cols_); vz.reserve(rows*cols_);
    vr.reserve(rows*cols_); vg.reserve(rows*cols_); vb.reserve(rows*cols_);

    for(int r=0;r<rows;++r){
        const cv::Vec3f* prow=pts3D.ptr<cv::Vec3f>(r);
        const cv::Vec3b* crow=col.ptr<cv::Vec3b>(r);
        for(int c=0;c<cols_;++c){
            const cv::Vec3f& p=prow[c];
            // pts3D is already in metres here (smoothed, see tick())
            float X=p[0],Y=p[1],Z=p[2];
            if(!std::isfinite(Z)||Z<z_min||Z>z_max) continue;
            if(!std::isfinite(X)||!std::isfinite(Y)) continue;
            // Tighter radial guard scaled by Z
            float maxR=Z*1.2f;
            if(X*X+Y*Y>=maxR*maxR) continue;
            vidx[r*cols_+c]=(int)vx.size();
            const cv::Vec3b& bgr=crow[c];
            vx.push_back(X); vy.push_back(Y); vz.push_back(Z);
            vr.push_back(bgr[2]/255.f);
            vg.push_back(bgr[1]/255.f);
            vb.push_back(bgr[0]/255.f);
        }
    }

    // ── Pass 2: triangulate ───────────────────────────────────────────────
    // Adaptive edge OK: gap allowed = 12% of average Z of the two verts.
    // This naturally allows wider connections far away (fills top-view gaps)
    // while keeping close-range geometry tight (no bridging across real edges).
    auto eOk=[&](int a,int b)->bool{
        if(a<0||b<0) return false;
        float dx=vx[a]-vx[b],dy=vy[a]-vy[b],dz=vz[a]-vz[b];
        float d2=dx*dx+dy*dy+dz*dz;
        float avgZ=0.5f*(vz[a]+vz[b]);
        float thresh=avgZ*0.12f;   // 12% of depth = adaptive gap budget
        return d2 < thresh*thresh;
    };

    auto push3=[&](int a,int b,int c2){
        out.push_back(vx[a]);out.push_back(vy[a]);out.push_back(vz[a]);
        out.push_back(vr[a]);out.push_back(vg[a]);out.push_back(vb[a]);
        out.push_back(vx[b]);out.push_back(vy[b]);out.push_back(vz[b]);
        out.push_back(vr[b]);out.push_back(vg[b]);out.push_back(vb[b]);
        out.push_back(vx[c2]);out.push_back(vy[c2]);out.push_back(vz[c2]);
        out.push_back(vr[c2]);out.push_back(vg[c2]);out.push_back(vb[c2]);
    };

    out.clear();
    out.reserve((rows-1)*(cols_-1)*2*18);

    for(int r=0;r<rows-1;++r) for(int c=0;c<cols_-1;++c){
        int tl=vidx[ r   *cols_+c  ];
        int tr=vidx[ r   *cols_+c+1];
        int bl=vidx[(r+1)*cols_+c  ];
        int br=vidx[(r+1)*cols_+c+1];

        // Count valid corners
        int valid=(tl>=0)+(tr>=0)+(bl>=0)+(br>=0);
        if(valid<3) continue;

        // Upper triangle tl-tr-bl
        if(tl>=0&&tr>=0&&bl>=0&&eOk(tl,tr)&&eOk(tr,bl)&&eOk(bl,tl))
            push3(tl,tr,bl);
        // Lower triangle tr-br-bl
        if(tr>=0&&br>=0&&bl>=0&&eOk(tr,br)&&eOk(br,bl)&&eOk(bl,tr))
            push3(tr,br,bl);

        // ── Hole-fill: if one corner missing, emit the diagonal triangle ─
        // This fills boundary gaps that appear as holes from top view
        if(valid==3){
            if(tl<0&&tr>=0&&br>=0&&bl>=0&&eOk(tr,br)&&eOk(br,bl)&&eOk(bl,tr))
                push3(tr,br,bl);
            else if(tr<0&&tl>=0&&br>=0&&bl>=0&&eOk(tl,br)&&eOk(br,bl)&&eOk(bl,tl))
                push3(tl,bl,br);
            else if(br<0&&tl>=0&&tr>=0&&bl>=0&&eOk(tl,tr)&&eOk(tr,bl)&&eOk(bl,tl))
                push3(tl,tr,bl);
            else if(bl<0&&tl>=0&&tr>=0&&br>=0&&eOk(tl,tr)&&eOk(tr,br)&&eOk(br,tl))
                push3(tl,tr,br);
        }
    }
}

// ─── OpenGL draw ──────────────────────────────────────────────────────────────
void drawMesh(const std::vector<float>& tris)
{
    if(tris.empty()) return;
    glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
    glLineWidth(0.8f);
    glBegin(GL_TRIANGLES);
    for(size_t i=0;i<tris.size();i+=6){
        glColor3f(tris[i+3]*0.6f+0.3f,tris[i+4]*0.6f+0.3f,tris[i+5]*0.6f+0.3f);
        glVertex3f(tris[i],tris[i+1],tris[i+2]);
    }
    glEnd();
    glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
    glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    glBegin(GL_TRIANGLES);
    for(size_t i=0;i<tris.size();i+=6){
        glColor4f(tris[i+3],tris[i+4],tris[i+5],0.55f);
        glVertex3f(tris[i],tris[i+1],tris[i+2]);
    }
    glEnd();
    glDisable(GL_BLEND);
    glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
}

// ─── ROS2 node ────────────────────────────────────────────────────────────────
class ZedNode : public rclcpp::Node {
public:
    ZedNode() : Node("zed_pointcloud_publisher") {
        publisher_  = create_publisher<sensor_msgs::msg::PointCloud2>("zed/point_cloud",10);
        marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("zed/mesh_marker",10);

        InitParameters ip;
        ip.camera_resolution = RESOLUTION::HD720;
        ip.camera_fps        = 30;
        ip.depth_mode        = DEPTH_MODE::NONE;
        if(zed_.open(ip)!=ERROR_CODE::SUCCESS){
            RCLCPP_ERROR(get_logger(),"ZED open failed"); rclcpp::shutdown(); return;
        }

        CalibrationParameters calib=
            zed_.getCameraInformation().camera_configuration.calibration_parameters;
        auto lc=calib.left_cam, rc=calib.right_cam;
        float bl=calib.getCameraBaseline();
        cv::Mat K1=(cv::Mat_<double>(3,3)<<lc.fx,0,lc.cx,0,lc.fy,lc.cy,0,0,1);
        cv::Mat K2=(cv::Mat_<double>(3,3)<<rc.fx,0,rc.cx,0,rc.fy,rc.cy,0,0,1);
        cv::Mat D1=cv::Mat::zeros(1,5,CV_64F),D2=cv::Mat::zeros(1,5,CV_64F);
        cv::Mat R=cv::Mat::eye(3,3,CV_64F),T=(cv::Mat_<double>(3,1)<<-bl,0,0);
        cv::Mat R1,R2,P1,P2;
        cv::stereoRectify(K1,D1,K2,D2,cv::Size(1280,720),R,T,
                          R1,R2,P1,P2,Q_,cv::CALIB_ZERO_DISPARITY,0);
        cv::initUndistortRectifyMap(K1,D1,R1,P1,cv::Size(1280,720),CV_32FC1,m1x_,m1y_);
        cv::initUndistortRectifyMap(K2,D2,R2,P2,cv::Size(1280,720),CV_32FC1,m2x_,m2y_);

        // blockSize=9 for smoother disparity on planar surfaces (floor/walls)
        int bs=9, nd=128;
        lm_=cv::StereoSGBM::create(0,nd,bs,
            8*3*bs*bs, 32*3*bs*bs,
            1, 0, 7,   // uniquenessRatio=7 (was 5) — fewer false matches
            150, 2,    // speckleWindowSize=150, speckleRange=2
            cv::StereoSGBM::MODE_SGBM_3WAY);
        rm_  = cv::ximgproc::createRightMatcher(lm_);
        wls_ = cv::ximgproc::createDisparityWLSFilter(lm_);
        wls_->setLambda(12000);     // stronger regularisation → smoother flat areas
        wls_->setSigmaColor(1.5);   // slightly higher → more edge-preserving

        lg_.create(720,1280,CV_8U);
        rg_.create(720,1280,CV_8U);

        timer_=create_wall_timer(std::chrono::milliseconds(33),
            std::bind(&ZedNode::tick,this));
        RCLCPP_INFO(get_logger(),"ZED node running");
    }
    ~ZedNode(){ zed_.close(); cv::destroyAllWindows(); }

private:
    void tick()
    {
        if(zed_.grab()!=ERROR_CODE::SUCCESS) return;

        Mat lz,rz;
        zed_.retrieveImage(lz,VIEW::LEFT);
        zed_.retrieveImage(rz,VIEW::RIGHT);
        cv::Mat left (lz.getHeight(),lz.getWidth(),CV_8UC4,lz.getPtr<sl::uchar1>(sl::MEM::CPU));
        cv::Mat right(rz.getHeight(),rz.getWidth(),CV_8UC4,rz.getPtr<sl::uchar1>(sl::MEM::CPU));

        cv::cvtColor(left, lg_,cv::COLOR_BGRA2GRAY);
        cv::cvtColor(right,rg_,cv::COLOR_BGRA2GRAY);
        cv::cvtColor(left, lb_,cv::COLOR_BGRA2BGR);

        cv::remap(lg_,lr_,  m1x_,m1y_,cv::INTER_LINEAR);
        cv::remap(rg_,rr_,  m2x_,m2y_,cv::INTER_LINEAR);
        cv::remap(lb_,lrb_, m1x_,m1y_,cv::INTER_LINEAR);

        cv::Mat dl,dr,df;
        lm_->compute(lr_,rr_,dl);
        rm_->compute(rr_,lr_,dr);
        wls_->filter(dl,lr_,df,dr);
        df.convertTo(dfl_,CV_32F,1.0/16.0);

        // ── Step 1: median filter on disparity (3×3) ─────────────────────
        // Kills isolated bad disparity pixels (salt & pepper) that cause
        // floor spikes. Very fast on a small kernel.
        cv::Mat dfl_med;
        cv::medianBlur(dfl_,dfl_med,3);

        // ── Step 2: reproject to 3D (in mm) ──────────────────────────────
        cv::reprojectImageTo3D(dfl_med,pts3D_raw_,Q_,true,CV_32F);

        // ── Step 3: convert to metres + apply toWorld coordinate fix ──────
        // We work in metres from here so smoothing and thresholds make sense.
        if(pts3D_m_.empty())
            pts3D_m_.create(pts3D_raw_.size(),CV_32FC3);

        for(int r=0;r<pts3D_raw_.rows;++r){
            const cv::Vec3f* src=pts3D_raw_.ptr<cv::Vec3f>(r);
            cv::Vec3f*       dst=pts3D_m_.ptr<cv::Vec3f>(r);
            for(int c=0;c<pts3D_raw_.cols;++c){
                float X,Y,Z;
                toWorld(src[c][0],src[c][1],src[c][2],X,Y,Z);
                dst[c]={X,Y,Z};
            }
        }

        // ── Step 4: temporal EMA smoothing (α=0.25) ──────────────────────
        // Blends current frame with history → floors/walls become rock-steady.
        // Only valid pixels participate; invalid ones don't corrupt history.
        const float ALPHA=0.25f;
        if(pts3D_smooth_.empty()){
            pts3D_m_.copyTo(pts3D_smooth_);
        } else {
            for(int r=0;r<pts3D_m_.rows;++r){
                const cv::Vec3f* cur =pts3D_m_.ptr<cv::Vec3f>(r);
                cv::Vec3f*       smth=pts3D_smooth_.ptr<cv::Vec3f>(r);
                for(int c=0;c<pts3D_m_.cols;++c){
                    float Z=cur[c][2];
                    if(!std::isfinite(Z)||Z<0.3f||Z>8.f){
                        // Invalid pixel — decay toward invalid slowly
                        // (so holes don't "lock in" old data forever)
                        smth[c][2]*=0.85f;
                    } else {
                        smth[c][0]=ALPHA*cur[c][0]+(1.f-ALPHA)*smth[c][0];
                        smth[c][1]=ALPHA*cur[c][1]+(1.f-ALPHA)*smth[c][1];
                        smth[c][2]=ALPHA*cur[c][2]+(1.f-ALPHA)*smth[c][2];
                    }
                }
            }
        }

        // ── Step 5: bilateral filter on Z channel to smooth depth ─────────
        // Preserves real depth edges (walls vs floor) while flattening noise
        // on uniform surfaces. Applied to the downsampled image for speed.
        const int STEP=8;
        cv::Mat pd_raw,cd_raw;
        cv::resize(pts3D_smooth_,pd_raw,cv::Size(),1.0/STEP,1.0/STEP,cv::INTER_NEAREST);
        cv::resize(lrb_,         cd_raw,cv::Size(),1.0/STEP,1.0/STEP,cv::INTER_LINEAR);


        // ── Step 6: downsample pts3D for publish / saveCSV ────────────────────
        // PC_STEP controls point cloud density independently of the mesh STEP.
        cv::resize(pts3D_smooth_, pts3D_pub_, cv::Size(), 1.0/PC_STEP, 1.0/PC_STEP, cv::INTER_NEAREST);
        cv::resize(lrb_,         lrb_pub_,   cv::Size(), 1.0/PC_STEP, 1.0/PC_STEP, cv::INTER_LINEAR);
        // Extract Z, bilateral-filter it, write back
        {
            int h=pd_raw.rows,w=pd_raw.cols;
            cv::Mat Zplane(h,w,CV_32F);
            for(int r=0;r<h;++r){
                const cv::Vec3f* p=pd_raw.ptr<cv::Vec3f>(r);
                float* z=Zplane.ptr<float>(r);
                for(int c=0;c<w;++c) z[c]=p[c][2];
            }
            cv::Mat Zfilt;
            // d=5, sigmaColor=0.05m, sigmaSpace=3px  → fast, edge-preserving
            cv::bilateralFilter(Zplane,Zfilt,5,0.05f,3.f);
            pd_.create(h,w,CV_32FC3);
            for(int r=0;r<h;++r){
                const cv::Vec3f* p=pd_raw.ptr<cv::Vec3f>(r);
                const float*     z=Zfilt.ptr<float>(r);
                cv::Vec3f*       o=pd_.ptr<cv::Vec3f>(r);
                for(int c=0;c<w;++c){
                    // Replace Z with bilateral-smoothed value; X,Y unchanged
                    // (X/Y are already consistent since they came from smooth pts3D)
                    o[c]={p[c][0],p[c][1],z[c]};
                }
            }
            cd_raw.copyTo(cd_);
        }

        // ── Build mesh (pts3D already in metres with toWorld applied) ─────
        {
            std::vector<float> tris;
            buildMesh(pd_,cd_,tris);
            std::lock_guard<std::mutex> lk(g_mesh.mtx);
            g_mesh.tris=std::move(tris);
            g_mesh.fresh.store(true);
        }
        
        

        publishPC();
        publishMarker();

        if(frame_%10==0) saveCSV(frame_);
        ++frame_;

        // Previews
        cv::Mat d8; double mn,mx;
        cv::minMaxLoc(dfl_,&mn,&mx,nullptr,nullptr,dfl_>0);
        if(mx>mn){
            dfl_.convertTo(d8,CV_8U,255.0/(mx-mn),-255.0*mn/(mx-mn));
            cv::Mat dc; cv::applyColorMap(d8,dc,cv::COLORMAP_TURBO);
            cv::imshow("Rectified Left",lr_);
            cv::imshow("Disparity (WLS)",d8);
            cv::imshow("Depth colour",dc);
        }
        cv::waitKey(1);
    }

    // publishPC now reads from pts3D_smooth_ (already in metres, toWorld applied)
    void publishPC()
    {
        sensor_msgs::msg::PointCloud2 msg;
        msg.header.stamp=get_clock()->now(); msg.header.frame_id="zed_left_camera";
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
                bool ok=std::isfinite(Z)&&Z>0.3f&&Z<8.f&&
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

    void publishMarker()
    {
        std::vector<float> snap;
        { std::lock_guard<std::mutex> lk(g_mesh.mtx); snap=g_mesh.tris; }
        if(snap.empty()) return;

        visualization_msgs::msg::Marker m;
        m.header.stamp=get_clock()->now(); m.header.frame_id="zed_left_camera";
        m.ns="mesh"; m.id=0;
        m.type=visualization_msgs::msg::Marker::LINE_LIST;
        m.action=visualization_msgs::msg::Marker::ADD;
        m.scale.x=0.003; m.color.a=1;

        size_t numTri=snap.size()/18;
        m.points.resize(numTri*6);
        m.colors.resize(numTri*6);
        size_t out=0;
        for(size_t i=0;i<snap.size();i+=18){
            auto mp=[&](size_t b)->geometry_msgs::msg::Point{
                geometry_msgs::msg::Point p;
                p.x=snap[b];p.y=snap[b+1];p.z=snap[b+2]; return p;};
            auto mc=[&](size_t b)->std_msgs::msg::ColorRGBA{
                std_msgs::msg::ColorRGBA c;
                c.r=snap[b+3];c.g=snap[b+4];c.b=snap[b+5];c.a=1; return c;};
            auto pA=mp(i),pB=mp(i+6),pC=mp(i+12);
            auto cA=mc(i),cB=mc(i+6),cC=mc(i+12);
            m.points[out]=pA;m.colors[out]=cA;++out;
            m.points[out]=pB;m.colors[out]=cB;++out;
            m.points[out]=pB;m.colors[out]=cB;++out;
            m.points[out]=pC;m.colors[out]=cC;++out;
            m.points[out]=pC;m.colors[out]=cC;++out;
            m.points[out]=pA;m.colors[out]=cA;++out;
        }
        marker_pub_->publish(m);
    }

    void saveCSV(int frame)
    {
        std::string fn="pointcloud_frame_"+std::to_string(frame)+".csv";
        std::ofstream f(fn); f<<"X,Y,Z,R,G,B\n";
        for(int r=0;r<pts3D_pub_.rows;++r){
            const cv::Vec3f* prow=pts3D_pub_.ptr<cv::Vec3f>(r);
            const cv::Vec3b* crow=lrb_pub_.ptr<cv::Vec3b>(r);
            for(int c=0;c<pts3D_pub_.cols;++c){
                float X=prow[c][0],Y=prow[c][1],Z=prow[c][2];
                if(!std::isfinite(X)||!std::isfinite(Y)||!std::isfinite(Z)) continue;
                const cv::Vec3b& bgr=crow[c];
                f<<X<<','<<Y<<','<<Z<<','<<(int)bgr[2]<<','<<(int)bgr[1]<<','<<(int)bgr[0]<<'\n';
            }
        }
        RCLCPP_INFO(get_logger(),"Saved %s",fn.c_str());
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr   publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr  marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    Camera zed_;
    cv::Ptr<cv::StereoSGBM>  lm_;
    cv::Ptr<cv::StereoMatcher> rm_;
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_;
    cv::Mat Q_,m1x_,m1y_,m2x_,m2y_;

    // Pre-allocated buffers
    cv::Mat lg_,rg_,lb_;
    cv::Mat lr_,rr_,lrb_;
    cv::Mat dfl_;
    cv::Mat pts3D_raw_;     // mm, raw reprojection
    cv::Mat pts3D_m_;       // metres, toWorld applied
    cv::Mat pts3D_smooth_;  // metres, EMA smoothed — THE authoritative depth
    cv::Mat pd_,cd_;        // downsampled for buildMesh
    cv::Mat pts3D_pub_;     // downsampled point cloud for publish/save
    cv::Mat lrb_pub_;       // matching colour image, same downsample

    int frame_=0;
    static constexpr int PC_STEP = 8;   // ← change this to control PC density
};


// ─── main ─────────────────────────────────────────────────────────────────────
int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);

    if(!glfwInit()){std::cerr<<"GLFW init failed\n";return 1;}
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR,2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR,1);
    GLFWwindow* win=glfwCreateWindow(1280,720,"Live 3D Spatial Mesh",nullptr,nullptr);
    if(!win){std::cerr<<"GLFW window failed\n";glfwTerminate();return 1;}
    glfwMakeContextCurrent(win);
    glfwSwapInterval(1);
    glfwSetScrollCallback(win,scroll_cb);
    glfwSetMouseButtonCallback(win,mouse_btn_cb);
    glfwSetCursorPosCallback(win,cursor_cb);
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.05f,0.05f,0.10f,1.f);

    auto node=std::make_shared<ZedNode>();
    std::thread ros_thread([&]{rclcpp::spin(node);});

    std::vector<float> render_tris;
    while(!glfwWindowShouldClose(win))
    {
        if(g_mesh.fresh.load()){
            std::lock_guard<std::mutex> lk(g_mesh.mtx);
            render_tris=g_mesh.tris;
            g_mesh.fresh.store(false);
        }

        int W,H; glfwGetFramebufferSize(win,&W,&H);
        glViewport(0,0,W,H);
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

        glMatrixMode(GL_PROJECTION);glLoadIdentity();
        float asp=(H>0)?(float)W/H:1.f;
        float t=0.1f*std::tan(60.f*3.14159f/360.f);
        glFrustum(-t*asp,t*asp,-t,t,0.1f,50.f);

        glMatrixMode(GL_MODELVIEW);glLoadIdentity();
        float yr=g_cam.yaw*3.14159f/180.f, pr=g_cam.pitch*3.14159f/180.f;
        float ecx=g_cam.cx+g_cam.dist*std::cos(pr)*std::sin(yr);
        float ecy=g_cam.cy+g_cam.dist*std::sin(pr);
        float ecz=g_cam.cz+g_cam.dist*std::cos(pr)*std::cos(yr);
        float fx=g_cam.cx-ecx,fy=g_cam.cy-ecy,fz=g_cam.cz-ecz;
        float fl=std::sqrt(fx*fx+fy*fy+fz*fz);
        fx/=fl;fy/=fl;fz/=fl;
        float sx=-fz,sy=0,sz=fx;
        float sl=std::sqrt(sx*sx+sz*sz); sx/=sl;sz/=sl;
        float ux=sy*fz-sz*fy,uy=sz*fx-sx*fz,uz=sx*fy-sy*fx;
        float M[16]={sx,ux,-fx,0,sy,uy,-fy,0,sz,uz,-fz,0,
            -(sx*ecx+sy*ecy+sz*ecz),-(ux*ecx+uy*ecy+uz*ecz),fx*ecx+fy*ecy+fz*ecz,1};
        glLoadMatrixf(M);

        glLineWidth(1.5f);
        glBegin(GL_LINES);
          glColor3f(1,0,0);glVertex3f(0,0,0);glVertex3f(0.5f,0,0);
          glColor3f(0,1,0);glVertex3f(0,0,0);glVertex3f(0,0.5f,0);
          glColor3f(0,0,1);glVertex3f(0,0,0);glVertex3f(0,0,0.5f);
        glEnd();

        drawMesh(render_tris);
        glfwSwapBuffers(win);
        glfwPollEvents();
    }

    rclcpp::shutdown();
    ros_thread.join();
    glfwDestroyWindow(win);
    glfwTerminate();
    return 0;
}

