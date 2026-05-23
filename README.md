# vSLAM
vSLAM_using_ZED_2i

# ZED 2i vSLAM — Full Installation & Run Guide
**Ubuntu 24.04 LTS | RTX 5060 (Blackwell) | CUDA 12.8+ | ROS 2 Jazzy**

**Targets:** `zed_rviz` (per-frame live mesh viewer) and `zed_slam_map` (accumulative TSDF SLAM mapper)

> ⚠️ **RTX 5060 (Blackwell) — critical differences from older GPUs:**
> The RTX 5060 is NVIDIA Blackwell architecture, compute capability **sm_120**.
> This is brand-new (2025) and requires:
> - NVIDIA driver **≥ 575**
> - CUDA **12.8 or newer**
> - OpenCV built with `CUDA_ARCH_BIN="12.0"`
> - Open3D built **from source** — no pip wheel has sm_120 support yet
> - ROS 2 **Jazzy** — Humble is for Ubuntu 22.04 and will NOT install on 24.04

---

## Table of Contents

1. [System Requirements](#1-system-requirements)
2. [OS and NVIDIA Driver Setup](#2-os-and-nvidia-driver-setup)
3. [CUDA Toolkit 12.8](#3-cuda-toolkit-128)
4. [ZED SDK 4.x](#4-zed-sdk-4x)
5. [ROS 2 Jazzy](#5-ros-2-jazzy)
6. [OpenCV 4.10 with contrib](#6-opencv-410-with-contrib)
7. [Open3D from Source](#7-open3d-from-source)
8. [Remaining Dependencies](#8-remaining-dependencies)
9. [Workspace Setup and Build](#9-workspace-setup-and-build)
10. [Running the Nodes](#10-running-the-nodes)
11. [Tuning Parameters](#11-tuning-parameters)
12. [Output Files](#12-output-files)
13. [Troubleshooting](#13-troubleshooting)

---

## 1. System Requirements

| Component     | Requirement                              |
|---------------|------------------------------------------|
| OS            | Ubuntu 24.04 LTS (Noble Numbat)          |
| GPU           | NVIDIA RTX 5060 — Blackwell, sm_120      |
| VRAM          | 8 GB (default map uses ~300 MB)          |
| NVIDIA Driver | **≥ 575** — mandatory for Blackwell      |
| CUDA Toolkit  | **12.8 or newer**                        |
| ROS 2         | **Jazzy Jalopy** (Ubuntu 24.04 LTS)      |
| OpenCV        | 4.10.x — built from source with contrib  |
| Open3D        | Built from source (main branch)          |
| USB           | USB 3.1 Gen 2 Type-C                     |
| RAM           | 16 GB min, 32 GB recommended             |

---

## 2. OS and NVIDIA Driver Setup

### 2.1 Update the system

```bash
sudo apt update && sudo apt full-upgrade -y
sudo apt install -y build-essential cmake git wget curl pkg-config \
    libssl-dev python3-pip python3-dev software-properties-common
```

### 2.2 Install NVIDIA driver 575+ (Blackwell requirement)

Ubuntu 24.04's default driver repo may ship an older version. Use the graphics-drivers PPA:

```bash
sudo add-apt-repository ppa:graphics-drivers/ppa -y
sudo apt update
sudo apt install -y nvidia-driver-575
sudo reboot
```

After reboot, verify:

```bash
nvidia-smi
# Should show: Driver Version: 575.xx  |  CUDA Version: 12.8
# GPU: NVIDIA GeForce RTX 5060
```

If it shows an older driver:

```bash
sudo apt remove --purge 'nvidia-*' && sudo apt autoremove -y
sudo apt install -y nvidia-driver-575
sudo reboot
```

### 2.3 About Blackwell (sm_120) — read once

The RTX 5060's compute capability is **12.0** (sm_120). Every time you compile software with CUDA support in this guide, the flag is always `CUDA_ARCH_BIN="12.0"`. Using older values like `8.6` or `8.9` will build a binary your GPU silently refuses to run.

---

## 3. CUDA Toolkit 12.8

The driver installs a CUDA runtime, but you need the full **toolkit** (nvcc, headers, libraries) to build OpenCV, Open3D, and for the ZED SDK.

### 3.1 Add NVIDIA CUDA repo for Ubuntu 24.04

```bash
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt update
```

### 3.2 Install CUDA 12.8

```bash
sudo apt install -y cuda-toolkit-12-8
```

### 3.3 Add CUDA to PATH — add these lines to ~/.bashrc

```bash
echo 'export PATH=/usr/local/cuda-12.8/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda-12.8/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
```

### 3.4 Verify

```bash
nvcc --version
# Cuda compilation tools, release 12.8
```

If nvcc is not found after this:

```bash
sudo ln -sf /usr/local/cuda-12.8 /usr/local/cuda
export PATH=/usr/local/cuda/bin:$PATH
```

---

## 4. ZED SDK 4.x

### 4.1 Download the Ubuntu 24 / CUDA 12 installer

Visit [stereolabs.com/developers/release](https://www.stereolabs.com/developers/release/) and grab the latest ZED SDK 4.x for Ubuntu 24 + CUDA 12. Example for SDK 4.2:

```bash
wget "https://download.stereolabs.com/zedsdk/4.2/cu12/ubuntu24" \
    -O ZED_SDK_Ubuntu24_cuda12_v4.2.run
chmod +x ZED_SDK_Ubuntu24_cuda12_v4.2.run
sudo ./ZED_SDK_Ubuntu24_cuda12_v4.2.run
```

During installation: accept the EULA, accept tools (ZED_Diagnostic, ZED_Explorer), accept the Python API.

### 4.2 Verify the camera

Plug in the ZED 2i via USB 3 (the blue port), then:

```bash
/usr/local/zed/tools/ZED_Diagnostic
```

All checks (stereo, depth, IMU) should pass.

### 4.3 USB permission fix (if you get "permission denied")

```bash
sudo cp /usr/local/zed/resources/99-slabs.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG video $USER
# Log out and back in for the group change to apply
```

---

## 5. ROS 2 Jazzy

Ubuntu 24.04 LTS maps to **ROS 2 Jazzy Jalopy**. Humble (Ubuntu 22.04) packages will not work here.

### 5.1 Set locale

```bash
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### 5.2 Add ROS 2 Jazzy repository

```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) \
    signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu noble main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
```

### 5.3 Install Jazzy Desktop + message packages

```bash
sudo apt install -y ros-jazzy-desktop

sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-jazzy-nav-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-visualization-msgs \
    ros-jazzy-std-msgs

sudo rosdep init
rosdep update
```

### 5.4 Source Jazzy — add to ~/.bashrc

```bash
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

---

## 6. OpenCV 4.10 with contrib

The WLS disparity filter (`cv::ximgproc::DisparityWLSFilter`) used in both nodes lives in `opencv_contrib`. The apt package never includes contrib — you must build from source.

### 6.1 Install build dependencies

```bash
sudo apt install -y \
    libgtk-3-dev libavcodec-dev libavformat-dev libswscale-dev \
    libv4l-dev libxvidcore-dev libx264-dev libjpeg-dev libpng-dev \
    libtiff-dev gfortran openexr libatlas-base-dev \
    libtbb-dev libdc1394-dev libopenexr-dev \
    libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev \
    python3-numpy
```

### 6.2 Clone source

```bash
cd ~
git clone https://github.com/opencv/opencv.git --branch 4.10.0 --depth 1
git clone https://github.com/opencv/opencv_contrib.git --branch 4.10.0 --depth 1
mkdir opencv/build && cd opencv/build
```

### 6.3 Configure — note sm_120

```bash
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DOPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
  -DWITH_CUDA=ON \
  -DCUDA_ARCH_BIN="12.0" \
  -DCUDA_ARCH_PTX="" \
  -DWITH_CUDNN=ON \
  -DWITH_TBB=ON \
  -DWITH_GTK=ON \
  -DBUILD_opencv_python3=ON \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_TESTS=OFF \
  -DBUILD_PERF_TESTS=OFF \
  -DOPENCV_GENERATE_PKGCONFIG=ON
```

### 6.4 Build and install (~30 minutes)

```bash
make -j$(nproc)
sudo make install
sudo ldconfig
```

### 6.5 Verify

```bash
python3 -c "import cv2; print(cv2.__version__); print(cv2.ximgproc)"
# 4.10.0
# <module 'cv2.ximgproc'>
```

---

## 7. Open3D from Source

As of mid-2025, no pip wheel includes CUDA kernels for sm_120. You must build Open3D from the `main` branch.

### 7.1 Install Open3D build dependencies

```bash
sudo apt install -y \
    libeigen3-dev libglfw3-dev libglew-dev \
    libpng-dev libjpeg-dev libtiff-dev \
    libglu1-mesa-dev freeglut3-dev \
    libusb-1.0-0-dev libudev-dev \
    libboost-all-dev libjsoncpp-dev \
    liblz4-dev libzstd-dev
```

### 7.2 Clone (main branch for Blackwell)

```bash
cd ~
git clone https://github.com/isl-org/Open3D.git --depth 1
mkdir Open3D/build && cd Open3D/build
```

### 7.3 Configure for sm_120

```bash
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_CUDA_MODULE=ON \
  -DCUDA_ARCH="12.0" \
  -DBUILD_SHARED_LIBS=ON \
  -DBUILD_PYTHON_MODULE=ON \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_UNIT_TESTS=OFF \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DGLIBCXX_USE_CXX11_ABI=ON
```

### 7.4 Build and install (~75 minutes)

```bash
make -j$(nproc)
sudo make install
sudo ldconfig
```

### 7.5 Install Python bindings

```bash
sudo make python-package
pip3 install lib/python_package/pip_package/open3d-*.whl
```

### 7.6 Verify CUDA

```bash
python3 -c "import open3d; print(open3d.__version__); print('CUDA:', open3d.core.cuda.is_available())"
# CUDA: True
```

---

## 8. Remaining Dependencies

```bash
# Eigen3
sudo apt install -y libeigen3-dev

# GLFW + OpenGL + GLEW (for the live OpenGL viewer windows)
sudo apt install -y libglfw3-dev libglew-dev libgl1-mesa-dev libglu1-mesa-dev

# PCL (Point Cloud Library)
sudo apt install -y libpcl-dev

# Boost (needed by PCL)
sudo apt install -y libboost-all-dev
```

---

## 9. Workspace Setup and Build

### 9.1 Create workspace

```bash
mkdir -p ~/ros2_ws/src/zed_rviz/src
cd ~/ros2_ws/src/zed_rviz
```

### 9.2 File layout

```
~/ros2_ws/src/zed_rviz/
├── CMakeLists.txt          ← your existing file
├── package.xml             ← create this (content below)
└── src/
    ├── zed_rviz.cpp
    └── zed_slam_map.cpp
```

### 9.3 Create package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd"
    schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>zed_rviz</name>
  <version>1.0.0</version>
  <description>ZED 2i live mesh viewer and vSLAM spatial mapper</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>
  <depend>visualization_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>std_msgs</depend>
  <depend>nav_msgs</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### 9.4 Build

```bash
cd ~/ros2_ws

# Must source Jazzy BEFORE building
source /opt/ros/jazzy/setup.bash

colcon build \
    --packages-select zed_rviz \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers $(nproc)
```

### 9.5 Source the workspace

```bash
source ~/ros2_ws/install/setup.bash

# Add permanently to ~/.bashrc (after the Jazzy source line)
echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc
```

---

## 10. Running the Nodes

### Connect the ZED 2i

Use the original USB 3.1 Gen 2 cable (USB-C). Confirm detection:

```bash
lsusb | grep -i stereo
# Stereolabs ZED 2i
```

### 10.1 zed_rviz — per-frame live mesh viewer

Shows a real-time 3D mesh of only the current frame. Good for verifying depth quality and calibration.

```bash
ros2 run zed_rviz zed_rviz
```

A GLFW window opens with the live mesh. Three OpenCV windows show `Rectified Left`, `Disparity (WLS)`, and `Depth colour`.

Controls: Left drag = orbit | Scroll = zoom

Published topics:
- `/zed/point_cloud` — sensor_msgs/PointCloud2
- `/zed/mesh_marker` — visualization_msgs/Marker

### 10.2 zed_slam_map — accumulative vSLAM mapper

Builds a growing global 3D map as you move the camera. TSDF fusion runs on the RTX 5060.

```bash
ros2 run zed_rviz zed_slam_map
```

The GLFW window shows the map growing. A yellow-orange line traces the camera path.

Controls:
- Left drag → orbit
- Scroll → zoom
- W / S → pan forward / back
- A / D → pan left / right
- Q / E → pan up / down

Published topics:
- `/zed/point_cloud` — sensor_msgs/PointCloud2
- `/zed/odom` — nav_msgs/Odometry
- `/zed/pose` — geometry_msgs/PoseStamped

Press **Ctrl+C** to stop. The node saves `final_map.ply`, `final_map_mesh.ply`, and `final_map.csv` before exiting.

### 10.3 Viewing in RViz2 (optional)

```bash
# Second terminal
ros2 run rviz2 rviz2
```

In RViz2:
- Fixed Frame → `zed_left_camera`
- Add → PointCloud2 → `/zed/point_cloud`
- Add → Marker → `/zed/mesh_marker` (zed_rviz)
- Add → PoseStamped → `/zed/pose` (zed_slam_map)

---

## 11. Tuning Parameters

All constants are at the top of each `.cpp` under the `TUNE` section.

### zed_slam_map.cpp

| Parameter | Default | Notes |
|-----------|---------|-------|
| `VOXEL_SIZE` | `0.04f` (4 cm) | Decrease to `0.02` for finer indoor maps |
| `TSDF_TRUNC` | `0.16f` | Keep at 4× VOXEL_SIZE |
| `BLOCK_COUNT` | `40000` (~300 MB VRAM) | RTX 5060 has 8 GB — safe to raise to `100000` |
| `KEYFRAME_EVERY` | `3` | Integrate every N frames |
| `EXTRACT_EVERY` | `1` | Re-mesh every N keyframes; raise to `5` to reduce load |
| `DEPTH_MIN` | `0.3f` | Min depth (metres) |
| `DEPTH_MAX` | `8.0f` | Max depth (metres) |
| `PC_STEP` | `3` | Point cloud publish downsample |

Suggested aggressive settings for the RTX 5060 8 GB:

```cpp
static constexpr float VOXEL_SIZE     = 0.03f;   // 3 cm — finer
static constexpr float TSDF_TRUNC     = 0.12f;   // 4× voxel
static constexpr int   BLOCK_COUNT    = 100000;  // ~750 MB VRAM, fine on 8 GB
static constexpr int   KEYFRAME_EVERY = 2;       // denser fusion
```

### Camera resolution

Default is HD720 (1280×720 @ 30 fps). Change in the constructor if needed:

```cpp
ip.camera_resolution = RESOLUTION::HD1080;  // 1920×1080 — sharper but heavier
ip.camera_fps        = 30;
```

---

## 12. Output Files

### zed_rviz
- `pointcloud_frame_N.csv` — saved every 10 frames. Columns: `X,Y,Z,R,G,B`

### zed_slam_map (saved on Ctrl+C shutdown)
- `final_map.ply` — coloured point cloud
- `final_map_mesh.ply` — triangle mesh
- `final_map.csv` — columns: `X,Y,Z,R_255,G_255,B_255,R_f,G_f,B_f`

### Opening final_map.csv in CloudCompare
1. File → Open → `final_map.csv`
2. ASCII wizard: tick "Contains header", separator = comma
3. Assign columns 1–3 as X/Y/Z, columns 4–6 as R/G/B (0–255 integers)

---

## 13. Troubleshooting

### nvidia-smi shows old driver or RTX 5060 not recognised

```bash
sudo apt remove --purge 'nvidia-*' && sudo apt autoremove -y
sudo add-apt-repository ppa:graphics-drivers/ppa -y
sudo apt update && sudo apt install -y nvidia-driver-575
sudo reboot
```

### nvcc not found after CUDA install

```bash
ls /usr/local/ | grep cuda
# Create the symlink if only cuda-12.8 exists:
sudo ln -sf /usr/local/cuda-12.8 /usr/local/cuda
export PATH=/usr/local/cuda/bin:$PATH
```

### ZED SDK installer says CUDA version mismatch

```bash
# Ensure nvcc shows 12.8 BEFORE running the installer
nvcc --version
export CUDA_HOME=/usr/local/cuda-12.8
export PATH=$CUDA_HOME/bin:$PATH
sudo ./ZED_SDK_Ubuntu24_cuda12_v4.2.run
```

### OpenCV ximgproc not found (build error or import error)

```bash
python3 -c "import cv2; cv2.ximgproc"
# Error means apt OpenCV is installed instead of the source build
sudo apt remove python3-opencv libopencv-dev -y
# Redo Section 6
```

### Open3D CUDA not available

```bash
python3 -c "import open3d; print(open3d.core.cuda.is_available())"
# False means pip wheel was used — it has no sm_120
pip3 uninstall open3d -y
# Redo Section 7 (source build)
```

### TSDF node prints "CUDA TSDF unavailable — CPU fallback"

Open3D's CUDA device init failed. Test directly:

```bash
python3 -c "
import open3d as o3d
dev = o3d.core.Device('CUDA:0')
print('OK:', dev)
"
```

If it throws, your Open3D build didn't compile sm_120 kernels. Rebuild with `-DCUDA_ARCH="12.0"`.

### GLFW window fails to open

```bash
sudo apt install -y libglfw3 libglew2.2 mesa-utils
glxinfo | grep "OpenGL version"
# On headless / SSH: export DISPLAY=:0
```

### ROS 2 "package not found" after build

```bash
# Always source Jazzy first, then build, then source the workspace
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --packages-select zed_rviz --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 run zed_rviz zed_slam_map
```

### Tracking lost / map drifts badly

- Move the camera slowly and smoothly — fast motion breaks visual odometry
- Good lighting and a textured environment are required (blank walls cause failure)
- The node sets `enable_imu_fusion=true` and `enable_area_memory=true` already
- If tracking is lost, restart the node — there is no in-code recovery

### Out of VRAM (very unlikely on 8 GB RTX 5060)

```cpp
// In zed_slam_map.cpp, reduce:
static constexpr int BLOCK_COUNT = 20000;  // drops to ~150 MB VRAM
```

---

## Quick-Reference: Full Install Order

```
1.  Ubuntu 24.04 LTS
2.  NVIDIA driver 575+        →  ppa:graphics-drivers/ppa
3.  CUDA Toolkit 12.8         →  cuda.nvidia.com repo for ubuntu2404
4.  ZED SDK 4.x               →  Ubuntu 24 / CUDA 12 build from stereolabs.com
5.  ROS 2 Jazzy               →  apt, ros2.list for noble
6.  OpenCV 4.10 from source   →  CUDA_ARCH_BIN="12.0", with contrib
7.  Open3D from source        →  main branch, CUDA_ARCH="12.0"
8.  Eigen3 / GLFW / GLEW / PCL  →  apt
9.  Create ~/ros2_ws, write package.xml, copy source files
10. colcon build              →  source jazzy first
11. source install/setup.bash
12. ros2 run zed_rviz zed_slam_map
```

Estimated build times on a modern system:
- OpenCV from source: ~30 minutes
- Open3D from source: ~75 minutes
- colcon build: ~5 minutes
