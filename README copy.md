# AIMS50_imgproc
ROS2-based image processing for cylinder 3D pose detection

# Required packages:

- Ubuntu server 22.04 base
- ROS Humble
- Intel RealSense SDK >= 2.54
- `sudo apt-get install ros-humble-librealsense2 ros-humble-diagnostic-updater ros-humble-xacro ros-humble-joint-state-publisher-gui ros-humble-joint-state-broadcaster`

# Usage/Installation

- Since these packages are wrapped to work in ROS2 environment, a ROS2 workspace needs to be created:
- `mkdir -p ~/img_proc_ws/src && cd ~/img_proc_ws`
- Clone the repository with all the submodules into an src folder:
- `git clone --recurse-submodules https://gitlab.com/Martzi/aims-img-proc.git src`
- Build the packages in the workspace:
- `colcon build`

# Testing

- Enter workspace:
- cd `~/img_proc_ws`
- Build containing packages:
- `colcon build`
- Source workspace:
- `. install/setup.bash`

## Visualizing camera:

- `ros2 launch realsense2_camera rs_launch.py`

## Image processing in camera stream:
