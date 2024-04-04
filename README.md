# ros2_cam_control
Camera control for RealSense cameras in a containerized environment

(for NUC)


# usage


```shell
git clone https://github.com/hsnlab-iot/ros2_cam_control.git camcontrol_ws --recurse-submodules
```


compile dockerfile:
```shell
./build.sh
```
start dockerfile:
```shell
./run.sh
```

# features
- publish realsense camera stream(s)
- aruco detection