## Starting D435i camera stream

ros2 launch realsense2_camera rs_launch.py align_depth:=true pointcloud.enable:=true serial_no:=937622071880

// serial_no:=_937622071880 (D435i)
// serial_no:=_117222250360 (D455)

## Starting Aruco detection

ros2 launch ros2_aruco aruco_recognition.launch.py