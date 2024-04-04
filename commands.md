## Starting D435i camera stream

ros2 launch realsense2_camera rs_launch.py align_depth:=true pointcloud.enable:=true serial_no:=_937622071880 depth_module.profile:=1280x720x30 rgb_camera.profile:=1280x720x30 enable_sync:=true align_depth.enable:=true pointcloud.ordered_pc:=true 
ros2 launch realsense2_camera rs_multi_camera_launch.py align_depth:=true pointcloud.enable:=true serial_no:=_937622071880 depth_module.profile:=1280x720x30 rgb_camera.profile:=1280x720x30 enable_sync:=true align_depth.enable:=true pointcloud.ordered_pc:=true

// serial_no:=_937622071880 (D435i)
// serial_no:=_117222250360 (D455)

## Starting Aruco detection

ros2 launch ros2_aruco aruco_recognition.launch.py

## YOLOv8

[https://github.com/mgonzs13/yolov8_ros](https://github.com/mgonzs13/yolov8_ros)

ros2 launch yolov8_bringup yolov8_3d.launch.py model:=yolov8m-seg.pt input_image_topic:=/camera/color/image_raw input_depth_topic:=/camera/depth/image_rect_raw input_depth_info_topic:=/camera/depth/metadata target_frame:=camera_link device:=cpu


ros2 launch yolov8_bringup yolov8.launch.py input_image_topic:=/camera/color/image_raw input_depth_topic:=/camera/depth/image_rect_raw input_depth_info_topic:=/camera/depth/metadata target_frame:=camera_link device:=cpu