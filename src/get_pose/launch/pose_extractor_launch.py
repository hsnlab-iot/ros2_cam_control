from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='realsense2_camera',
        #     namespace='',
        #     executable='realsense2_camera_node',
        #     name='sim'
        # ),
        Node(
            package='get_pose',
            namespace='',
            executable='get_pose',
            name='pose_extractor_node',
            output='screen'
        )
    ])