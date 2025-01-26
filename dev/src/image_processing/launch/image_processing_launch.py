from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            parameters=[
                {"image_size": [640,480]},
            ]
        ),
        # Node 1
        Node(
            package='image_processing',
            executable='cube_detect',
            name='cube_detect_node',
            output='screen',
        ),
        # Node 2
        Node(
            package='image_processing',
            executable='cube_locate',
            name='cube_locate_node',
            output='screen',
        ),
    ])