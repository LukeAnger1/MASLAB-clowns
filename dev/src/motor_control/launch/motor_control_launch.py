from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node 1
        Node(
            package='motor_control',
            executable='map_generator',
            name='map_generator_node',
            output='screen',
        ),
        # Node 2
        Node(
            package='motor_control',
            executable='decision_node',
            name='decision_node',
            output='screen',
        ),
        # Node 3
        Node(
            package='motor_control',
            executable='drive_node',
            name='drive_node',
            output='screen',
        ),
    ])