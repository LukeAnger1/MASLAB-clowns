from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    number_publisher_node = Node(
        package='fizzbuzz',
        executable='number_publisher',
        output='screen'
    )

    fizzbuzz_subscriber_node = Node(
        package='fizzbuzz',
        executable='fizzbuzz_subscriber',
        output='screen'
    )

    return LaunchDescription([number_publisher_node, fizzbuzz_subscriber_node])