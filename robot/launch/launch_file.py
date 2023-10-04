from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='keyboard_input_publisher',
            namespace='keyboard',
            executable='talker',
            name='input'
        ),
        Node(
            package='robot',
            namespace='robot',
            executable='listener',
            name='robot'
        )
    ])