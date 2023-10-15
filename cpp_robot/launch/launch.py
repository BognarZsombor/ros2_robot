from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot',
            namespace='robot',
            executable='robot_node',
            name='robot'
        ),
        Node(
            package='robot',
            namespace='robot',
            executable='rviz_pose',
            name='destination'
        )
    ])