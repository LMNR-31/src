from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    supervisor_T = Node(
        package='drone_control',
        executable='supervisor_T',
        name='supervisor_T',
        output='screen',
    )
    return LaunchDescription([supervisor_T])
