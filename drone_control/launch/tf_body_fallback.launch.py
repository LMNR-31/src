from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Structural fallback: uav1/base_link -> uav1/fcu
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="tf_base_link_to_fcu",
            arguments=[
                "0", "0", "0",    # x y z
                "0", "0", "0",    # roll pitch yaw
                "uav1/base_link", "uav1/fcu",
            ],
            output="screen",
        ),
    ])