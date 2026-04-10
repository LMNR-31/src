from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # uav1/fcu -> uav1/rgbd_down
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="tf_rgbd_down",
            arguments=[
                "0.153", "0.0", "-0.129",  # x y z
                "0.0", "0.0", "0.0",       # roll pitch yaw
                "uav1/fcu", "uav1/rgbd_down",
            ],
            output="screen",
        ),

        # uav1/fcu -> uav1/rgbd_front
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="tf_rgbd_front",
            arguments=[
                "0.181", "0.0", "-0.089",  # x y z
                "0.0", "0.0", "0.0",       # roll pitch yaw
                "uav1/fcu", "uav1/rgbd_front",
            ],
            output="screen",
        ),
    ])