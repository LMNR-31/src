"""Launch odom_tf_broadcaster alongside pad_waypoint_supervisor."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    odom_tf = Node(
        package='yolo_pad_pose',
        executable='odom_tf_broadcaster',
        name='odom_tf_broadcaster',
        output='screen',
        parameters=[{
            'odom_topic': '/uav1/mavros/local_position/odom',
            'use_odom_header_stamp': True,
        }],
    )

    supervisor = Node(
        package='yolo_pad_pose',
        executable='pad_waypoint_supervisor',
        name='pad_waypoint_supervisor',
        output='screen',
    )

    return LaunchDescription([odom_tf, supervisor])
