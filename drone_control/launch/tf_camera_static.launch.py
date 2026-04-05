from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # uav1/fcu -> uav1/rgbd_down  (0.153, 0.0, -0.129), identity RPY
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_fcu_rgbd_down',
            arguments=[
                '0.153', '0.0', '-0.129',
                '0', '0', '0',
                'uav1/fcu', 'uav1/rgbd_down',
            ],
            output='screen',
        ),

        # uav1/fcu -> uav1/rgbd_front  (0.181, 0.0, -0.089), identity RPY
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_fcu_rgbd_front',
            arguments=[
                '0.181', '0.0', '-0.089',
                '0', '0', '0',
                'uav1/fcu', 'uav1/rgbd_front',
            ],
            output='screen',
        ),
    ])
