from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Fallback structural TF: uav1/base_link -> uav1/fcu  identity
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_link_fcu',
            arguments=[
                '0', '0', '0',
                '0', '0', '0',
                'uav1/base_link', 'uav1/fcu',
            ],
            output='screen',
        ),
    ])
