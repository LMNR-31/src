from setuptools import find_packages, setup

package_name = 'yolo_pad_pose'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/odom_tf_broadcaster.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lmnr31',
    maintainer_email='rasufsj@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
          'yolo_pad_pose = yolo_pad_pose.yolo_pad_pose_ros2:main',
          'capture_dataset = dataset_capture.capture_dataset:main',
          'pad_waypoint_supervisor = yolo_pad_pose.pad_waypoint_supervisor:main',
          'pad_waypoint_nn = yolo_pad_pose.base_waypoint_publisher:main',
          'yolo_mission_controller = yolo_pad_pose.yolo_mission_controller:main',
          'odom_tf_broadcaster = yolo_pad_pose.odom_tf_broadcaster:main',
        ],
    },
)
