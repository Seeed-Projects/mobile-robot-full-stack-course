from setuptools import find_packages, setup

package_name = 'bev_tracking'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name,
            ['package.xml']),
        ('share/' + package_name + '/config',
            ['config/bytetrack.yaml']),
        ('share/' + package_name + '/launch',
         ['launch/tracking.launch.py',
          'launch/tracking_with_detection.launch.py',
          'launch/tracking_demo.launch.py']),
    ],
    install_requires=[
        'setuptools',
        'supervision==0.27.0',
        'numpy',
    ],
    zip_safe=True,
    maintainer='bev-dev',
    maintainer_email='dev@jetson.local',
    description='ByteTrack multi-object tracking ROS2 node for M4.2 course',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tracking_node = bev_tracking.tracking_node:main',
            'mock_detection_publisher = bev_tracking.mock_detection_publisher:main',
            'tracking_visualizer = bev_tracking.tracking_visualizer:main',
        ],
    },
)
