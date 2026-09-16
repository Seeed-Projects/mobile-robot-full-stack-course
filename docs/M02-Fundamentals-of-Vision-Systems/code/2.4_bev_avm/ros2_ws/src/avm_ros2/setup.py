from glob import glob
from setuptools import setup

package_name = 'avm_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bev-dev',
    maintainer_email='dev@jetson.local',
    description='Calibrated AVM ROS2 bridge and local visual parking map',
    license='MIT',
    entry_points={
        'console_scripts': [
            'avm_bev_node = avm_ros2.bev_node:main',
            'avm_local_map_node = avm_ros2.local_map_node:main',
            'avm_status_overlay_node = avm_ros2.status_overlay_node:main',
        ],
    },
)
