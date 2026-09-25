# setup.py — bev_pose (ament_python)
#
# Entry points:
#   foundationpose_node  — ROS2 node: RGB+Depth+CameraInfo+object_mask → /perception/object_pose
#   object_mask_node     — ROS2 node: ROI service + depth threshold → /perception/object_mask
#   mesh_preprocessor    — CLI: offline OBJ → processed mesh .npz

from glob import glob
import os

from setuptools import setup

PACKAGE_NAME = 'bev_pose'

setup(
    name=PACKAGE_NAME,
    version='0.1.0',
    packages=[PACKAGE_NAME],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        # launch + config files
        (os.path.join('share', PACKAGE_NAME, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', PACKAGE_NAME, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seeed',
    maintainer_email='seeed@seeed.local',
    description='6D pose estimation via NVlabs/FoundationPose for the M4 perception stack.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'foundationpose_node = bev_pose.foundationpose_node:main',
            'object_mask_node = bev_pose.object_mask_node:main',
            'mesh_preprocessor = bev_pose.mesh_preprocessor:main',
        ],
    },
)
