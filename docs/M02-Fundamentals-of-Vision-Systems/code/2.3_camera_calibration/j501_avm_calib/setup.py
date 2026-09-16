import os
from glob import glob

from setuptools import setup

package_name = 'j501_avm_calib'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'camera_info'),
         glob('config/camera_info/*.yaml')),
        (os.path.join('share', package_name, 'config', 'rviz'),
         glob('config/rviz/*.rviz')),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seeed',
    maintainer_email='seeed@todo.todo',
    description='J501 four-camera fisheye AVM calibration pipeline',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_driver = j501_avm_calib.camera_driver:main',
            'intrinsics_parse = j501_avm_calib.intrinsics_parse:main',
            'intrinsics_auto = j501_avm_calib.intrinsics_auto:main',
            'extrinsic_calibrator = j501_avm_calib.extrinsic_calibrator:main',
            'bev_publisher = j501_avm_calib.bev_publisher:main',
            'evaluator = j501_avm_calib.evaluator:main',
            'calib_pipeline = j501_avm_calib.calib_pipeline:main',
        ],
    },
)