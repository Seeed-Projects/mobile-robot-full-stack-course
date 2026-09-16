from setuptools import find_packages, setup

package_name = 'pointcloud_utils'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'rgbd_to_pointcloud = pointcloud_utils.scripts.rgbd_to_pointcloud:main',
            'open3d_viewer = pointcloud_utils.scripts.open3d_pointcloud_viewer:main',
        ],
    },
)
