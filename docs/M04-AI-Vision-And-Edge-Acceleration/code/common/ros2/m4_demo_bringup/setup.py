from setuptools import find_packages, setup

package_name = 'm4_demo_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/m4_1_demo.launch.py', 'launch/m4_2_demo.launch.py',
            'launch/m4_3_demo.launch.py', 'launch/m4_all_demo.launch.py']),
        ('share/' + package_name + '/config', ['config/demo.yaml']),
        ('share/' + package_name + '/web/static', [
            'web/static/index.html', 'web/static/app.js', 'web/static/style.css']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bev-dev',
    maintainer_email='dev@jetson.local',
    description='M4 demo integration visualizers and demo launches',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'segmentation_visualizer = m4_demo_bringup.segmentation_visualizer:main',
        'detection_visualizer = m4_demo_bringup.detection_visualizer:main',
        'pose_visualizer = m4_demo_bringup.pose_visualizer:main',
        'wait_for_message_node = m4_demo_bringup.wait_for_message_node:main',
        'csi_camera_publisher = m4_demo_bringup.csi_camera_publisher:main',
        'm4_web_demo_server = m4_demo_bringup.web_demo_server:main',
        'video_file_publisher = m4_demo_bringup.video_file_publisher:main',
    ]},
)
