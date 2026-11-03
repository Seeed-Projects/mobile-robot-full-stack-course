"""Survey with SLAM; reload the measured map, relocalize, then activate Nav2."""
from pathlib import Path
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    share = Path(get_package_share_directory('j501_navigation'))
    chassis = Path(get_package_share_directory('j501_chassis_description'))
    nav2 = Path(get_package_share_directory('nav2_bringup'))
    mode = LaunchConfiguration('mode').perform(context)
    lidar = LaunchConfiguration('lidar').perform(context)
    odometry_source = LaunchConfiguration('odometry_source').perform(context)
    demo = LaunchConfiguration('demo_map').perform(context).lower() == 'true'
    planner = LaunchConfiguration('planner').perform(context)
    controller = LaunchConfiguration('controller').perform(context).upper()
    planners = {'astar': 'AStar', 'dijkstra': 'Dijkstra', 'theta_star': 'ThetaStar'}
    if mode not in ('mapping', 'localization', 'navigation') or lidar not in ('2d', '3d') or odometry_source not in ('wheel', 'ekf'):
        raise ValueError('Choose mode:=mapping|localization|navigation and lidar:=2d|3d')
    if planner not in planners or controller not in ('DWB', 'MPPI', 'RPP'):
        raise ValueError('Choose planner:=astar|dijkstra|theta_star and controller:=DWB|MPPI|RPP')
    map_file = Path(LaunchConfiguration('map').perform(context)).expanduser().resolve()
    if demo:
        map_file = share / 'maps/office.yaml'
    if mode != 'mapping' and not map_file.is_file():
        raise FileNotFoundError('No measured map. Run run_mapping.sh, survey, and save_map first; '
                                'or pass map:=/absolute/path/office.yaml. demo_map:=true is a prebuilt-map algorithm benchmark only.')
    scene = yaml.safe_load((share / 'scenes/office.yaml').read_text())
    x, y, heading = scene['initial_pose']
    runtime = Path(tempfile.mkdtemp(prefix='j501-nav2-'))
    rviz = yaml.safe_load((share / 'rviz/navigation.rviz').read_text())
    if demo:
        rviz['Visualization Manager']['Views']['Current'].update(X=0., Y=0.)
    (runtime / 'view.rviz').write_text(yaml.safe_dump(rviz, sort_keys=False))
    actions = [
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', os.pathsep.join(filter(None, [
            str(share / 'models'), context.environment.get('IGN_GAZEBO_RESOURCE_PATH', '')]))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(chassis / 'launch/sim.launch.py')),
            launch_arguments={'world': str(share / ('worlds/office_mapping.sdf' if mode == 'mapping' else 'worlds/office.sdf')),
                'x': str(x), 'y': str(y), 'yaw': str(heading), 'lidar': lidar,
                'odometry_source': odometry_source,
                'headless': LaunchConfiguration('headless'), 'rviz': LaunchConfiguration('rviz'),
                'rviz_config': str(runtime / 'view.rviz')}.items()),
    ]
    if odometry_source == 'ekf':
        actions.append(Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
            output='screen', parameters=[str(share / 'config/ekf.yaml')], remappings=[('odometry/filtered', '/odom')]))
    if lidar == '3d':
        actions.append(Node(package='j501_navigation', executable='lidar_perception', output='screen',
                            parameters=[str(share / 'config/lidar_perception.yaml')]))
    if mode == 'mapping':
        actions.append(LogInfo(msg='MAPPING: SLAM only; navigation is unavailable. After survey, run save_map, '
            'Ctrl+C this launch, start run_navigation.sh, then run relocalize.'))
        actions.append(Node(package='slam_toolbox', executable='async_slam_toolbox_node',
            name='slam_toolbox', parameters=[str(share / 'config/slam.yaml')], output='screen'))
        return actions
    tree = (share / 'behavior_trees/navigate.xml').read_text()
    tree = tree.replace('default_planner="AStar"', f'default_planner="{planners[planner]}"')
    tree = tree.replace('default_controller="DWB"', f'default_controller="{controller}"')
    (runtime / 'navigate.xml').write_text(tree)
    params = yaml.safe_load((share / 'config/nav2_params.yaml').read_text())
    params['amcl']['ros__parameters']['set_initial_pose'] = demo
    if demo:
        params['amcl']['ros__parameters']['initial_pose'] = dict(x=float(x), y=float(y), z=0.0, yaw=float(heading))
    if lidar == '3d':
        obstacle_layer = params['local_costmap']['local_costmap']['ros__parameters']['obstacle_layer']
        obstacle_layer['observation_sources'] = 'scan cloud'
        obstacle_layer['scan']['marking'] = False
        obstacle_layer['cloud'] = dict(topic='/perception/obstacles', data_type='PointCloud2',
            marking=True, clearing=False, min_obstacle_height=0.07, max_obstacle_height=1.5,
            obstacle_max_range=8.0, obstacle_min_range=0.35)
    params['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = str(runtime / 'navigate.xml')
    params_file = runtime / 'params.yaml'
    params_file.write_text(yaml.safe_dump(params, sort_keys=False))
    common = dict(map=str(map_file), params_file=str(params_file), use_sim_time='true',
                  use_composition='False', log_level=LaunchConfiguration('log_level'))
    actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(nav2 / 'launch/localization_launch.py')),
        launch_arguments={**common, 'autostart': 'true'}.items()))
    if mode == 'navigation':
        if not demo:
            actions.append(LogInfo(msg='NAVIGATION: waiting for AMCL. Run: '
                'ros2 run j501_navigation relocalize --pose 0 0 0 . Nav2 activates after localization checks pass.'))
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(nav2 / 'launch/navigation_launch.py')),
            launch_arguments={**common, 'autostart': 'true' if demo else 'false'}.items()))
    actions.append(Node(package='j501_navigation', executable='localization_monitor', output='screen',
        parameters=[{'use_sim_time': True, 'start_navigation': mode == 'navigation' and not demo,
                     'demo_map': demo, 'metadata_file': str(map_file.with_suffix('.metadata.json'))}]))
    if LaunchConfiguration('algorithm_view').perform(context).lower() == 'true':
        actions.append(Node(package='j501_navigation', executable='algorithm_view', output='screen',
            parameters=[{'use_sim_time': True, 'output': LaunchConfiguration('view_output')}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='navigation'),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('demo_map', default_value='false'),
        DeclareLaunchArgument('lidar', default_value='3d'),
        DeclareLaunchArgument('odometry_source', default_value='ekf'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('planner', default_value='astar'),
        DeclareLaunchArgument('controller', default_value='MPPI'),
        DeclareLaunchArgument('log_level', default_value='warn'),
        DeclareLaunchArgument('algorithm_view', default_value='true'),
        DeclareLaunchArgument('view_output', default_value='build/m06_results/algorithm_view'),
        OpaqueFunction(function=start),
    ])
