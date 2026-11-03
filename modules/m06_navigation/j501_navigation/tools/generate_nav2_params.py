#!/usr/bin/python3
"""Adapt the installed Apache-2.0 Nav2 Humble parameter example to this chassis."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory

PACKAGE = Path(__file__).resolve().parents[1]


def generate():
    upstream = Path(get_package_share_directory('nav2_bringup')) / 'params/nav2_params.yaml'
    data = yaml.safe_load(upstream.read_text())
    amcl = data['amcl']['ros__parameters']
    amcl.update(alpha1=0.5, alpha2=0.3, alpha3=0.1, alpha4=0.2, max_beams=120,
                max_particles=1500, min_particles=500, laser_max_range=12.0, laser_min_range=0.1,
                sigma_hit=0.1, z_hit=0.8, z_rand=0.2, update_min_a=0.04, update_min_d=0.05,
                transform_tolerance=0.5, set_initial_pose=True,
                initial_pose={'x': -4.5, 'y': -3.0, 'z': 0.0, 'yaw': 0.0})
    bt = data['bt_navigator']['ros__parameters']
    bt.update(robot_base_frame='base_footprint', default_nav_to_pose_bt_xml='__BT_FILE__')
    control = data['controller_server']['ros__parameters']
    control.update(controller_frequency=15.0, failure_tolerance=2.0, controller_plugins=['DWB', 'MPPI', 'RPP'])
    control['progress_checker'].update(required_movement_radius=0.1, movement_time_allowance=25.0)
    control['general_goal_checker'].update(xy_goal_tolerance=0.18, yaw_goal_tolerance=0.35)
    dwb = control.pop('FollowPath')
    dwb.update(plugin='nav2_rotation_shim_controller::RotationShimController',
               primary_controller='dwb_core::DWBLocalPlanner', angular_dist_threshold=0.5,
               forward_sampling_distance=0.5, rotate_to_heading_angular_vel=0.4,
               max_angular_accel=1.0, simulate_ahead_time=1.5,
               xy_goal_tolerance=0.18,
               max_vel_x=0.25, max_speed_xy=0.25, max_vel_theta=0.6,
               acc_lim_x=0.5, acc_lim_theta=1.0, decel_lim_x=-0.5, decel_lim_theta=-1.0,
               vx_samples=15, vy_samples=1, vtheta_samples=21, sim_time=2.5,
               transform_tolerance=0.5, trans_stopped_velocity=0.03,
               debug_trajectory_details=False, **{'PathAlign.scale': 8.0, 'GoalAlign.scale': 8.0,
               'PathDist.scale': 16.0, 'GoalDist.scale': 12.0, 'ObstacleFootprint.scale': 1.0})
    dwb['critics'] = ['RotateToGoal', 'Oscillation', 'ObstacleFootprint', 'GoalAlign', 'PathAlign', 'PathDist', 'GoalDist']
    dwb.pop('BaseObstacle.scale', None)
    control['DWB'] = dwb
    control['MPPI'] = {
        'plugin': 'nav2_mppi_controller::MPPIController', 'time_steps': 80, 'model_dt': 0.0666667,
        'batch_size': 1000, 'iteration_count': 2, 'vx_std': 0.15, 'vy_std': 0.0, 'wz_std': 0.5,
        'vx_max': 0.25, 'vx_min': -0.05, 'vy_max': 0.0, 'wz_max': 0.6,
        'temperature': 0.3, 'gamma': 0.015, 'motion_model': 'DiffDrive',
        'transform_tolerance': 0.5, 'prune_distance': 3.0, 'visualize': False,
        'critics': ['ConstraintCritic', 'CostCritic', 'GoalCritic', 'GoalAngleCritic',
                    'PathAlignCritic', 'PathFollowCritic', 'PathAngleCritic', 'PreferForwardCritic'],
        'ConstraintCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 4.0},
        'CostCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 3.81, 'critical_cost': 253.0,
                       'consider_footprint': True, 'collision_cost': 1000000.0, 'near_goal_distance': 0.5,
                       'trajectory_point_step': 2},
        'GoalCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 5.0, 'threshold_to_consider': 1.4},
        'GoalAngleCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 3.0, 'threshold_to_consider': 0.5},
        'PathAlignCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 4.0, 'threshold_to_consider': 0.5,
                            'max_path_occupancy_ratio': 0.05, 'trajectory_point_step': 4, 'offset_from_furthest': 12},
        'PathFollowCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 5.0, 'threshold_to_consider': 1.4,
                             'offset_from_furthest': 6},
        'PathAngleCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 2.0, 'threshold_to_consider': 0.5},
        'PreferForwardCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 5.0, 'threshold_to_consider': 0.5},
    }
    control['RPP'] = {
        'plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
        'desired_linear_vel': 0.2, 'lookahead_dist': 0.6, 'min_lookahead_dist': 0.3,
        'max_lookahead_dist': 0.9, 'lookahead_time': 1.5, 'use_velocity_scaled_lookahead_dist': True,
        'use_collision_detection': True, 'max_allowed_time_to_collision_up_to_carrot': 1.5,
        'use_regulated_linear_velocity_scaling': True, 'use_cost_regulated_linear_velocity_scaling': True,
        'regulated_linear_scaling_min_radius': 0.6, 'regulated_linear_scaling_min_speed': 0.05,
        'use_rotate_to_heading': True, 'rotate_to_heading_angular_vel': 0.4,
        'rotate_to_heading_min_angle': 0.5, 'max_angular_accel': 1.0, 'transform_tolerance': 0.5,
        'min_approach_linear_velocity': 0.04, 'allow_reversing': False, 'inflation_cost_scaling_factor': 3.0,
    }
    footprint = '[[-0.245, -0.245], [-0.245, 0.245], [0.245, 0.245], [0.245, -0.245]]'
    for name in ['local_costmap', 'global_costmap']:
        costmap = data[name][name]['ros__parameters']
        costmap.pop('robot_radius', None)
        costmap.update(robot_base_frame='base_footprint', footprint=footprint, footprint_padding=0.025,
                       resolution=0.05, transform_tolerance=0.5, always_send_full_costmap=True)
        costmap['inflation_layer'].update(inflation_radius=0.65, cost_scaling_factor=3.0)
    local = data['local_costmap']['local_costmap']['ros__parameters']
    local.update(update_frequency=10.0, publish_frequency=5.0, width=5, height=5,
                 plugins=['static_layer', 'obstacle_layer', 'inflation_layer'])
    local.pop('voxel_layer', None)
    local['obstacle_layer'] = {
        'plugin': 'nav2_costmap_2d::ObstacleLayer', 'enabled': True, 'observation_sources': 'scan',
        'scan': {'topic': '/scan', 'max_obstacle_height': 1.5, 'clearing': True, 'marking': True,
                 'data_type': 'LaserScan', 'raytrace_max_range': 11.5, 'raytrace_min_range': 0.1,
                 'obstacle_max_range': 8.0, 'obstacle_min_range': 0.1, 'inf_is_valid': True},
    }
    global_map = data['global_costmap']['global_costmap']['ros__parameters']
    global_map.update(plugins=['static_layer', 'inflation_layer'], track_unknown_space=True)
    global_map.pop('obstacle_layer', None)
    planners = data['planner_server']['ros__parameters']
    planners.pop('GridBased', None)
    planners.update(expected_planner_frequency=2.0, planner_plugins=['AStar', 'Dijkstra', 'ThetaStar'])
    for name, astar in [('AStar', True), ('Dijkstra', False)]:
        planners[name] = {'plugin': 'nav2_navfn_planner/NavfnPlanner', 'use_astar': astar,
                          'tolerance': 0.2, 'allow_unknown': False}
    planners['ThetaStar'] = {'plugin': 'nav2_theta_star_planner/ThetaStarPlanner', 'how_many_corners': 8,
                             'w_euc_cost': 1.0, 'w_traversal_cost': 2.0, 'allow_unknown': False}
    data['behavior_server']['ros__parameters'].update(robot_base_frame='base_footprint',
        max_rotational_vel=0.6, min_rotational_vel=0.15, rotational_acc_lim=1.0, transform_tolerance=0.5)
    data['velocity_smoother']['ros__parameters'].update(max_velocity=[0.25, 0.0, 0.6],
        min_velocity=[-0.1, 0.0, -0.6], max_accel=[0.5, 0.0, 1.0], max_decel=[-0.5, 0.0, -1.0], velocity_timeout=0.5)
    (PACKAGE / 'config').mkdir(exist_ok=True)
    (PACKAGE / 'config/nav2_params.yaml').write_text(
        '# Based on nav2_bringup Humble nav2_params.yaml (Apache-2.0); J501 lab adaptations.\n'
        '# Global: static map + inflation. Local: static + live laser obstacles + inflation.\n'
        + yaml.safe_dump(data, sort_keys=False))


if __name__ == '__main__':
    generate()
