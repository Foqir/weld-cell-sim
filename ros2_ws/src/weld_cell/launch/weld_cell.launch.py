#!/usr/bin/env python3
"""Robotic arc-welding cell: Gazebo world + HD Hyundai robot with torch + ros2_control + MoveIt 2 + RViz.

  ros2 launch weld_cell weld_cell.launch.py [robot_model:=hdr20_17] [headless:=false] [rviz:=true]
                                            [part_dx:=0.0 part_dy:=0.0 part_dyaw:=0.0]

part_d*: displacement of the REAL part in the fixture (m, deg) relative to the nominal pose the
weld program was written for - used to study the effect of part tolerances on the weld path.
"""
import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from weld_cell import geometry


def load_yaml(package, rel_path):
    with open(os.path.join(get_package_share_directory(package), rel_path)) as f:
        return yaml.safe_load(f)


def launch_setup(context, *args, **kwargs):
    robot_model = LaunchConfiguration('robot_model').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    use_rviz = LaunchConfiguration('rviz').perform(context).lower() == 'true'
    offset = tuple(float(LaunchConfiguration(n).perform(context)) for n in ('part_dx', 'part_dy', 'part_dyaw'))

    pkg = get_package_share_directory('weld_cell')
    moveit_pkg = f'{robot_model}_moveit_config'
    moveit_share = get_package_share_directory(moveit_pkg)

    # --- Gazebo world generated from the cell layout (actual part pose = nominal + offset)
    cell = geometry.load_yaml(os.path.join(pkg, 'config', 'cell.yaml'))
    world_path = '/tmp/weld_cell_world.sdf'
    with open(world_path, 'w') as f:
        f.write(geometry.world_sdf(cell, offset))

    # --- robot description (robot + torch + gz_ros2_control) and semantic description
    robot_description = xacro.process_file(
        os.path.join(pkg, 'urdf', 'weld_cell.urdf.xacro'),
        mappings={'robot_model': robot_model, 'name': 'hdr_robot', 'use_sim': 'true',
                  'controllers_file': os.path.join(pkg, 'config', 'controllers.yaml'),
                  'initial_positions_file': os.path.join(pkg, 'config', 'initial_positions.yaml')},
    ).toxml()
    robot_description_semantic = xacro.process_file(
        os.path.join(pkg, 'srdf', 'weld_cell.srdf.xacro'),
        mappings={'name': 'hdr_robot',
                  'model_srdf': os.path.join(moveit_share, 'config', f'{robot_model}.srdf.xacro')},
    ).toxml()

    common = {'use_sim_time': True}
    rd = {'robot_description': robot_description}
    rds = {'robot_description_semantic': robot_description_semantic}
    kinematics = {'robot_description_kinematics': load_yaml('weld_cell', 'config/kinematics.yaml')}
    planning = {'robot_description_planning': {
        **load_yaml(moveit_pkg, 'config/joint_limits.yaml'),
        **load_yaml(moveit_pkg, 'config/pilz_cartesian_limits.yaml')}}
    ompl = load_yaml(moveit_pkg, 'config/ompl_planning.yaml')
    ompl['welder'] = dict(ompl.get('hdr_manipulator', {}))
    # the rib of the part is only 4 mm thick: check OMPL motions for collisions more densely
    ompl['welder']['longest_valid_segment_fraction'] = 0.002
    pipelines = {
        'default_planning_pipeline': 'ompl',
        'planning_pipelines': ['ompl', 'pilz'],
        'ompl': ompl,
        'pilz': load_yaml(moveit_pkg, 'config/pilz_industrial_motion_planner_planning.yaml'),
    }

    gz_args = f'-r -v 2 {world_path}'
    if headless:
        gz_args = '-s ' + gz_args
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items(),
    )

    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='log',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='log',
        parameters=[rd, common],
    )

    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description', '-name', 'hdr_robot', '-z', '0.0'],
    )

    def spawner(name):
        return Node(package='controller_manager', executable='spawner', output='screen',
                    arguments=[name, '--controller-manager', '/controller_manager',
                               '--controller-manager-timeout', '120'],
                    parameters=[common])

    controllers = RegisterEventHandler(OnProcessExit(
        target_action=spawn,
        on_exit=[spawner('joint_state_broadcaster'), spawner('joint_trajectory_controller')],
    ))

    move_group = Node(
        package='moveit_ros_move_group', executable='move_group', output='log',
        parameters=[
            rd, rds, kinematics, planning, pipelines, common,
            {'publish_robot_description_semantic': True},
            {'moveit_simple_controller_manager': load_yaml(moveit_pkg, 'config/moveit_controllers.yaml'),
             'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'},
            {'moveit_manage_controllers': False,
             'trajectory_execution.allowed_execution_duration_scaling': 1.5,
             'trajectory_execution.allowed_goal_duration_margin': 1.0,
             'trajectory_execution.allowed_start_tolerance': 0.02,
             'trajectory_execution.execution_duration_monitoring': False},
            {'publish_planning_scene': True, 'publish_geometry_updates': True,
             'publish_state_updates': True, 'publish_transforms_updates': True},
        ],
    )

    actions = [gazebo, clock_bridge, robot_state_publisher, spawn, controllers,
               TimerAction(period=5.0, actions=[move_group])]

    if use_rviz:
        actions.append(TimerAction(period=8.0, actions=[Node(
            package='rviz2', executable='rviz2', name='rviz2', output='log',
            arguments=['-d', os.path.join(pkg, 'rviz', 'weld_cell.rviz')],
            parameters=[rd, rds, kinematics, planning, pipelines, common],
        )]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_model', default_value='hdr20_17',
                              description='HD Hyundai Robotics model (hdr20_17, hh020, ...)'),
        DeclareLaunchArgument('headless', default_value='false', description='Gazebo without GUI'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument('part_dx', default_value='0.0', description='Real part shift X, m'),
        DeclareLaunchArgument('part_dy', default_value='0.0', description='Real part shift Y, m'),
        DeclareLaunchArgument('part_dyaw', default_value='0.0', description='Real part rotation, deg'),
        OpaqueFunction(function=launch_setup),
    ])
