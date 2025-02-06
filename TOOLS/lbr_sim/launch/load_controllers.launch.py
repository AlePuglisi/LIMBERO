#!/usr/bin/env python3
"""
Launch ROS 2 controllers for the robot.

This script creates a launch description that starts the necessary controllers
for operating the robotic arm and gripper in a specific sequence.

Launched Controllers:
    1. Joint State Broadcaster: Publishes joint states to /joint_states
    2. Arm Controller: Controls the robot arm movements via /follow_joint_trajectory
    3. Gripper Action Controller: Controls gripper actions via /gripper_action

Launch Sequence:
    1. Joint State Broadcaster
    2. Arm Controller (starts after Joint State Broadcaster)
    3. Gripper Action Controller (starts after Arm Controller)

:author: Addison Sears-Collins
:date: November 15, 2024
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """Generate a launch description for sequentially starting robot controllers.

    Returns:
        LaunchDescription: Launch description containing sequenced controller starts
    """
    # Start arm controller
    # start_arm_controller_cmd = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #          'arm_controller'],
    #     output='screen')

    # Start gripper action controller
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time',
                                      default_value='true')

    start_grieel_action_controller_LF_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'grieel_action_controller_LF'],
        output='screen')
    start_grieel_action_controller_LH_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'grieel_action_controller_LH'],
        output='screen')
    start_grieel_action_controller_RH_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'grieel_action_controller_RH'],
        output='screen')
    start_grieel_action_controller_RF_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'grieel_action_controller_RF'],
        output='screen')
    
    # Start arm controller
    start_drive_controller_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'drive_controller'],
        output='screen')
    
    start_limb_controller_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'limb_controller'],
        output='screen')

    # Launch joint state broadcaster
    start_joint_state_broadcaster_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen')

    # Add delay to joint state broadcaster (if necessary)
    delayed_start = TimerAction(
        period=10.0,
        actions=[start_joint_state_broadcaster_cmd]
    )

    lbr_state_estimator_node = Node(
        package='lbr_state_estimator',
        executable='lbr_state_estimator',
        name='lbr_state_estimator',
        output='screen', 
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]  
    )
    run_lbr_state_estimator_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_joint_state_broadcaster_cmd,
            on_exit=[lbr_state_estimator_node]))


    # Register event handlers for sequencing
    # Launch the joint state broadcaster after spawning the robot
    load_limb_controller_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_joint_state_broadcaster_cmd,
            on_exit=[start_limb_controller_cmd]))
    load_drive_controller_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_limb_controller_cmd,
            on_exit=[start_drive_controller_cmd]))

    # Launch the arm controller after launching the joint state broadcaster
    load_grieel_controller_LF_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_joint_state_broadcaster_cmd,
            on_exit=[start_grieel_action_controller_LF_cmd]))
    load_grieel_controller_LH_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_joint_state_broadcaster_cmd,
            on_exit=[start_grieel_action_controller_LH_cmd]))
    load_grieel_controller_RH_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_joint_state_broadcaster_cmd,
            on_exit=[start_grieel_action_controller_RH_cmd]))
    load_grieel_controller_RF_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_joint_state_broadcaster_cmd,
            on_exit=[start_grieel_action_controller_RF_cmd]))

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the actions to the launch description in sequence
    ld.add_action(use_sim_time_arg)
    ld.add_action(delayed_start)
    #ld.add_action(load_joint_state_broadcaster_cmd)
    ld.add_action(run_lbr_state_estimator_cmd)
    # ld.add_action(load_limb_controller_cmd)
    # ld.add_action(load_drive_controller_cmd)
    ld.add_action(load_grieel_controller_LF_cmd)
    ld.add_action(load_grieel_controller_LH_cmd)
    ld.add_action(load_grieel_controller_RH_cmd)
    ld.add_action(load_grieel_controller_RF_cmd)

    return ld