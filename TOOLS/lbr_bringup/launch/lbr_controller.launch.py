import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, LogInfo, RegisterEventHandler)
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration



def generate_launch_description():

    wheel_mode_arg = DeclareLaunchArgument('wheel_mode',
                                      default_value='true',
                                      description='grieel in wheel mode if true')
    joy_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joy_node',
        output='screen'
    )

    lbr_high_level_controller_node = Node(
        package='lbr_high_level_controller',
        executable='lbr_high_level_controller',
        name='lbr_high_level_controller',
        output='screen',
        parameters=[{'wheel_mode': LaunchConfiguration('wheel_mode')}]
    )

    lbr_low_level_controller_node = Node(
        package='lbr_low_level_controller',
        executable='lbr_low_level_controller',
        name='lbr_low_level_controller',
        output='screen'
    )

    lbr_limb_controller_node_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('lbr_limb_controller'), 'launch'),
            '/lbr_limb_controller.launch.py']
        ),
        launch_arguments= {'wheel_mode' : LaunchConfiguration('wheel_mode')}.items(),
    )

    lbr_indipendent_joint_controller_node_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('lbr_indipendent_joint_controller'), 'launch'),
            '/lbr_indipendent_joint_controller.launch.py']
        ),
        launch_arguments= {'wheel_mode' : LaunchConfiguration('wheel_mode')}.items(),
    )

    lbr_state_estimator_node = Node(
        package='lbr_state_estimator',
        executable='lbr_state_estimator',
        name='lbr_state_estimator',
        output='screen'
    )

    return LaunchDescription([
        joy_node,
        lbr_high_level_controller_node,
        RegisterEventHandler(
            OnProcessStart(
                target_action=lbr_high_level_controller_node,
                on_start=[
                    LogInfo(msg='HLC started, spawning LLC'),
                    lbr_low_level_controller_node
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=lbr_low_level_controller_node,
                on_start=[
                    lbr_limb_controller_node_group
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=lbr_low_level_controller_node,
                on_start=[
                    lbr_indipendent_joint_controller_node_group
                ]
            )
        ),
        lbr_state_estimator_node,
        wheel_mode_arg,
    ])
