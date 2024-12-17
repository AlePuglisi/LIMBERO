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

limb_name = 'RH'


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

    lbr_limb_controller_node = Node(
        package='lbr_limb_controller',
        executable='lbr_limb_controller',
        name='lbr_limb_controller',
        namespace=limb_name,
        output='screen',
        parameters=[{'wheel_mode': LaunchConfiguration('wheel_mode')}]
    )

    lbr_dynamixel_controller_node = Node(
        package='lbr_dynamixel_controller',
        executable='lbr_dynamixel_controller',
        name='lbr_dynamixel_controller',
        namespace=limb_name,
        output='screen',
    )

    lbr_state_estimator_node = Node(
        package='lbr_state_estimator',
        executable='lbr_state_estimator',
        name='lbr_state_estimator',
        output='screen'
    )

    leptrino_node = Node(
        package="leptrino_force_torque",
        executable="leptrino_force_torque",
        namespace=limb_name,
        name='leptrino_force_torque',
        output="screen"
    )

    rviz_related_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('lbr_sim'), 'launch'),
            '/lbr_rviz.launch.py']
        ),
        launch_arguments= {'wheel_mode' : LaunchConfiguration('wheel_mode')}.items(),
    )

    return LaunchDescription([
        wheel_mode_arg,
        joy_node,
        leptrino_node,
        rviz_related_nodes,
        lbr_high_level_controller_node,
        RegisterEventHandler(
            OnProcessStart(
                target_action=lbr_high_level_controller_node,
                on_start=[
                    LogInfo(msg='HLC started, spawning LLC.'),
                    lbr_low_level_controller_node
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=lbr_low_level_controller_node,
                on_start=[
                    LogInfo(msg='LLC started, spawning LC.'),
                    lbr_limb_controller_node
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=lbr_limb_controller_node,
                on_start=[
                    LogInfo(msg='LC started, spawning Dynamixel controller.'),
                    lbr_dynamixel_controller_node
                ]
            )
        ),
        lbr_state_estimator_node,
    ])
