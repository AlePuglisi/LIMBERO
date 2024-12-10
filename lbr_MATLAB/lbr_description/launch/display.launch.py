from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    lbr_description_path = get_package_share_path('lbr_description')
    if 0:
        default_model_path = lbr_description_path / 'urdf/Grieel/LIMBERO.xacro'
    else:
        grieel_model_path = lbr_description_path / 'urdf/Grieel/LIMBERO.xacro'
    default_rviz_config_path = lbr_description_path / 'config/lbr_monitor.rviz'
    
    default_wheel_mode = 'true'

    pkg_dir = get_package_share_directory("lbr_description")
    xacro_path = os.path.join(pkg_dir, "urdf", "Grieel/LIMBERO.xacro")

    gui_arg = DeclareLaunchArgument(
        name='gui', default_value='true', choices=['true', 'false'],
        description='Flag to enable joint_state_publisher_gui')
    model_arg = DeclareLaunchArgument(
        name='model', default_value=str(grieel_model_path),
        description='Absolute path to robot urdf file')
    rviz_arg = DeclareLaunchArgument(
        name='rviz_config', default_value=str(default_rviz_config_path),
        description='Absolute path to rviz config file')
    
    wheel_mode_arg = DeclareLaunchArgument('wheel_mode',
                                      default_value = default_wheel_mode)

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str)
    
    robot_description_command = ['xacro ', xacro_path, ' wheel_mode:=', LaunchConfiguration('wheel_mode')]

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(robot_description_command)}]
    )

    # Depending on gui parameter,
    # either launch joint_state_publisher or joint_state_publisher_gui
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )

    return LaunchDescription([
        gui_arg,
        model_arg,
        wheel_mode_arg,
        rviz_arg,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])
