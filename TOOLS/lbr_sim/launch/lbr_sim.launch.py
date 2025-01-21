# Copyright (c) 2023 Tohoku Univ. Space Robotics Lab.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os

from ament_index_python.packages import (
    get_package_share_directory, get_package_share_path)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
import xacro


def generate_launch_description():
    # define file path
    pkg_dir = get_package_share_directory("lbr_description")
    xacro_path = os.path.join(pkg_dir, "urdf", "Grieel/LIMBERO.xacro")
    urdf_path = os.path.join(pkg_dir, "urdf", "limbero.urdf")

    # make urdf from xacro
    # load xacro
    doc = xacro.process_file(xacro_path)
    # make urdf
    robot_desc = doc.toprettyxml(indent=' ')
    # export urdf to urdf path
    f = open(urdf_path, 'w')
    f.write(robot_desc)
    f.close()


    lbr_sim_path = get_package_share_path('lbr_sim')

    model_path = urdf_path
    rviz_config_path = lbr_sim_path / 'config/lbr_sim.rviz'

    # xacro if/unless for wheel/gripper

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    rviz_arg = DeclareLaunchArgument(name='rviz_config',
                                     default_value=str(rviz_config_path))
    model_arg = DeclareLaunchArgument(name='model',
                                      default_value=str(model_path))

    wheel_mode_arg = DeclareLaunchArgument('wheel_mode',
                                      default_value='true',
                                      description='grieel in wheel mode if true')

    robot_description = ParameterValue(Command(['xacro ',
                                                LaunchConfiguration('model'), ' wheel_mode:=', LaunchConfiguration('wheel_mode')]),
                                       value_type=str)

    # world_file_name = "mars.world"
    # world_path = os.path.join(get_package_share_directory('lbr_sim'), 'worlds', world_file_name)

    robot_description_command = ['xacro ', xacro_path, ' wheel_mode:=', LaunchConfiguration('wheel_mode')]
    # somehow if I pass the xacro_path it works, insted by using the model of the launch configuration the wheel_mode doesn't work..

    default_world = os.path.join(
        get_package_share_directory('lbr_sim'),
        'worlds',
        'empty.world'
        )    
    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
        )
    
    lbr_sim_node = Node(
        package='lbr_sim',
        executable='lbr_sim',
        name='lbr_sim',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(robot_description_command)}]
    )

    kin_analysis_node = Node(
        package='kin_analysis',
        executable='kin_analysis',
        name='kin_analysis',
        output='screen',
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'source_list':
                    ['/lbr_state_estimator/joint_state']}]
    )

    # robot_controllers = PathJoinSubstitution(
    #     [
    #         FindPackageShare("lbr_description"),
    #         "config",
    #         "lbr_controller_with_grieel.yaml",
    #     ]
    # )

    # Start controller manager
    # controller_manager_node = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     name='controller_manager',
    #     parameters=[{'robot_description': Command(robot_description_command)}],
    #     output='screen'
    # )

    # robot_controller_spawner_joint = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     # arguments=['joint_trajectory_controller', '--controller-manager',
    #     #            '/controller_manager'],
    #     arguments=["joint_trajectory_controller", "--param-file", robot_controllers],
    # )

    # robot_controller_spawner_effort = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     # arguments=['joint_trajectory_controller', '--controller-manager',
    #     #            '/controller_manager'],
    #     arguments=["effort_controller", "--param-file", robot_controllers],
    # )

    # effort_controller_spawner = Node(
    # package='controller_manager',
    # executable='spawner',
    # arguments=['effort_controller', '--controller-manager',
    #                '/controller_manager'],
    # )

    # joint_state_broadcaster_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['joint_state_broadcaster', "--param-file", robot_controllers],
    # )

    # Event handler to launch joint trajectory controller after joint state broadcaster
    # joint_controller_handler = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_state_broadcaster_spawner,
    #         on_exit=[robot_controller_spawner_joint]
    #     )
    # )

    # Event handler to launch effort controller after joint trajectory controller
    # effort_controller_handler = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_state_broadcaster_spawner,
    #         on_exit=[robot_controller_spawner_effort]
    #     )
    # )

    gazebo_spawner = Node(
        package='ros_gz_sim',
        executable='create',
        name='urdf_spawner',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-topic', 'robot_description',
                   '-name', 'LIMBERO',
                   '-z', '0.5'],
                #    '-z', '3.5',
                #    '-x', '-3',
                #    '-y', '8'],
        output='screen'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'),
            '/gz_sim.launch.py']),
        launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    bridge_params = os.path.join(get_package_share_directory('lbr_description'),'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    return LaunchDescription([
        model_arg,
        world_arg,
        rviz_arg,
        wheel_mode_arg,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
        lbr_sim_node,
        #joint_state_broadcaster_spawner,
        #robot_controller_spawner_effort,
        #robot_controller_spawner_effort,
        gazebo,
        ros_gz_bridge,
        gazebo_spawner,
        kin_analysis_node,
    ])
