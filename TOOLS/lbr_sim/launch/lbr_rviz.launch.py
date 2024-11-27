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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def generate_launch_description():
    # define file path
    pkg_dir = get_package_share_directory('lbr_description')
    xacro_path = os.path.join(pkg_dir, 'urdf', 'Grieel/LIMBERO.xacro')
    urdf_path = os.path.join(pkg_dir, 'urdf', 'limbero.urdf')

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

    rviz_arg = DeclareLaunchArgument(name='rviz_config',
                                     default_value=str(rviz_config_path))
    model_arg = DeclareLaunchArgument(name='model',
                                      default_value=str(model_path))

    robot_description = ParameterValue(Command(['xacro ',
                                                LaunchConfiguration('model')]),
                                       value_type=str)
    
    wheel_mode_arg = DeclareLaunchArgument('wheel_mode',
                                      default_value='true',
                                      description='grieel in wheel mode if true')
    
    robot_description_command = ['xacro ', xacro_path, ' wheel_mode:=', LaunchConfiguration('wheel_mode')]
    
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

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'source_list':
                    ['/lbr_state_estimator/joint_state']}]
    )

    return LaunchDescription([
        wheel_mode_arg,
        model_arg,
        rviz_arg,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
        lbr_sim_node,
    ])
