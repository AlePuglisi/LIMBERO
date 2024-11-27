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

from launch import LaunchDescription
from launch.actions import (LogInfo, RegisterEventHandler)
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    wheel_mode_arg = DeclareLaunchArgument('wheel_mode',
                                      default_value='true',
                                      description='grieel in wheel mode if true')

    lbr_limb_controller_node_LF = Node(
        package='lbr_limb_controller',
        executable='lbr_limb_controller',
        namespace='LF',
        name='lbr_limb_controller',
        output='screen',
        parameters=[{'wheel_mode': LaunchConfiguration('wheel_mode')}]
    )

    lbr_limb_controller_node_LH = Node(
        package='lbr_limb_controller',
        executable='lbr_limb_controller',
        namespace='LH',
        name='lbr_limb_controller',
        output='screen',
        parameters=[{'wheel_mode': LaunchConfiguration('wheel_mode')}]
    )

    lbr_limb_controller_node_RH = Node(
        package='lbr_limb_controller',
        executable='lbr_limb_controller',
        namespace='RH',
        name='lbr_limb_controller',
        output='screen',
        parameters=[{'wheel_mode': LaunchConfiguration('wheel_mode')}]
    )

    lbr_limb_controller_node_RF = Node(
        package='lbr_limb_controller',
        executable='lbr_limb_controller',
        namespace='RF',
        name='lbr_limb_controller',
        output='screen',
        parameters=[{'wheel_mode': LaunchConfiguration('wheel_mode')}]
    )

    return LaunchDescription([
        wheel_mode_arg,
        lbr_limb_controller_node_LF,
        RegisterEventHandler(
            OnProcessStart(
                target_action=lbr_limb_controller_node_LF,
                on_start=[
                    LogInfo(msg='LC for LF started, start LC for LH limb.'),
                    lbr_limb_controller_node_LH
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=lbr_limb_controller_node_LH,
                on_start=[
                    LogInfo(msg='LC for LH started, start LC for RH limb.'),
                    lbr_limb_controller_node_RH
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=lbr_limb_controller_node_RH,
                on_start=[
                    LogInfo(msg='LC for RH started, start LC for RF limb.'),
                    lbr_limb_controller_node_RF
                ]
            )
        )
    ])
