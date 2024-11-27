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


def generate_launch_description():
    lbr_dynamixel_controller_node_LF = Node(
        package='lbr_dynamixel_controller',
        executable='lbr_dynamixel_controller',
        name='lbr_dynamixel_controller',
        namespace='LF',
        output='screen',
    )

    lbr_dynamixel_controller_node_LH = Node(
        package='lbr_dynamixel_controller',
        executable='lbr_dynamixel_controller',
        name='lbr_dynamixel_controller',
        namespace='LH',
        output='screen',
    )

    lbr_dynamixel_controller_node_RH = Node(
        package='lbr_dynamixel_controller',
        executable='lbr_dynamixel_controller',
        name='lbr_dynamixel_controller',
        namespace='RH',
        output='screen',
    )

    lbr_dynamixel_controller_node_RF = Node(
        package='lbr_dynamixel_controller',
        executable='lbr_dynamixel_controller',
        name='lbr_dynamixel_controller',
        namespace='RF',
        output='screen',
    )

    return LaunchDescription([
        lbr_dynamixel_controller_node_LF,
        LogInfo(msg='spawning Dyanamixel controller for LF limb'),
        RegisterEventHandler(
            OnProcessStart(
                target_action=lbr_dynamixel_controller_node_LF,
                on_start=[
                    LogInfo(msg='spawning Dyanamixel controller for LH limb'),
                    lbr_dynamixel_controller_node_LH
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=lbr_dynamixel_controller_node_LH,
                on_start=[
                    LogInfo(msg='spawning Dyanamixel controller for RH limb'),
                    lbr_dynamixel_controller_node_RH
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=lbr_dynamixel_controller_node_RH,
                on_start=[
                    LogInfo(msg='spawning Dyanamixel controller for RF limb'),
                    lbr_dynamixel_controller_node_RF
                ]
            )
        )
    ])
