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
        output='screen'
    )

    return LaunchDescription([
        joy_node,
        RegisterEventHandler(
            OnProcessStart(
                target_action=joy_node,
                on_start=[
                    LogInfo(msg='joy_node started, spawning HLC.'),
                    lbr_high_level_controller_node
                ]
            )
        )
    ])
