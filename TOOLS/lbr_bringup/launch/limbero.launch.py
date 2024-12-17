import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, LogInfo, RegisterEventHandler)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LocalSubstitution
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

launch_dir_path = os.path.dirname(os.path.realpath(__file__))


def generate_launch_description():

    wheel_mode_arg = DeclareLaunchArgument('wheel_mode',
                                    default_value='true',
                                    description='grieel in wheel mode if true')

    lbr_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            launch_dir_path + '/lbr_controller.launch.py']
        ),
        launch_arguments= {'wheel_mode' : LaunchConfiguration('wheel_mode')}.items(),
    )

    lbr_dynamixel_controller_node_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('lbr_dynamixel_controller'), 'launch'),
            '/lbr_dynamixel_controller.launch.py']
        )
    )

    leptrino_FT_sensor_node_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('leptrino_force_torque'), 'launch'),
            '/multiple_leptrino.launch.py']
        )
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
        lbr_controller,
        lbr_dynamixel_controller_node_group,
        leptrino_FT_sensor_node_group,
        rviz_related_nodes,
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[LogInfo(
                    msg=['Launch was asked to shutdown: ',
                         LocalSubstitution('event.reason')]
                )]
            )
        )
    ])
