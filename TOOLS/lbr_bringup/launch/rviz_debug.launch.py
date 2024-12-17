import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

launch_dir_path = os.path.dirname(os.path.realpath(__file__))


def generate_launch_description():
    lbr_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            launch_dir_path + '/lbr_controller.launch.py']
        )
    )

    rviz_related_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('lbr_sim'), 'launch'),
            '/lbr_rviz.launch.py']
        )
    )

    return LaunchDescription([
        lbr_controller,
        rviz_related_nodes
    ])
