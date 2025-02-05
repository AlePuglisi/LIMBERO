import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


launch_dir_path = os.path.dirname(os.path.realpath(__file__))
default_wheel_mode = 'true'

def generate_launch_description():

    wheel_mode_arg = DeclareLaunchArgument('wheel_mode',
                                      default_value = default_wheel_mode)
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', 
                                             default_value = 'true')
    
    lbr_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('lbr_bringup'), 'launch'),
            '/lbr_controller.launch.py']
        ),
        launch_arguments= {'wheel_mode' : LaunchConfiguration('wheel_mode'), 'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )

    simulation_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('lbr_sim'), 'launch'),
            '/lbr_sim.launch.py']
        ),
        launch_arguments = {'wheel_mode': LaunchConfiguration('wheel_mode'), 'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )

    return LaunchDescription([
        wheel_mode_arg,
        use_sim_time_arg,

        lbr_controller,
        simulation_nodes
    ])
