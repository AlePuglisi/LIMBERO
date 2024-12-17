import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

launch_dir_path = os.path.dirname(os.path.realpath(__file__))
default_wheel_mode = 'true'

#default_data_analysis = 'false'
#data_analysis = False

def generate_launch_description():

    wheel_mode_arg = DeclareLaunchArgument('wheel_mode',
                                      default_value = default_wheel_mode)
    
    #data_analysis_arg = DeclareLaunchArgument('data_analysis',
    #                                  default_value = default_data_analysis)
    
    #data_analysis = LaunchConfiguration('data_analysis')

    analysis_node = Node(
        package='lbr_data_analysis',
        executable='lbr_data_analysis',
        name='lbr_data_analysis',
        output='screen'
    )

    lbr_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            launch_dir_path + '/lbr_controller.launch.py']
        ),
        launch_arguments= {'wheel_mode' : LaunchConfiguration('wheel_mode')}.items(),
    )

    simulation_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('lbr_sim'), 'launch'),
            '/lbr_sim.launch.py']
        ),
        launch_arguments = {'wheel_mode': LaunchConfiguration('wheel_mode')}.items(),
    )


    return LaunchDescription([
        analysis_node,
        wheel_mode_arg,
        #data_analysis_arg,
        lbr_controller,
        simulation_nodes,
    ])
