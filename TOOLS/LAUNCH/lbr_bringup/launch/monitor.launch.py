from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
# from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    lbr_description_path = get_package_share_path('lbr_description')

    model_path = lbr_description_path / 'urdf/slt_description.urdf'
    rviz_config_path = lbr_description_path / 'config/lbr_monitor.rviz'

    rviz_arg = DeclareLaunchArgument(name='rvizconfig',
                                     default_value=str(rviz_config_path),
                                     description='Absolute path to rviz config file')
    model_arg = DeclareLaunchArgument(name='model',
                                      default_value=str(model_path),
                                      description='Absolute path to robot urdf file')

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        # Visualize based on input angles
        parameters=[{'source_list': ['LH/lbr_limb_controller/joint_state']}]
        # Visualize based on Dynamixel encoder feecbacks
        # parameters=[{'source_list': ['LH/lbr_dynamixel_controller/encorder_joint_state']}]
    )

    return LaunchDescription([
        model_arg,
        rviz_arg,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
    ])
