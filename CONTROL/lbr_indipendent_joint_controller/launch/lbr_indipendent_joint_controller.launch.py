
from launch import LaunchDescription
from launch.actions import (LogInfo, RegisterEventHandler)
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # wheel_mode_arg = DeclareLaunchArgument('wheel_mode',
    #                                   default_value='true',
    #                                   description='grieel in wheel mode if true')

    lbr_indipendent_joint_controller_node_LF = Node(
        package='lbr_indipendent_joint_controller',
        executable='lbr_indipendent_joint_controller',
        namespace='LF',
        name='lbr_indipendent_joint_controller',
        output='screen',
        #parameters=[{'wheel_mode': LaunchConfiguration('wheel_mode')}]
    )

    lbr_indipendent_joint_controller_node_LH = Node(
        package='lbr_indipendent_joint_controller',
        executable='lbr_indipendent_joint_controller',
        namespace='LH',
        name='lbr_indipendent_joint_controller',
        output='screen',
        #parameters=[{'wheel_mode': LaunchConfiguration('wheel_mode')}]
    )

    lbr_indipendent_joint_controller_node_RH = Node(
        package='lbr_indipendent_joint_controller',
        executable='lbr_indipendent_joint_controller',
        namespace='RH',
        name='lbr_indipendent_joint_controller',
        output='screen',
        #parameters=[{'wheel_mode': LaunchConfiguration('wheel_mode')}]
    )

    lbr_indipendent_joint_controller_node_RF = Node(
        package='lbr_indipendent_joint_controller',
        executable='lbr_indipendent_joint_controller',
        namespace='RF',
        name='lbr_indipendent_joint_controller',
        output='screen',
        #parameters=[{'wheel_mode': LaunchConfiguration('wheel_mode')}]
    )

    return LaunchDescription([
        lbr_indipendent_joint_controller_node_LF,
        RegisterEventHandler(
            OnProcessStart(
                target_action=lbr_indipendent_joint_controller_node_LF,
                on_start=[
                    LogInfo(msg='IJC for LF started, start IJC for LH limb.'),
                    lbr_indipendent_joint_controller_node_LH
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=lbr_indipendent_joint_controller_node_LH,
                on_start=[
                    LogInfo(msg='IJC for LH started, start IJC for RH limb.'),
                    lbr_indipendent_joint_controller_node_RH
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=lbr_indipendent_joint_controller_node_RH,
                on_start=[
                    LogInfo(msg='IJC for RH started, start IJC for RF limb.'),
                    lbr_indipendent_joint_controller_node_RF
                ]
            )
        )
    ])
