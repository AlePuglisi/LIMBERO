from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    leptrino_node_LF = Node(
        package="leptrino_force_torque",
        executable="leptrino_force_torque",
        namespace='LF',
        name='leptrino_force_torque',
        output="screen"
    )

    leptrino_node_LH = Node(
        package="leptrino_force_torque",
        executable="leptrino_force_torque",
        namespace='LH',
        name='leptrino_force_torque',
        output="screen"
    )

    leptrino_node_RH = Node(
        package="leptrino_force_torque",
        executable="leptrino_force_torque",
        namespace='RH',
        name='leptrino_force_torque',
        output="screen"
    )

    leptrino_node_RF = Node(
        package="leptrino_force_torque",
        executable="leptrino_force_torque",
        namespace='RF',
        name='leptrino_force_torque',
        output="screen"
    )

    return LaunchDescription([
        leptrino_node_LF,
        leptrino_node_LH,
        leptrino_node_RH,
        leptrino_node_RF
    ])
