
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time',
                                      default_value='false')
    

    lbr_data_csv_LF = Node(
        package='lbr_data_csv',
        executable='lbr_data_csv',
        namespace='LF',
        name='lbr_data_csv',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    lbr_data_csv_LH = Node(
        package='lbr_data_csv',
        executable='lbr_data_csv',
        namespace='LH',
        name='lbr_data_csv',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    lbr_data_csv_RH = Node(
        package='lbr_data_csv',
        executable='lbr_data_csv',
        namespace='RH',
        name='lbr_data_csv',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    lbr_data_csv_RF = Node(
        package='lbr_data_csv',
        executable='lbr_data_csv',
        namespace='RF',
        name='lbr_data_csv',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

 

    return LaunchDescription([ 
        use_sim_time_arg,
        lbr_data_csv_LF,
        lbr_data_csv_LH,
        lbr_data_csv_RH,
        lbr_data_csv_RF,
    ])
