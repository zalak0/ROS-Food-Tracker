import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    
    params_file = os.path.join( 'config', 'nav2_params.yaml' )
    rviz_config= os.path.join( 'rviz', 'tb3_nav.rviz' )

    # launch_file_dir = os.path.join(get_package_share_directory('world_sim'), 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time', default='False')  # Changed to false for real robot

    # Launch slam_toolbox
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time  # Added use_sim_time for real robot
        }.items(),
    )

    # Launch nav2
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': use_sim_time  # Added use_sim_time for real robot
        }.items(),
    )

    # Launch rviz
    rviz = Node(
        package='rviz2',
        output='screen',
        executable='rviz2',
        name='rviz2_node',
        arguments=['-d', rviz_config]
    )

    ld = LaunchDescription()

    ld.add_action(
        TimerAction(
            period=0.0,
            actions=[nav_launch]
        )
    )

    ld.add_action(
        TimerAction(
            period=5.0,
            actions=[slam_launch]
        )
    )

    ld.add_action(
        TimerAction(
            period=10.0,
            actions=[rviz]
        )
    )

    return ld
