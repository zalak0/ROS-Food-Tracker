import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    
    config_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'config')
    params_file = os.path.join(config_dir, 'nav2_params.yaml')
    rviz_config = os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'rviz', 'tb3_navigation2.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='False')  # Set to False for real robot use

    # Launch SLAM toolbox
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # Launch Nav2
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': use_sim_time
        }.items(),
    )

    # Launch RViz
    rviz = Node(
        package='rviz2',
        output='screen',
        executable='rviz2',
        name='rviz2_node',
        arguments=['-d', rviz_config]
    )

    # Launch Stock System
    stock_system = Node(
        package='turtlebot3_gazebo',
        output='screen',
        executable='CStockList',
        name='CStockList_node'
    )

    # Launch Tag Detector
    tag_detector = Node(
        package='turtlebot3_gazebo',
        output='screen',
        executable='CArUcoDetector',
        name='CArUcoDetector_node'
    )

    # Launch Custom Drive Logic (CDriveLogic)
    drive_logic = Node(
        package='turtlebot3_gazebo',
        output='screen',
        executable='CDriveLogic',
        name='turtlebot3_drive_node'
    )

    ld = LaunchDescription()

    # Launch Nav2 immediately
    ld.add_action(
        TimerAction(
            period=0.0,
            actions=[nav_launch]
        )
    )

    # Launch SLAM after 5 seconds
    ld.add_action(
        TimerAction(
            period=5.0,
            actions=[slam_launch]
        )
    )

    # Launch Custom Drive Logic after SLAM (10 seconds)
    ld.add_action(
        TimerAction(
            period=10.0,
            actions=[drive_logic]
        )
    )

    # Launch Stock System after 12 seconds
    ld.add_action(
        TimerAction(
            period=12.0,
            actions=[stock_system]
        )
    )

    # Launch Tag Detector after 14 seconds
    ld.add_action(
        TimerAction(
            period=14.0,
            actions=[tag_detector]
        )
    )

    # Launch RViz after 15 seconds (gives enough time for Nav2 and SLAM to initialize)
    ld.add_action(
        TimerAction(
            period=15.0,
            actions=[rviz]
        )
    )

    return ld

