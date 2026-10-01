import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
 
def generate_launch_description():

    pkg_maze = get_package_share_directory('turtlebot_maze')
    pkg_logic = get_package_share_directory('turtlebot_logic')
 
    # 1. Avvio Gazebo 
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_maze, 'launch', 'maze_sim.launch.py'))
    )
 
    # 2. Avvio SLAM e Nav2 (ritardo di 5 secondi per far caricare Gazebo)
    start_nav = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_logic, 'launch', 'navigation.launch.py'))
            )
        ]
    )
 
    # 3. Avvio Explore Lite (ritardo di 12 secondi per far inizializzare lo SLAM)
    start_explore = TimerAction(
        period=12.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_logic, 'launch', 'explore.launch.py'))
            )
        ]
    )

    # 4. Avvio data_logger per registrare i dati
    start_logger = Node(
        package='turtlebot_logic',
        executable='data_logger',
        name='data_logger_node',
        output='screen',
    )
 
    return LaunchDescription([
        start_gazebo,
        start_nav,
        start_explore,
        start_logger,
    ])
 