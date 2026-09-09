import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
 
def generate_launch_description():
    # Percorsi dei pacchetti ufficiali di Nav2 e SLAM
    pkg_turtlebot_logic = get_package_share_directory('turtlebot_logic')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
 
    # File di configurazione di default per il TurtleBot (inclusi in Nav2)
    nav2_params = os.path.join(pkg_turtlebot_logic, 'config', 'nav2_params.yaml')
 
    # 1. Nodo SLAM Toolbox (Per creare la mappa in tempo reale usando il laser)
    start_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params
        }.items()
    )
 
    # 2. Nodo Navigation2 (Il cervello che calcola i percorsi e schiva gli ostacoli)
    start_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params
        }.items()
    )
 
    # 3. RViz2 (L'interfaccia grafica per vedere cosa "pensa" il robot)
    # RViz è fondamentale per farti vedere la mappa che si crea e i marker che il robot vede
    start_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
 
    return LaunchDescription([
        start_slam,
        start_nav2,
        start_rviz
    ])