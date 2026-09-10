import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    pkg_turtlebot_logic = get_package_share_directory('turtlebot_logic')

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    nav2_params = os.path.join(pkg_turtlebot_logic, 'config', 'nav2_params.yaml')

    # NOVITA': Usiamo i parametri di default di slam_toolbox per assicurarci che parta!

    slam_params = os.path.join(slam_toolbox_dir, 'config', 'mapper_params_online_async.yaml')

    start_slam = IncludeLaunchDescription(

        PythonLaunchDescriptionSource(

            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')

        ),

        launch_arguments={

            'use_sim_time': 'true',

            'slam_params_file': slam_params # Modificato qui

        }.items()

    )

    start_nav2 = IncludeLaunchDescription(

        PythonLaunchDescriptionSource(

            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')

        ),

        launch_arguments={

            'use_sim_time': 'true',

            'params_file': nav2_params

        }.items()

    )

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
 