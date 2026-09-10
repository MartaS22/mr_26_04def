import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='explore_lite',
            executable='explore',
            name='explore_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_base_frame': 'base_link',
                'costmap_topic': '/global_costmap/costmap',
                'costmap_updates_topic': '/global_costmap/costmap_updates',
                'visualize': True,
                'planner_frequency': 0.1,
                'progress_timeout': 30.0,
                'potential_scale': 3.0,
                'orientation_scale': 0.0,
                'gain_scale': 1.0,
                'transform_tolerance': 0.3,
                'min_frontier_size': 0.5,
            }]

        )

    ])
 