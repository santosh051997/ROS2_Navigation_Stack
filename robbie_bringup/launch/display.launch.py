
from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_path

def generate_launch_description():
     
    rviz_config = os.path.join(get_package_share_path('path_planning'), 'rviz', 'astar.rviz')


    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{'yaml_filename': '/home/santosh/ros2mapping_ws/src/astar/maps/local_roadmap.yaml'}],  # Update with your map file path
            
        ),
        
        Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config]
    )
        # Add other nodes for navigation (e.g., planner, controller, etc.) if needed
    ])

