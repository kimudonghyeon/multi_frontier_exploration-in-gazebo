# Authors: Abdulkadir Ture
# Github : abdulkadrtr

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    rviz_file = os.path.join(get_package_share_directory('merge_map'), 'config', 'merge_map.rviz')
    
    static_tf_tb3_0_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_tb3_0_map',
        arguments=['0', '0', '0.01', '0', '0', '0', 'merge_map', 'tb3_0/map']
    )
    static_tf_tb3_1_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_tb3_1_map',
        arguments=['0', '0', '0.01', '0', '0', '0', 'merge_map', 'tb3_1/map']
    )

    return LaunchDescription([
        static_tf_tb3_0_map,
        static_tf_tb3_1_map,
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_file],
            parameters=[{'use_sim_time': True}],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        ),
        Node(
            package='merge_map',
            executable='merge_map',
            output='screen',
            parameters=[{'use_sim_time': True}],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        ),
    ])
