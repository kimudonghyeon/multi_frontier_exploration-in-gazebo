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
    model_folder = 'turtlebot3_burger'
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    robot_desc_path = os.path.join(get_package_share_directory("turtlebot3_multi"), "urdf", "turtlebot3_burger.urdf")
    world = os.path.join(get_package_share_directory('turtlebot3_multi'),'worlds','turtlebot3_house.world')
    urdf_path1 = os.path.join(get_package_share_directory('turtlebot3_multi'),'models',model_folder+'_0','model.sdf')
    urdf_path2 = os.path.join(get_package_share_directory('turtlebot3_multi'),'models',model_folder+'_1','model.sdf')
    with open(robot_desc_path, 'r') as infp:
        robot_desc = infp.read()
    name1 = "tb3_0"
    name2 = "tb3_1"
    # 1. ROBOT ICIN KODLAR
    spawn_robot1 = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py', 
        arguments=[
            '-entity', name1, 
            '-file', urdf_path1, 
            '-x', '5.0', 
            '-y', '2.5', 
            '-z', '0.01',
            '-robot_namespace', name1,
        ],
        output='screen'
    )
    robot_state_publisher1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=name1,
        output='screen',
        parameters=[{'frame_prefix': name1 + '/',
                    'use_sim_time': True,
                    'robot_description': robot_desc}]
    )
    cartographer1 = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        namespace=name1,
        parameters=[{
            'use_sim_time': True,
#            'map_frame': name1 + '/map',
#            'tracking_frame': name1 + '/base_link',
#            'published_frame': name1 + '/odom',
#            'odom_frame': name1 + '/odom',
#            'provide_odom_frame': False,
#            'use_odometry': True,
#            'use_nav_sat': False,
#            'use_landmarks': False,
        }],
        arguments=[
            '-configuration_directory', os.path.join(get_package_share_directory('turtlebot3_multi'), 'config'),
            '-configuration_basename', 'tb3_0.lua'
        ],
        remappings=[
            ("scan", "scan")#,
            #("imu", "imu"),
        ],
        output='screen',
    )  
    map_occupancy_grid_node1 = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        namespace=name1,
        parameters=[{
            'use_sim_time': True,
#            'map_frame': 'tb3_0/map',
#            'tracking_frame': 'tb3_0/base_link',
#            'published_frame': 'tb3_0/odom',
#            'odom_frame': 'tb3_0/odom',
#            'provide_odom_frame': True,
#            'use_odometry': True,
#            'use_nav_sat': False,
#            'use_landmarks': False,
#            '-resolution',        '0.05',
#            '-publish_period_sec','1.0'
        }],
        remappings=[
            ('map', '/tb3_0/map'),
            ('map_metadata', '/tb3_0/map_metadata'),
        ],
        output='screen'
    )
    
    rviz1 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d', os.path.join(get_package_share_directory('turtlebot3_multi'), 'config',name1+'.rviz')]#,
            #'--no-gui']
    )
    # 2. ROBOT ICIN KODLAR

    spawn_robot2 = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py', 
        arguments=[
            '-entity', name2, 
            '-file', urdf_path2, 
            '-x', '3.0', 
            '-y', '2.5', 
            '-z', '0.01',
            '-robot_namespace', name2,
        ],
        output='screen'
    )
    robot_state_publisher2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=name2,
        output='screen',
        parameters=[{'frame_prefix': name2 + '/',
                    'use_sim_time': True,
                    'robot_description': robot_desc}]
    )
    cartographer2 = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        namespace=name2,
        parameters=[{
            'use_sim_time': True,
#            'map_frame': 'tb3_1/map',
#            'tracking_frame': 'tb3_1/base_link',
#            'published_frame': 'tb3_1/odom',
#            'odom_frame': 'tb3_1/odom',
#            'provide_odom_frame': False,
#            'use_odometry': True,
#            'use_nav_sat': False,
#            'use_landmarks': False,
        }],
        arguments=[
            '-configuration_directory', os.path.join(get_package_share_directory('turtlebot3_multi'), 'config'),
            '-configuration_basename', 'tb3_1.lua'
        ],
        remappings=[
            ("scan", "scan")#,
            #("imu", "imu"),
        ],
        output='screen',
    )
      
    map_occupancy_grid_node2 = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        namespace=name2,
        parameters=[{
            'use_sim_time': True,
#            'map_frame': 'tb3_1/map',
#            'tracking_frame': 'tb3_1/base_link',
#            'published_frame': 'tb3_1/odom',
#            'odom_frame': 'tb3_1/odom',
#            'provide_odom_frame': True,
#            'use_odometry': True,
#            'use_nav_sat': False,
#            'use_landmarks': False,
#            '-resolution',        '0.05',
#            '-publish_period_sec','1.0'
        }],
        remappings=[
            ('map', '/tb3_1/map'),
            ('map_metadata', '/tb3_1/map_metadata'),
        ],
        output='screen'
    )
    
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d', os.path.join(get_package_share_directory('turtlebot3_multi'), 'config', name2+'.rviz')]#,
            #'--no-gui']
    )
    
    # GAZEBO ICIN KODLAR
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world,'verbose':"true",'extra_gazebo_args': 'verbose'}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        launch_arguments={'verbose':"true"}.items()
    )    
    pathFollow1 = Node(
        package='path_follow',
        executable='path_follow',
        name='pathFollow',
        output='screen',
        namespace=name1,
        parameters=[{
            'use_sim_time': True,
        }],
        remappings=[
            ("/path", "/"+name1+"/path"),
            ("/cmd_vel", "/"+name1+"/cmd_vel"),
            ("/odom", "/"+name1+"/odom"),
            ("/visual_path", "/"+name2+"/visual_path"),
        ]
    )
    pathFollow2 = Node(
        package='path_follow',
        executable='path_follow',
        name='pathFollow',
        output='screen',
        namespace=name2,
        parameters=[{
            'use_sim_time': True,
        }],
        remappings=[
            ("/path", "/"+name2+"/path"),
            ("/cmd_vel", "/"+name2+"/cmd_vel"),
            ("/odom", "/"+name2+"/odom"),
            ("/visual_path", "/"+name1+"/visual_path"),
        ]
    )

    

    ld = LaunchDescription()
    
    ld.add_action(spawn_robot1)
    ld.add_action(robot_state_publisher1)
    ld.add_action(cartographer1)
    ld.add_action(map_occupancy_grid_node1)
    ld.add_action(rviz1)
    
    ld.add_action(spawn_robot2)
    ld.add_action(robot_state_publisher2)
    ld.add_action(cartographer2)
    ld.add_action(map_occupancy_grid_node2)
    ld.add_action(rviz2)
    
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(pathFollow1)
    ld.add_action(pathFollow2)
    return ld
