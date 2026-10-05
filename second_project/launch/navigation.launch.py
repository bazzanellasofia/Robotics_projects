import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('second_project')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    return LaunchDescription([
        
        Node(
            package='stage_ros2_stageros', 
            executable='stageros',
            arguments=[os.path.join(pkg_dir, 'world', 'stage_world.world')],
            parameters=[{'use_sim_time': True}],
            remappings=[('/base_scan', '/scan')]
        ),


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': os.path.join(pkg_dir, 'map', 'my_final_map.yaml'),
                'use_sim_time': 'True',
                'params_file': os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
            }.items()
        ),


        Node(
            package='second_project', 
            executable='csv_controller',
            output='screen', 
            parameters=[{'use_sim_time': True}]
        ),


        Node(
            package='tf2_ros', 
            executable='static_transform_publisher',
            name='link_base_to_scan',
            arguments=['0.0', '0.0', '0.2', '0.0', '0.0', '0.0', 'base_link', 'base_laser_link'],
            parameters=[{'use_sim_time': True}] 
        ),



        Node(
            package='rviz2', 
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_dir, 'rviz', 'navigation.rviz')],
            parameters=[{'use_sim_time': True}] 
        )
    ])
