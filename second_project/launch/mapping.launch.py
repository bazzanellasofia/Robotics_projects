import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_dir = get_package_share_directory('second_project')
    rviz_config_dir = os.path.join(package_dir, 'rviz', 'mapping.rviz')
    

    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/ugv/rslidar_points'),
            ('scan', '/scan')
        ],
        parameters=[{
            'use_sim_time': True,
            'target_frame': 'rslidar',
            'transform_tolerance': 0.1,
            'queue_size': 20
        }]
    )

    
    
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_lidar',
        arguments=['0.0', '0.0', '0.2', '0.0', '0.0', '0.0', 'UGV_base_link', 'rslidar']
    )


    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[{
            'use_sim_time': True,
            'odom_frame': 'UGV_odom',
            'map_frame': 'map',
            'base_frame': 'UGV_base_link',
            'scan_topic': '/scan',
	    'transform_timeout': 0.5,
	    'map_update_interval': 1.0,
	    'resolution': 0.05,
        }],
	output='screen'
    )


    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        pointcloud_to_laserscan_node,
        static_tf_node,
        slam_node,
        rviz_node
    ])
