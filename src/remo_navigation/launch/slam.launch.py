import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    remo_navigation_dir = get_package_share_directory('remo_navigation')
    
    # Launch arguments
    slam_config_file = LaunchConfiguration('slam_config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_slam_config_file_cmd = DeclareLaunchArgument(
        'slam_config_file',
        default_value=os.path.join(remo_navigation_dir, 'config', 'slam_toolbox.yaml'),
        description='Full path to the SLAM Toolbox config file')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time')
    
    # SLAM Toolbox node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/scan', '/scan'),
            ('/map', '/map'),
            ('/map_metadata', '/map_metadata')
        ]
    )
    
    ld = LaunchDescription()
    ld.add_action(declare_slam_config_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(slam_toolbox_node)
    
    return ld

