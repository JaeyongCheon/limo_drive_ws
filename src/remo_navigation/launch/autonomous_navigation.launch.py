import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    remo_navigation_dir = get_package_share_directory('remo_navigation')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',  # Webots 시뮬레이션 환경 기본값
        description='Use simulation time')
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2')
    
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(remo_navigation_dir, 'rviz', 'autonomy_view.rviz'),
        description='Full path to RViz2 config file')
    
    # Terrain analysis node
    terrain_analysis_node = Node(
        package='terrain_analysis',
        executable='terrain_analysis_node',
        name='terrain_analysis_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_length_x': 8.0,
            'map_length_y': 8.0,
            'map_resolution': 0.5,
            'map_frame': 'odom',
            'base_frame': 'base_link',
            'scan_topic': '/scan',
            'imu_topic': '/imu/data',
            'odom_topic': '/odom',
            'publish_rate': 10.0,
            'max_slope': 0.5,
            'max_roughness': 0.2
        }]
    )
    
    # Grid Map to Costmap converter
    grid_map_costmap_converter_node = Node(
        package='remo_navigation',
        executable='grid_map_costmap_converter.py',
        name='grid_map_costmap_converter',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'grid_map_topic': '/grid_map',
            'costmap_topic': '/grid_map_costmap',
            'traversability_layer': 'traversability',
            'inflation_radius': 0.5
        }]
    )
    
    # SLAM launch
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(remo_navigation_dir, 'launch', 'slam.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # Navigation launch
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(remo_navigation_dir, 'launch', 'navigation.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # RViz2
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(use_rviz)
    )
    
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(terrain_analysis_node)
    ld.add_action(grid_map_costmap_converter_node)
    ld.add_action(slam_launch)
    ld.add_action(navigation_launch)
    ld.add_action(rviz2_node)
    
    return ld

