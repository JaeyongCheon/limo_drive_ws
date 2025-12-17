import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    remo_navigation_dir = get_package_share_directory('remo_navigation')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',  # LIMO Pro는 실제 로봇이므로 false
        description='Use simulation time')
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2')
    
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(remo_navigation_dir, 'rviz', 'autonomy_view.rviz'),
        description='Full path to RViz2 config file')
    
    # Static TF: base_link -> laser_link (LIMO Pro LiDAR 위치)
    laser_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_tf_publisher',
        arguments=[
            '--x', '0.10',      # 전방 10cm
            '--y', '0.0',
            '--z', '0.10',      # 높이 10cm
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser_link'
        ]
    )
    
    # Static TF: base_link -> camera_link (LIMO Pro 카메라 위치)
    camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_publisher',
        arguments=[
            '--x', '0.12',      # 전방 12cm
            '--y', '0.0',
            '--z', '0.15',      # 높이 15cm
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link'
        ]
    )
    
    # Terrain analysis node (LIMO Pro version with depth camera)
    terrain_analysis_node = Node(
        package='terrain_analysis',
        executable='terrain_analysis_node_limo',
        name='terrain_analysis_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_length_x': 4.0,           # LIMO Pro: 4m 범위 (실내 최적화)
            'map_length_y': 4.0,
            'map_resolution': 0.05,        # 5cm 해상도 (6,400 셀)
            'map_frame': 'base_link',      # 로봇 중심으로 따라감
            'base_frame': 'base_link',
            'scan_topic': '/scan',
            'imu_topic': '/imu',
            'odom_topic': '/odom',
            'depth_topic': '/camera/depth/points',  # Orbbec Dabai
            'publish_rate': 10.0,
            'depth_processing_rate': 5.0,
            'max_slope': 0.5,
            'max_roughness': 0.15,
            'use_depth_camera': True,
            'depth_max_range': 3.0,        # 3m로 제한 (실내 최적화)
            'scan_max_range': 3.0,         # LiDAR 범위 제한
            'depth_min_range': 0.3
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
            'inflation_radius': 0.3         # LIMO Pro 크기에 맞게 조정
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
    
    # TF publishers (먼저 실행되어야 함)
    ld.add_action(laser_tf_node)
    ld.add_action(camera_tf_node)
    
    ld.add_action(terrain_analysis_node)
    ld.add_action(grid_map_costmap_converter_node)
    ld.add_action(slam_launch)
    ld.add_action(navigation_launch)
    ld.add_action(rviz2_node)
    
    return ld

