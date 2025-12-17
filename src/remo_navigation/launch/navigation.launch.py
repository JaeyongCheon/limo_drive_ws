import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    remo_navigation_dir = get_package_share_directory('remo_navigation')
    
    # Launch arguments
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_nav2_params_file_cmd = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(remo_navigation_dir, 'config', 'nav2_params_slam.yaml'),
        description='Full path to the Nav2 parameters file (optimized for real-time SLAM)')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time')
    
    nav2_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'namespace': '',
            'use_sim_time': use_sim_time,
            'autostart': 'True',
            'params_file': nav2_params_file,
            'use_composition': 'False',  # 개별 노드로 실행
            'use_respawn': 'True',        # 노드 재시작 활성화
            'log_level': 'info'
        }.items()
    )
    
    ld = LaunchDescription()
    ld.add_action(declare_nav2_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(nav2_navigation_launch)
    
    return ld

