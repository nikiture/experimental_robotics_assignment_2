"""
launch navigation2 nodes with custom configs
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    test_robot_description_share = FindPackageShare(package='robot_exploration').find('robot_exploration')
    nav_config = os.path.join(test_robot_description_share, 'config/nav2_params.yaml')
    
    nav_launcher = IncludeLaunchDescription (
        PythonLaunchDescriptionSource (
            #os.path.join(nav_launch, 'navigation_launch.py')
            PathJoinSubstitution([
                FindPackageShare ('nav2_bringup'),
                'launch', 
                'navigation_launch.py'
            ])
        ),
        launch_arguments = {
            'use_sim_time': 'True', 
            'autostart': 'True', 
            'params_file': nav_config,
        }.items()
    )
    
    return LaunchDescription ([
        nav_launcher,
    ])