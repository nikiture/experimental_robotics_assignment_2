"""
Launch slam_toolbox nodes with custom configs
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
    slam_config = os.path.join(test_robot_description_share, 'config/slam_config.yaml')
    
    slam_launcher = IncludeLaunchDescription (
        PythonLaunchDescriptionSource (
            #os.path.join(slam_launch, 'online_sync_launch.py')
            PathJoinSubstitution([
                FindPackageShare ('slam_toolbox'),
                'launch', 
                'online_sync_launch.py'
            ])
        ),
        launch_arguments = {
            'params_file': slam_config,
            'use_sim_time': 'True',
            'autostart': 'True',
        }.items()
    )
    
    return LaunchDescription ([
        slam_launcher,
    ])
