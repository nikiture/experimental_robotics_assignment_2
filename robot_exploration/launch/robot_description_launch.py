"""
Spawn Robot Description
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

""" marker_printer,
        robot_controller,
        marker_circler, """
        

""" marker_printer = Node (
    package = 'marker_localizer',
    executable = 'aruco_test',
    parameters = [
        #{"starting_rotation_speed": "1.0"}
        {"starting_rotation_speed": 1.0}
    ]
)
robot_controller = Node (
    package = "robot_exploration",
    executable = 'Robot_controller',
    parameters = [
        #{"starting_rotation_speed": "1.0"}
        {"starting_rotation_speed": 1.0}
    ]
)


marker_circler = Node (
    package = 'py_cv_marker',
    executable = 'marker_circler',
) """

def generate_launch_description():
    
    test_robot_description_share = FindPackageShare(package='robot_exploration').find('robot_exploration')
    default_model_path = os.path.join(test_robot_description_share, 'urdf/robot5.xacro')
    default_world_path = os.path.join(test_robot_description_share, 'worlds/assignment_2.world')
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(name='model', default_value=default_model_path,
                                    description='Absolute path to robot urdf file'),
        #nav_launcher,
        #slam_launcher,
        robot_state_publisher_node,
    ])