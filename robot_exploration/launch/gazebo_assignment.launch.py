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
    rviz_config_path = os.path.join(test_robot_description_share, 'config/rviz.rviz')
    planning_files_path = os.path.join(test_robot_description_share, 'planning_files')
    planning_domain_file = os.path.join(planning_files_path, 'maze.pddl')
    planning_problem_file = os.path.join(planning_files_path, 'find_lowest_marker.pddl')
    param_file_dir = os.path.join(test_robot_description_share, 'config/plansys_params.yaml')
    plansys_dir = FindPackageShare(package = 'plansys2_bringup').find('plansys2_bringup')
    plansys_launch_dir = os.path.join (plansys_dir, 'launch')
    """ slam_launch = os.path.join (
        FindPackageShare (package = 'slam_toolbox').find('slam_toolbox'),
        'launch'
    ) """
    """ nav_launch = os.path.join (
        FindPackageShare(package = 'nav2_bringup').find('nav2_bringup'),
        'launch'
    ) """
    slam_config = os.path.join(test_robot_description_share, 'config', 'slam_config.yaml')
    nav_config = os.path.join(test_robot_description_share, 'config', 'nav2_params.yaml')
    #aruco_detector = FindPackageShare (package = 'ros2_aruco').find ('ros2_aruco')
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )
    
    
    """ plansys = Node (
       package = 'plansys2_bringup',
       #executable = 'plan2sys_bringup_launch_distributed.py',
       executable = 'plansys2_bringup_launch_distributed.py',
       arguments = ['model_file:=', planning_domain_file],
    ) """
    
    plansys2_launcher = IncludeLaunchDescription (
        PythonLaunchDescriptionSource (
            #os.path.join(plansys_launch_dir, 'plansys2_bringup_launch_distributed.py')
            PathJoinSubstitution ([
                FindPackageShare ('plansys2_bringup').find('plansys2_bringup'),
                'launch',
                'plansys2_bringup_launch_distributed.py'
            ])
        ),
        launch_arguments={
            'model_file': planning_domain_file,
            'problem_file': planning_problem_file,
            'params_file': param_file_dir,
        }.items()
    )

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
            #'use_lifecycle_manager': 'False',
        }.items()
    )
    
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
    
    move_action_node = Node (
        package = 'navigation_actions',
        executable = 'move_action',
    )
    
    search_marker_server = Node(
        package = 'search_marker_server',
        executable = 'search_marker_server'
    )
    
    search_marker_action = Node (
        package = 'navigation_actions',
        executable = 'search_action'
    )
    reach_min_action = Node (
        package = 'navigation_actions',
        executable = 'reach_min_action'
    )
    
    # GAZEBO_MODEL_PATH has to be correctly set for Gazebo to be able to find the model
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-entity', 'my_test_robot', '-topic', '/robot_description', '-y', '1.0'],
                        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument(name='model', default_value=default_model_path,
                                    description='Absolute path to robot urdf file'),
        #nav_launcher,
        #slam_launcher,
        robot_state_publisher_node,
        joint_state_publisher_node,
        spawn_entity,
        #plansys2_launcher,
        
        #camera_controller,
        ExecuteProcess(
            cmd=['gazebo', '--verbose', default_world_path, '-s', 'libgazebo_ros_factory.so'],
            output='screen'),
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config_path],
            output='screen'),
        
        nav_launcher,
        slam_launcher,
        #plansys2_launcher,
        #move_action_node,
        #search_marker_action,
        #search_marker_server,
        #reach_min_action,
        
        
        #aruco_detector,
        
        
    ])

