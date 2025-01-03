"""
Launch plansys2 nodes with custom configs
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    test_robot_description_share = FindPackageShare(package='robot_exploration').find('robot_exploration')
    
    """ planning_files_path = os.path.join(test_robot_description_share, 'planning_files')
    planning_domain_file = os.path.join(planning_files_path, 'maze.pddl')
    planning_problem_file = os.path.join(planning_files_path, 'find_lowest_marker.pddl') """
    
    
    planning_domain_file = os.path.join (test_robot_description_share, 'planning_files/maze.pddl')
    planning_problem_file_dir = os.path.join (test_robot_description_share, 'planning_files/find_lowest_marker.pddl')
    #print ("generated problem file directory:")
    #print (planning_problem_file)
    param_file_dir = os.path.join(test_robot_description_share, 'config/plansys_params.yaml')
    plansys_dir = FindPackageShare(package = 'plansys2_bringup').find('plansys2_bringup')
    plan_problem_file = open(planning_problem_file_dir, 'r')
    plan_problem_str = plan_problem_file.read()
    
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
            #'problem_file': planning_problem_file,
            'params_file': param_file_dir,
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
    
    problem_file_adder = ExecuteProcess(
        cmd=[[
            #FindExecutable(name='ros2'),
            'ros2',
            " service call ",
            "/problem_expert/add_problem ",
            "plansys2_msgs/srv/AddProblem ",
            '"{problem: ', plan_problem_str, '}"',
        ]],
        shell=True
    )
    
    problem_file_loader = Node (
        package = 'problem_load',
        executable = 'problem_loader',
        arguments = [planning_problem_file_dir],
    )
    
    return LaunchDescription ([
        plansys2_launcher,
        #problem_file_adder,
        #problem_file_loader,
        move_action_node,
        search_marker_server,
        search_marker_action,
    ])