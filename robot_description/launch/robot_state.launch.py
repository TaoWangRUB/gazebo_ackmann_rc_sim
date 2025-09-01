import os, xacro
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart

ARGUMENTS = [
    DeclareLaunchArgument('robot_name', default_value='',
                          description='Robot name'),
    DeclareLaunchArgument('namespace', default_value=LaunchConfiguration('robot_name'),
                          description='Robot namespace'),
    DeclareLaunchArgument('rviz', default_value='true',
                          choices=['true', 'false'], description='Start rviz.'),
    DeclareLaunchArgument('world', default_value='warehouse',
                          description='Ignition World'),
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument(
        'unite_imu_method', default_value='1',
        description='0-None, 1-copy, 2-linear_interpolation. Use unite_imu_method:="1" if imu topics stop being published.')
]

def generate_launch_description():
    
    # Directories
    pkg_ros_realsense = get_package_share_directory(
        'realsense2_camera')
    pkg_turtlebot4_viz = get_package_share_directory(
        'turtlebot4_viz')
    
    realsense_launch = PathJoinSubstitution(
        [pkg_ros_realsense, 'launch', 'rs_launch.py'])
    rviz_launch = PathJoinSubstitution(
        [pkg_turtlebot4_viz, 'launch', 'view_robot.launch.py'])
    
    # Robot description
    pkg_robot_description = get_package_share_directory('robot_description')
    xacro_file = PathJoinSubstitution([pkg_robot_description,
                                       'urdf',
                                       'donkey_sensors.urdf'
                                       ])
    namespace = LaunchConfiguration('namespace')
    
    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_description': Command([
                'xacro', ' ', xacro_file, ' ',
                'gazebo:=ignition', ' ',
                'namespace:=', namespace
                ])},
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                    ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    # Open RViz
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rviz_launch]),
        launch_arguments=[
            ('namespace', ''),
        ],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(rviz)
    return ld