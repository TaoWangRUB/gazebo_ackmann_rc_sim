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
robot_base_color = '0.0 0.0 1.0 0.95' #Ign and Rviz color of the robot's main body (rgba)

ARGUMENTS = [
    DeclareLaunchArgument('robot_name', default_value='ackermann',
                          description='Robot name'),
    DeclareLaunchArgument('namespace', default_value=LaunchConfiguration('robot_name'),
                          description='Robot namespace'),
    DeclareLaunchArgument('rviz', default_value='true',
                          choices=['true', 'false'], description='Start rviz.'),
    DeclareLaunchArgument('world', default_value='warehouse',
                          description='Ignition World'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('localization', default_value='false',
                          choices=['true', 'false'], description='Start rtabmap in localization mode (a map should have been already created).'),
    DeclareLaunchArgument('nav2', default_value='true',
                          choices=['true', 'false'], description='Start nav2.'),
    DeclareLaunchArgument(
        'unite_imu_method', default_value='1',
        description='0-None, 1-copy, 2-linear_interpolation. Use unite_imu_method:="1" if imu topics stop being published.')
]

for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(
        DeclareLaunchArgument(pose_element, default_value='0.0',
                              description=f'{pose_element} component of the robot pose.')
    )

def generate_launch_description():
    
    # Directories
    pkg_robot_ignition_bringup = get_package_share_directory(
        'robot_description')
    pkg_ros_realsense = get_package_share_directory(
        'realsense2_camera')
    pkg_turtlebot4_navigation = get_package_share_directory(
        'turtlebot4_navigation')
    pkg_turtlebot4_viz = get_package_share_directory(
        'turtlebot4_viz')
    pkg_controller = get_package_share_directory(
        'robot_description')
    
    realsense_launch = PathJoinSubstitution(
        [pkg_ros_realsense, 'launch', 'rs_launch.py'])
    rviz_launch = PathJoinSubstitution(
        [pkg_turtlebot4_viz, 'launch', 'view_robot.launch.py'])
    localization_launch = PathJoinSubstitution(
        [pkg_turtlebot4_navigation, 'launch', 'localization.launch.py'])
    slam_launch = PathJoinSubstitution(
        [pkg_turtlebot4_navigation, 'launch', 'slam.launch.py'])
    nav2_launch = PathJoinSubstitution(
        [pkg_robot_ignition_bringup, 'launch', 'nav2_bringup.launch.py'])
    
    # Robot description
    pkg_robot_description = get_package_share_directory('robot_description')
    xacro_file = PathJoinSubstitution([pkg_robot_description,
                                       'urdf',
                                       'donkey_sensors.urdf'
                                       ])
    namespace = '' #LaunchConfiguration('namespace')
    
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

    # Camera launch
    realsense_d435i = IncludeLaunchDescription(realsense_launch,
        launch_arguments=[
            ('camera_namespace', ''),
            ('camera_name', 'd435i'),
            ('publish_tf', 'false'),
            ('enable_accel', 'true'),
            ('enable_gyro', 'true'),
            ('enable_rgbd', 'true'),
            ('unite_imu_method', LaunchConfiguration('unite_imu_method')),
            ('enable_sync', 'true'),

            ('enable_color', 'true'),
            ('enable_depth', 'true'),
            ('align_depth.enable', 'true'),
            ('enable_rgbd', 'false'),
            ('rgb_camera.color_profile', '640x480x15'),                              
            ('depth_module.depth_profile', '640x480x15'),
            ('enable_infra1', 'false'),
            ('enable_infra2', 'false'),
            ('pointcloud.enable', 'true'),
        ],
        #condition=IfCondition(LaunchConfiguration('use_sim_time'))
    )

    # Scan clipper node
    scan_clipper_node = Node(
        package='robot_description',
        executable='scan_clipper.py',
        name='scan_clipper',
        output='screen',
        parameters=[{
            #'max_range': LaunchConfiguration('max_range'),
            #'min_range': LaunchConfiguration('min_range'),
            'input_topic': "/rplidar/scan",
            'output_topic': "/scan",
        }],
    )
    
    # IMU covariance injector node
    # This node injects fixed covariance values into the IMU messages
    # It is used to ensure that the IMU messages have non-zero covariance values
    # This is important for some algorithms that require covariance information
    # such as sensor fusion algorithms
    imu_covariance_injector = Node(
        package='robot_description',
        executable='imu_covariance_injector.py',
        name='imu_covariance_injector',
        output='screen',
        parameters=[{
            'input_topic': '/l515/imu/raw',
            'output_topic': '/l515/imu/data',
        }],
    )      

    ros2_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_controller, 'launch', 'robot_control.launch.py')),
        launch_arguments=[
            ('namespace', namespace),
        ]
    ) 
    ros2_controller_callback = RegisterEventHandler(
        #event_handler=OnProcessExit(
        event_handler=OnProcessStart(
            target_action=robot_state_publisher,
            #on_exit=[ros2_controller],
            on_start=[ros2_controller],
        )
    )

    # Localization
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([localization_launch]),
        launch_arguments=[
            ('namespace', namespace),
            ('use_sim_time', LaunchConfiguration('use_sim_time'))
        ],
        condition=IfCondition(LaunchConfiguration('localization'))
    )

    # SLAM
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([slam_launch]),
        launch_arguments=[
            ('namespace', namespace),
            ('use_sim_time', LaunchConfiguration('use_sim_time'))
        ],
        condition=IfCondition(LaunchConfiguration('slam'))
    )

    # Nav2
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch]),
        launch_arguments=[
            ('namespace', ''),
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('localization', LaunchConfiguration('localization')),
            ('autostart', 'true'),
            ('params_file', PathJoinSubstitution(
                [pkg_robot_description, 'config', 'nav2_params.yaml'])),
            
        ],
        condition=IfCondition(LaunchConfiguration('nav2'))
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
    ld.add_action(realsense_d435i)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(ros2_controller_callback)
    #ld.add_action(nav2)
    ld.add_action(rviz)
    return ld