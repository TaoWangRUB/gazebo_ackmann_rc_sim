import os, xacro
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

robot_base_color = '0.0 0.0 1.0 0.95' #Ign and Rviz color of the robot's main body (rgba)

ARGUMENTS = [
    DeclareLaunchArgument('robot_name', default_value='ackmann',
                          description='Robot name'),
    DeclareLaunchArgument('namespace', default_value=LaunchConfiguration('robot_name'),
                          description='Robot namespace'),
    DeclareLaunchArgument('rviz', default_value='false',
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
    pkg_irobot_create_description = get_package_share_directory(
        'irobot_create_description')
    pkg_irobot_create_ignition_bringup = get_package_share_directory(
        'irobot_create_ignition_bringup')
    pkg_ros_ign_gazebo = get_package_share_directory(
        'ros_ign_gazebo')
    pkg_turtlebot4_navigation = get_package_share_directory(
        'turtlebot4_navigation')
    pkg_turtlebot4_viz = get_package_share_directory(
        'turtlebot4_viz')
    
    rviz_launch = PathJoinSubstitution(
        [pkg_turtlebot4_viz, 'launch', 'view_robot.launch.py'])
    localization_launch = PathJoinSubstitution(
        [pkg_turtlebot4_navigation, 'launch', 'localization.launch.py'])
    slam_launch = PathJoinSubstitution(
        [pkg_turtlebot4_navigation, 'launch', 'slam.launch.py'])
    nav2_launch = PathJoinSubstitution(
        [pkg_turtlebot4_navigation, 'launch', 'nav2.launch.py'])
    
    # Set ignition resource path
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.path.join(pkg_robot_ignition_bringup, 'models'), ':' +
            os.path.join(pkg_robot_ignition_bringup, 'worlds'), ':' +
            os.path.join(pkg_irobot_create_ignition_bringup, 'worlds'), ':' +
            str(Path(pkg_irobot_create_description).parent.resolve())])
    
    # Paths
    ign_gazebo_launch = PathJoinSubstitution(
        [pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py'])
    ignition_gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ign_gazebo_launch]),
            launch_arguments=[
                ('ign_args', [LaunchConfiguration('world'),
                              '.sdf',
                              ' -r',
                              ' -v 4']
                )
            ]
    )

    # Clock bridge
    clock_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                        name='clock_bridge',
                        output='screen',
                        arguments=[
                            '/clock' + '@rosgraph_msgs/msg/Clock' + '[ignition.msgs.Clock'
                        ]
    )
    
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
                'namespace:=', namespace])},
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
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    # Spawn robot in Gazebo
    x, y, z = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    robot_name = LaunchConfiguration('robot_name')

    # Spawn entity in Ignition Gazebo
    gz_spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=[
                   '-topic', '/robot_description',
                   '-x', x, 
                   '-y', y, 
                   '-z', z,
                   '-R', '0.0', '-P', '0.0', '-Y', yaw,
                   '-name', robot_name,
                   ],
    )

    # ROS IGN Bridge
    topic_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[             # ign topic -t <topic_name> --info
            '/ackmann/depth_camera/camera_info' + '@sensor_msgs/msg/CameraInfo' + '@ignition.msgs.CameraInfo',
            '/ackmann/depth_camera/points' + '@sensor_msgs/msg/PointCloud2' + '@ignition.msgs.PointCloudPacked',
            '/ackmann/depth_camera/depth_image' + '@sensor_msgs/msg/Image' + '@ignition.msgs.Image',
            '/ackmann/depth_camera/image' + '@sensor_msgs/msg/Image' + '@ignition.msgs.Image',
            '/ackmann/odom' + '@nav_msgs/msg/Odometry' + '@ignition.msgs.Odometry',
            '/ackmann/tf' + '@tf2_msgs/msg/TFMessage' + '[ignition.msgs.Pose_V',
            '/ackmann/cmd_vel' + '@geometry_msgs/msg/Twist' + '@ignition.msgs.Twist',
            '/rplidar/scan' + '@sensor_msgs/msg/LaserScan' + '@ignition.msgs.LaserScan'
            
        ],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
            #'qos_overrides./model/'+robot_ns+'.subscriber.reliability': 'reliable'
        }],
        output='screen',
        remappings=[ 
            ('/ackmann/tf', '/tf'),
            ('/ackmann/odom', '/odom'),
        ]
    ) 

    # Static transform publisher for depth camera
    tf_pub = Node(
        name='camera_stf',
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=[
            '0', '0', '0',
            '1.5707', '-1.5707', '0',
            'l515_depth_optical_frame',
            [robot_name, '/ackmann/base_footprint/rgbd_camera']
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
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
            ('slam', LaunchConfiguration('slam')),
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
            ('use_sim_time', LaunchConfiguration('use_sim_time'))],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(ign_resource_path)
    ld.add_action(ignition_gazebo)
    ld.add_action(clock_bridge)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(gz_spawn_entity)
    ld.add_action(tf_pub)
    ld.add_action(topic_bridge)
    #ld.add_action(localization)
    #ld.add_action(slam)
    #ld.add_action(nav2)
    ld.add_action(rviz)
    return ld