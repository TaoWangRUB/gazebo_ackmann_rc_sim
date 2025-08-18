import os, xacro
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
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
    pkg_turtlebot4_viz = get_package_share_directory(
        'turtlebot4_viz')
    
    rviz_launch = PathJoinSubstitution(
        [pkg_turtlebot4_viz, 'launch', 'view_robot.launch.py'])
    
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
            '/ackmann/depth_camera/camera_info' + '@sensor_msgs/msg/CameraInfo' + '[ignition.msgs.CameraInfo',
            '/ackmann/depth_camera/points' + '@sensor_msgs/msg/PointCloud2' + '[ignition.msgs.PointCloudPacked',
            '/ackmann/depth_camera/depth_image' + '@sensor_msgs/msg/Image' + '[ignition.msgs.Image',
            '/ackmann/depth_camera/image' + '@sensor_msgs/msg/Image' + '[ignition.msgs.Image',
            '/ackmann/odom' + '@nav_msgs/msg/Odometry' + '[ignition.msgs.Odometry',
            '/ackmann/tf' + '@tf2_msgs/msg/TFMessage' + '[ignition.msgs.Pose_V',
            '/model/ackmann/tf' + '@tf2_msgs/msg/TFMessage' + '[ignition.msgs.Pose_V',
            #'/ackmann/joint_state' + '@sensor_msgs/msg/JointState' + '[ignition.msgs.Model',
            '/ackmann/cmd_vel' + '@geometry_msgs/msg/Twist' + ']ignition.msgs.Twist',
            '/rplidar/scan' + '@sensor_msgs/msg/LaserScan' + '[ignition.msgs.LaserScan',
            '/l515/imu/raw' + '@sensor_msgs/msg/Imu' + '[ignition.msgs.IMU',
            
        ],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
            #'qos_overrides./model/'+robot_ns+'.subscriber.reliability': 'reliable'
        }],
        output='screen',
        remappings=[ 
            #('/ackmann/tf', '/tf'),
            #('/ackmann/odom', '/odom'),
            #('/world/warehouse/model/ackmann/joint_state', '/joint_states'),
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
    
    ros2_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_ignition_bringup, 'launch', 'robot_control.launch.py')),
        launch_arguments=[
            ('namespace', namespace),
        ]
    ) 
    ros2_controller_callback = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[ros2_controller],
        )
    )
    # Add delayed controller spawning
    delayed_controller_spawning = TimerAction(
        period=3.0,
        actions=[ros2_controller_callback]
    )
    
    parameters=[{
          'frame_id': 'ackmann/base_footprint',
          'odom_frame': 'odom',
          'base_frame': 'ackmann/base_footprint',
          'map_frame': 'map',
          'publish_tf': False,
          'subscribe_rgbd': True,
          #'subscribe_depth':True,
          #'subscribe_odom_info':True,
          'odom_sensor_sync': True,
          'approx_sync':True,
          'approx_sync_max_interval': 0.05,
          'wait_imu_to_init': True,
          'wait_for_transform': 0.2,
          'wait_for_odom_to_init': True,
          'use_sim_time': LaunchConfiguration('use_sim_time'),
          # RTAB-Map's parameters should be strings:
          'Mem/NotLinkedNodesKept':'false',
          'Grid/MaxGroundHeight': '0.1',        # Maximum height of ground points
          'Grid/MaxObstacleHeight': '0.8',      # Maximum height of obstacle points
          'Grid/NormalsSegmentation': 'false',   # Enable normals segmentation
          #'Grid/RangeMax': '100',                # Maximum range for point cloud processing
          'Grid/3D': 'false',                   # Use 2D grid for navigation
          'Grid/RayTracing': 'true',            # Enable ray tracing for better obstacle detection
          'Reg/Strategy':'1',                   # Use 3D->2D visual odometry
          'Reg/Force3DoF':'true',               # Force 3 DoF for 2D navigation
          'Mem/NotLinkedNodesKept':'false',     # Do not keep not linked nodes
          }
    ]

    remappings=[
          ('imu', '/imu/data'),#'/l515/imu/raw' '/imu/data'
          ('rgb/image', '/ackmann/depth_camera/image'),
          ('rgb/camera_info', '/ackmann/depth_camera/camera_info'),
          ('depth/image', '/ackmann/depth_camera/depth_image'),
          ('depth/camera_info', '/ackmann/depth_camera/camera_info')]
    
    # Nodes to launch
    rgbd_sync = Node(
        package='rtabmap_sync', executable='rgbd_sync', output='screen',
        parameters=[{
            'approx_sync':False,  # Use exact sync for better accuracy
            'approx_sync_max_interval': 0.05,  # Maximum interval for approximate sync
            'use_sim_time':LaunchConfiguration('use_sim_time')}],
        remappings=remappings)
    
    ekf_filter_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',

            parameters=[#os.path.join(pkg_robot_ignition_bringup, 'config', 'ekf.yaml'),
                {"frequency": 30.0,
                 "predict_to_current_time": True,
                 "history_length": 5.0, 
                 "transform_time_offset": 0.1,  
                 "transform_timeout": 0.2,  
                 "publish_tf": True,
                 "two_d_mode": True,  # CRITICAL: Enable 2D mode
                 "use_sim_time": LaunchConfiguration('use_sim_time'),
                 "map_frame": "map",                # Defaults to "map" if unspecified
                 "odom_frame": "odom",              # Defaults to "odom" if unspecified
                 "base_link_frame": "ackmann/base_footprint",  # Defaults to "base_link" if unspecified
                 "world_frame": "odom",             # Defaults to the value of odom_frame if unspecified
                 "odom0": "/odom",
                 "odom0_config": [True, True, False,      # x, y, z (disable z for 2D)
                                  False, False, True,     # roll, pitch, yaw (only yaw)
                                  True, True, False,      # x_vel, y_vel, z_vel (disable z_vel)
                                  False, False, True,     # roll_vel, pitch_vel, yaw_vel (only yaw_vel)
                                  False, False, False],   # accelerations (disable all)
                 "odom0_queue_size": 10,
                 "odom0_nodelay": False,
                 "odom0_differential": True,
                 "odom0_relative": True,
                 # Higher noise for visual odometry (less trust)
                 #"odom0_pose_noise": [0.5, 0.5, 0.0, 0.0, 0.0, 0.2],   # Higher position noise
                 #"odom0_twist_noise": [0.3, 0.3, 0.0, 0.0, 0.0, 0.15], # Higher velocity noise
                 # Wheel encoder odometry (new)
                 "odom1": "/ackmann/odom",                 # ✅ Wheel encoder topic
                 "odom1_config": [True, True, False,      # x, y, z (same as odom0)
                                False, False, True,     # roll, pitch, yaw
                                True, True, False,      # x_vel, y_vel, z_vel
                                False, False, True,     # roll_vel, pitch_vel, yaw_vel
                                False, False, False],   # accelerations
                 "odom1_queue_size": 10,
                 "odom1_nodelay": False,
                 "odom1_differential": True,
                 "odom1_relative": True,
                 # Lower noise for wheel encoders (more trust)
                 #"odom1_pose_noise": [0.1, 0.1, 0.0, 0.0, 0.0, 0.05],  # Lower position noise  
                 #"odom1_twist_noise": [0.05, 0.05, 0.0, 0.0, 0.0, 0.02], # Lower velocity noise

                 "imu0": "imu/data", #"/l515/imu/raw",
                 "imu0_config": [False, False, False,    # position (disable all)
                                 False, False, True,      # orientation (only yaw)
                                 False, False, False,     # linear velocity (disable all)
                                 False, False, True,      # angular velocity (only yaw)
                                 False, False, False],      # linear acceleration (x, y only)
                 "imu0_queue_size": 10,
                 "imu0_nodelay": False,
                 "imu0_differential": False,
                 "imu0_relative": True,  # Important for IMU
                 "imu0_remove_gravitational_acceleration": True,
                 # Process noise covariance (conservative values)
                 "process_noise_covariance": [0.05, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                            0, 0.05, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                            0, 0, 0.06, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                            0, 0, 0, 0.03, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                            0, 0, 0, 0, 0.03, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                            0, 0, 0, 0, 0, 0.06, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                            0, 0, 0, 0, 0, 0, 0.025, 0, 0, 0, 0, 0, 0, 0, 0,
                                            0, 0, 0, 0, 0, 0, 0, 0.025, 0, 0, 0, 0, 0, 0, 0,
                                            0, 0, 0, 0, 0, 0, 0, 0, 0.04, 0, 0, 0, 0, 0, 0,
                                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0.01, 0, 0, 0, 0, 0,
                                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.01, 0, 0, 0, 0,
                                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.02, 0, 0, 0,
                                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.01, 0, 0,
                                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.01, 0,
                                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.015],
                 # Initial estimate covariance (diagonal matrix)
                 "initial_estimate_covariance": [1e-9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                            0, 1e-9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                            0, 0, 1e-9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                            0, 0, 0, 1e-9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                            0, 0, 0, 0, 1e-9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                            0, 0, 0, 0, 0, 1e-9, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                            0, 0, 0, 0, 0, 0, 1e-9, 0, 0, 0, 0, 0, 0, 0, 0,
                                            0, 0, 0, 0, 0, 0, 0, 1e-9, 0, 0, 0, 0, 0, 0, 0,
                                            0, 0, 0, 0, 0, 0, 0, 0, 1e-9, 0, 0, 0, 0, 0, 0,
                                            0, 0, 0, 0, 0, 0, 0, 0, 0, 1e-9, 0, 0, 0, 0, 0,
                                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1e-9, 0, 0, 0, 0,
                                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1e-9, 0, 0, 0,
                                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1e-9, 0, 0,
                                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1e-9, 0,
                                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1e-9]
                            }]
    )
    
    # IMU transform node to convert IMU data to the robot's base frame
    imu_transform_node = Node(
        package='imu_transformer',
        executable='imu_transformer_node',
        parameters=[{
            'target_frame': 'ackmann/base_footprint',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        remappings=[
            ('imu_in', '/l515/imu/raw'),
            ('imu_out', '/l515/imu/raw_transformed'),
        ]
    )      
    # Compute quaternion of the IMU
    imu_filter_node = Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
            parameters=[{'use_mag': False, 
                         'world_frame':'enu', # ned, enu, nwu
                         #'yaw_offset': 1.5708,
                         'publish_tf':False,
                         'reverse_tf': False,
                         #'fixed_frame': "odom",
                         }],
            remappings=[('imu/data_raw', '/l515/imu/raw_transformed'),])
    
    vio_node = Node(
            package='rtabmap_odom', 
            executable='rgbd_odometry', 
            output='screen',
            parameters=parameters + [
                {'Odom/Strategy': '0'},             # Frame-to-Map visual odometry
                {'Vis/MinInliers': '10'},           # Minimum inliers for robust matching
                {'Vis/FeatureType': '6'},           # ORB features (6), robust for visual odometry
                {'Vis/MaxFeatures': '1000'},        # Max features to detect
                {'Vis/EstimationType': '1'},        # 3D->2D (PnP) for 2D navigation
                {'Vis/MaxDepth': '20.0'},           # Max depth for point cloud
                {'Odom/GuessMotion': 'true'},       # Use motion model for better initial guess
                {'Odom/GuessSmoothingDelay': '0.1'},# Smoothing delay for guess motion
            ],
            remappings=remappings)
    
    slam_node = Node(
        package='rtabmap_slam', 
        executable='rtabmap', 
        output='screen',
        parameters=parameters + [
            {'publish_tf': False},
            {'publish_map_tf': True},
            #{'publish_odom_tf': True},
            {'localization': LaunchConfiguration('localization')}
            # Add more parameters as needed
        ],
        remappings=remappings + [
            ('odom', '/odometry/filtered'),
        ],
        arguments=['-d'])
    # Open RViz
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rviz_launch]),
        launch_arguments=[
            ('namespace', ''),
            ('use_sim_time', LaunchConfiguration('use_sim_time'))],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    rtabmap_viz = Node(
        condition=UnlessCondition(LaunchConfiguration('rviz')),
        package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        remappings=remappings
    )

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(ign_resource_path)
    ld.add_action(ignition_gazebo)
    ld.add_action(clock_bridge)
    ld.add_action(robot_state_publisher)
    ld.add_action(tf_pub)
    ld.add_action(topic_bridge)
    ld.add_action(gz_spawn_entity)
    #ld.add_action(delayed_controller_spawning)  # Add delay
    ld.add_action(rgbd_sync)
    ld.add_action(imu_transform_node)
    ld.add_action(imu_filter_node)
    ld.add_action(vio_node)
    ld.add_action(ekf_filter_node)
    ld.add_action(slam_node)
    ld.add_action(ros2_controller_callback)
    ld.add_action(rviz)
    ld.add_action(rtabmap_viz)
    return ld