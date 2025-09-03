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
    DeclareLaunchArgument('use_sim_time', default_value='false',
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
    pkg_ros_px4_offboard = get_package_share_directory(
        'px4_offboard')
    pkg_turtlebot4_navigation = get_package_share_directory(
        'turtlebot4_navigation')
    pkg_turtlebot4_viz = get_package_share_directory(
        'turtlebot4_viz')
    pkg_controller = get_package_share_directory(
        'robot_description')
    pkg_robot_description = get_package_share_directory(
        'robot_description')
    
    realsense_launch = PathJoinSubstitution(
        [pkg_ros_realsense, 'launch', 'rs_launch.py'])
    rviz_launch = PathJoinSubstitution(
        [pkg_turtlebot4_viz, 'launch', 'view_robot.launch.py'])
    pkg_offboard_launch = PathJoinSubstitution(
        [pkg_ros_px4_offboard, 'launch', 'offboard_control.launch.py'])
    robot_state_launch = PathJoinSubstitution(
        [pkg_robot_description, 'launch', 'robot_state.launch.py'])
    
    # Robot description
    namespace = '' #LaunchConfiguration('namespace')

    robot_state_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_state_launch]),
        launch_arguments=[
            ('rviz', 'false'),
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
        ]
    )

    # Camera launch
    realsense_d435i = IncludeLaunchDescription(realsense_launch,
        launch_arguments=[
            ('camera_namespace', ''),
            ('camera_name', 'd435i'),
            ('publish_tf', 'false'),

            ('accel_qos', 'SYSTEM_DEFAULT'),
            ('enable_accel', 'true'),
            ('gyro_qos', 'SYSTEM_DEFAULT'),
            ('enable_gyro', 'true'),
            ('unite_imu_method', LaunchConfiguration('unite_imu_method')),
            
            ('enable_sync', 'true'),
            ('enable_color', 'true'),
            ('enable_depth', 'true'),
            ('enable_rgbd', 'false'),
            ('align_depth.enable', 'true'),
            
            ('rgb_camera.exposure', '200'), # 1000us = 1 ms
            ('rgb_camera.gain', '128'),         
            ('rgb_camera.color_profile', '640x480x60'), # 0: 424x240x30, 1: 424x240x60, 2: 640x360x30, 3: 640x360x60, 4: 640x480x30, 5: 640x480x60, 6: 1280x720x30
            ('rgb_camera.enable_auto_exposure', 'false'),   

            ('depth_module.exposure', '7500'),
            ('depth_module.gain', '16'),                          
            ('depth_module.depth_profile', '640x480x60'), # 0: 424x240x30, 1: 424x240x60, 2: 640x360x30, 3: 640x360x60, 4: 640x480x30, 5: 640x480x60, 6: 1280x720x30
            ('depth_module.enable_auto_exposure', 'false'),
            
            ('enable_infra1', 'false'),
            ('enable_infra2', 'false'),
            ('pointcloud.enable', 'false'),
        ],
        #condition=IfCondition(LaunchConfiguration('use_sim_time'))
    )
    remappings=[
        ('scan', '/scan'),
        ('odom', '/vo_odom'),
        ('imu', '/imu/data'),
        ('rgb/image', '/d435i/color/image_raw'), 
        ('rgb/camera_info', '/d435i/color/camera_info'),
        ('depth/image', '/d435i/depth/image_rect_raw'),
        ('depth/camera_info', '/d435i/depth/camera_info'),
    ]

    # Nodes to launch
    rgbd_sync = Node(
        package='rtabmap_sync', executable='rgbd_sync', output='screen',
        parameters=[{
            'approx_sync':False,
            'approx_sync_max_interval': 0.05,  # Maximum interval for approximate sync
            'use_sim_time':LaunchConfiguration('use_sim_time')}],
        remappings=remappings)
    
    # IMU transform node to convert IMU data to the robot's base frame
    imu_transform_node = Node(
        package='imu_transformer',
        executable='imu_transformer_node',
        parameters=[{
            'target_frame': 'ackmann/base_footprint',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        remappings=[
            ('imu_in', '/d435i/imu'), 
            ('imu_out', '/imu/raw_transformed'),
        ]
    )     

    # IMU filter node
    imu_filter_node = Node(
        package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
        parameters=[{'use_mag': False, 
                     'world_frame':'enu', # ned, enu, nwu
                     #'yaw_offset': -1.5708,
                     'publish_tf':False,
                     #'fixed_frame': "camera_link"
        }],
        remappings=[('imu/data_raw', '/imu/raw_transformed'),  # Use transformed IMU data
        ]
    )

    # Odometry node
    visual_odom_parameters = {
        'frame_id': 'ackmann/base_footprint',   # Defaults to "base_footprint" if unspecified
        'odom_frame_id': 'odom',                # Defaults to "odom" if unspecified
        #'guess_frame_id': 'ackmann/base_footprint',
        'publish_tf': False,
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'Odom/Strategy': '0',  # Frame-to-Map visual odometry
        'Vis/MinInliers': '10',  # Minimum inliers for robust matching
        'Vis/FeatureType': '6',  # ORB features (6), robust for visual odometry
        'Vis/MaxFeatures': '1000',  # Max features to detect
        'Vis/EstimationType': '1',  # 3D->2D (PnP) for 2D navigation
        'Vis/MaxDepth': '20.0',  # Max depth for point cloud
        'Odom/GuessMotion': 'true',  # Use motion model for better initial guess
        'Odom/GuessSmoothingDelay': '0.1',
    }
    visual_odom = Node(
        package='rtabmap_odom', executable='rgbd_odometry', output='screen',
        parameters=[visual_odom_parameters],
        remappings=remappings,
        arguments=["--ros-args", "--log-level", 'rgbd_odometry:=warn']
    )
    
    # EKF node
    ekf_filter_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[#control_params_file,
                {"frequency": 100.0,
                 "predict_to_current_time": True,
                 "history_length": 5.0, 
                 "use_sim_time": LaunchConfiguration('use_sim_time'),
                 "two_d_mode": True,  # Often helpful for ground robots
                 "publish_tf": True,
                 "map_frame": "map",                # Defaults to "map" if unspecified
                 "odom_frame": "odom",           # Defaults to "odom" if unspecified
                 "base_link_frame": "ackmann/base_footprint",    # Defaults to "base_link" if unspecified
                 "world_frame": "odom",             # Defaults to the value of odom_frame if unspecified
                 
                 # Additional stability parameters
                 "sensor_timeout": 0.2,  # Wait for sensor data
                 "transform_timeout": 0.2,
                 "transform_time_offset": 0.1,

                 "odom0": "/vo_odom",
                 "odom0_config": [True, True, False,    # x, y, z position
                                  False, False, True,     # roll, pitch, yaw
                                  True, True, False,     # x, y, z velocity
                                  False, False, True,     # roll, pitch, yaw rates
                                  False, False, False], # x, y, z acceleration
                 "odom0_queue_size": 10,
                 "odom0_nodelay": False,
                 "odom0_differential": False,
                 "odom0_relative": True,
                 #"odom0_pose_noise": [0.01, 0.01, 0.01, 0.01, 0.01, 0.01],  # Lower noise for odometry
                 #"odom0_twist_noise": [0.01, 0.01, 0.01, 0.01, 0.01, 0.01],
                 
                 # IMU Configuration  
                 "imu0": "/imu/data", #"/l515/imu/data",
                 "imu0_config": [False, False, False,   # x, y, z position
                                False,  False,  True,    # roll, pitch, yaw
                                False, False, False,   # x, y, z velocity
                                True,  True,  True,    # roll, pitch, yaw rates
                                True,  True,  False],   # x, y, z acceleration
                 "imu0_queue_size": 10,
                 "imu0_nodelay": False,
                 "imu0_differential": False,
                 "imu0_relative": True,
                 "imu0_remove_gravitational_acceleration": True,
                
                }]
    )
    
    # SLAM Mode:
    rtabmap_parameters={
        'subscribe_rgbd':True,
        'subscribe_scan':False,
        'subscribe_odom':True,      # Use odometry
        'use_action_for_goal':True,
        'odom_sensor_sync': True,   
        # RTAB-Map's parameters should be strings:
        'Mem/NotLinkedNodesKept':'false',
        'Grid/MaxGroundHeight': '0.1',
        'Grid/MaxObstacleHeight': '0.8',
        'Grid/NormalsSegmentation': 'false',
        #'Grid/RangeMax': '20',
        'Grid/3D': 'false',
        'Grid/RayTracing': 'true',
        
        'frame_id':'ackmann/base_footprint',
        'use_sim_time':LaunchConfiguration('use_sim_time'),
        # RTAB-Map's parameters should be strings:
        'Reg/Strategy':'1',     # 1: 3D->2D (PnP) for 2D navigation
        'Reg/Force3DoF':'true', # Force 3 DoF (2D) registration
        'Mem/NotLinkedNodesKept':'false',
        'Icp/PointToPlaneMinComplexity':'0.04' # to be more robust to long corridors with low geometry
    }

    slam = Node(
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=[rtabmap_parameters],
        remappings=remappings + [
            ('odom', "/odometry/filtered"),     # '/odometry/filtered'
        ],
        arguments=['-d'])
    
    # Px4 offboard control
    offboard_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pkg_offboard_launch]),
        launch_arguments=[
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('odom_topic', '/odometry/filtered'),
        ]
    )

    # Open RViz
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rviz_launch]),
        launch_arguments=[
            ('namespace', ''),
        ],
        #condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(robot_state_launch)
    ld.add_action(realsense_d435i)
    ld.add_action(imu_transform_node)
    ld.add_action(imu_filter_node)
    ld.add_action(rgbd_sync)
    ld.add_action(visual_odom)
    ld.add_action(ekf_filter_node)
    ld.add_action(slam)
    ld.add_action(offboard_control)
    ld.add_action(rviz)
    return ld