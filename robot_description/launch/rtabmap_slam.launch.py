#
# Note: Make sure you have this fix for turtlebot4_description https://github.com/turtlebot/turtlebot4/pull/434,
#       otherwise, the lidar and camera point cloud won't be aligned correctly.
#
# Example with gazebo:
#   1) Launch simulator (turtlebot4 and nav2):
#     $ ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py slam:=false nav2:=true rviz:=true
#
#   2) Launch SLAM:
#     $ ros2 launch rtabmap_demos turtlebot4_slam.launch.py use_sim_time:=true
#     OR
#     $ ros2 launch rtabmap_launch rtabmap.launch.py rtabmap_viz:=true subscribe_scan:=true rgbd_sync:=true depth_topic:=/oakd/rgb/preview/depth odom_sensor_sync:=true camera_info_topic:=/oakd/rgb/preview/camera_info rgb_topic:=/oakd/rgb/preview/image_raw visual_odometry:=false approx_sync:=true approx_rgbd_sync:=false odom_guess_frame_id:=odom icp_odometry:=true odom_topic:="icp_odom" map_topic:="/map" use_sim_time:=true odom_log_level:=warn rtabmap_args:="--delete_db_on_start --Reg/Strategy 1 --Reg/Force3DoF true --Mem/NotLinkedNodesKept false" use_action_for_goal:=true
#
#   3) Click on "Play" button on bottom-left of gazebo.
#
#   4) Click on double points ".." button on top-right next to power button to undock.
#
#   5) Move the robot:
#     b) By sending goals with RVIZ's "Nav2 Goal" button in action bar.
#     a) By teleoperating:
#        $ ros2 run teleop_twist_keyboard teleop_twist_keyboard
#     c) By using autonomous exploration node (tested with https://github.com/robo-friends/m-explore-ros2):
#        $ ros2 launch explore_lite explore.launch.py
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

ARGUMENTS = [
    # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='false', choices=['true', 'false'],
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'localization', default_value='false', choices=['true', 'false'],
            description='Launch rtabmap in localization mode (a map should have been already created).'),
        
        DeclareLaunchArgument(
            'vision', default_value='true', choices=['true', 'false'],
            description='Using vision odometry or icp odometry.'),
        
        DeclareLaunchArgument(
            'rtabmap_viz', default_value='true', choices=['true', 'false'],
            description='Launch rtabmap_viz for visualization.'),
]

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')
    vision = LaunchConfiguration('vision')
    rtabmap_viz = LaunchConfiguration('rtabmap_viz')
    
    # Define odom topic based on vision
    odom_topic = PythonExpression([
        '"/vo_odom" if "', LaunchConfiguration('vision'), '" == "true" else "/icp_odom"'
    ])

    # Define subscribe_scan based on vision (True when vision is false)
    subscribe_scan = PythonExpression([
        'True if "', LaunchConfiguration('vision'), '" == "false" else False'
    ])
    # Define scan topic based on vision (True when vision is false)
    scan_topic = PythonExpression([
        '"/scan" if "', LaunchConfiguration('vision'), '" == "false" else "/scan"'
    ])

    rtabmap_parameters={
        'subscribe_rgbd':True,
        'subscribe_scan':subscribe_scan,
        'use_action_for_goal':True,
        'odom_sensor_sync': True,
        # RTAB-Map's parameters should be strings:
        'Mem/NotLinkedNodesKept':'false',
        'Grid/MaxGroundHeight': '0.1',
        'Grid/MaxObstacleHeight': '0.8',
        'Grid/NormalsSegmentation': 'true',
        'Grid/RangeMax': '20',
        'Grid/3D': 'false',
        'Grid/RayTracing': 'true'
    }

    # Shared parameters between different nodes
    shared_parameters={
        'frame_id':'ackmann/base_footprint',
        'use_sim_time':use_sim_time,
        # RTAB-Map's parameters should be strings:
        'Reg/Strategy':'1',
        'Reg/Force3DoF':'true',
        'Mem/NotLinkedNodesKept':'false',
        'Icp/PointToPlaneMinComplexity':'0.04' # to be more robust to long corridors with low geometry
    }

    remappings=[
        ('scan', scan_topic),
        ('odom', odom_topic),
        ('rgb/image', '/ackmann/depth_camera/image'),
        ('rgb/camera_info', '/ackmann/depth_camera/camera_info'),
        ('depth/image', '/ackmann/depth_camera/depth_image'),
        ('depth/camera_info', '/ackmann/depth_camera/camera_info')]
    
    # Nodes to launch
    rgbd_sync = Node(
        package='rtabmap_sync', executable='rgbd_sync', output='screen',
        parameters=[{'approx_sync':False, 'use_sim_time':use_sim_time}],
        remappings=remappings)

    # EKF filter node for localization
    # This node uses the Extended Kalman Filter to fuse odometry and IMU data.
    # It publishes the estimated pose to the /odom topic.
    # It is used to provide a more accurate pose estimate for the robot.
    # It is a part of the robot_localization package.
    pkg = get_package_share_directory('robot_description')
    control_params_file = PathJoinSubstitution([pkg, 'config', 'ekf.yaml'])
    ekf_filter_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[control_params_file,
                {"frequency": 50.0,
                 "predict_to_current_time": True,
                 "publish_tf": True,
                 "map_frame": "map",                # Defaults to "map" if unspecified
                 "odom_frame": "odom",           # Defaults to "odom" if unspecified
                 "base_link_frame": "ackmann/base_footprint",    # Defaults to "base_link" if unspecified
                 "world_frame": "odom",             # Defaults to the value of odom_frame if unspecified
                 "odom0": "/vo_odom",
                 "odom0_config": [True, True, False,    # x, y, z position
                                  False, False, True,     # roll, pitch, yaw
                                  False, False, False,     # x, y, z velocity
                                  False, False, False,     # roll, pitch, yaw rates
                                  False, False, False], # x, y, z acceleration
                 "odom0_queue_size": 10,
                 "odom0_nodelay": False,
                 # IMU Configuration  
                "imu0": "/l515/imu/data",
                "imu0_config": [False, False, False,   # x, y, z position
                                True,  True,  True,    # roll, pitch, yaw
                                False, False, False,   # x, y, z velocity
                                True,  True,  True,    # roll, pitch, yaw rates
                                True,  True,  True],   # x, y, z acceleration
                 "imu0_queue_size": 50,
                 "imu0_nodelay": False,
                 "odom0_pose_noise": [0.01, 0.01, 0.01, 0.01, 0.01, 0.01],  # Lower noise for odometry
                 "odom0_twist_noise": [0.01, 0.01, 0.01, 0.01, 0.01, 0.01],
                 # OVERRIDE zero covariances from Gazebo
                "imu0_pose_covariance": [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.01, 0.0, 0.0,  # Lower noise for sim
                                        0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.01],
                
                "imu0_angular_velocity_covariance": [0.001, 0.0, 0.0,
                                                    0.0, 0.001, 0.0,
                                                    0.0, 0.0, 0.001],
                
                "imu0_linear_acceleration_covariance": [0.01, 0.0, 0.0,
                                                        0.0, 0.01, 0.0,
                                                        0.0, 0.0, 0.01],
                
                # Gazebo-specific settings
                "imu0_differential": False,
                "imu0_relative": False,
                "imu0_remove_gravitational_acceleration": True,
                "two_d_mode": True,  # Often helpful for ground robots
                }]
    )

    # IMU filter node
    # This node filters the IMU data using the Madgwick filter.
    # It publishes the filtered IMU data to the /imu/data topic.
    # It is used to provide a more accurate IMU data for the robot.
    imu_filter_node = Node(
        package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
        parameters=[{'use_mag': False, 
                     'world_frame':'nwu', # ned, enu, nwu
                     #'yaw_offset': -1.5708,
                     'publish_tf':False,
                     #'fixed_frame': "camera_link"
                     }],
        remappings=[('imu/data_raw', '/l515/imu/data')]
    )
    # Visual Odometry node
    # This node uses visual odometry to estimate the pose of the robot based on the camera images.
    # It publishes the estimated pose to the /vo_odom topic.
    # It is used to provide an initial guess for the RTAB-Map SLAM algorithm
    # and to provide a more accurate pose estimate for the robot.
    visual_odom_parameters = {
        'frame_id': 'ackmann/base_footprint',
        'odom_frame_id': 'vo_odom',
        'guess_frame_id': 'ackmann/odom',
        'publish_tf': True,
        'use_sim_time': use_sim_time,
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
        condition=IfCondition(vision),
        package='rtabmap_odom', executable='rgbd_odometry', output='screen',
        parameters=[visual_odom_parameters],
        remappings=remappings,
        arguments=["--ros-args", "--log-level", 'rgbd_odometry:=warn'])
    
    # ICP Odometry node
    # This node uses the ICP algorithm to estimate the odometry of the robot based on the depth images.
    # It publishes the estimated odometry to the /icp_odom topic.
    # It is used to provide an initial guess for the RTAB-Map SLAM algorithm
    # and to provide a more accurate odometry estimate for the robot.
    # It is a part of the RTAB-Map SLAM framework.
    icp_parameters={
        'odom_frame_id':'icp_odom',
        'guess_frame_id':'ackmann/odom',
        'publish_tf': True,  # Add this
        'Icp/CorrespondenceRatio': '0.03',  # Further relax from 0.05
        'Icp/PointToPlaneMinComplexity': '0.01',  # Lower for corridors
        'Icp/MaxCorrespondenceDistance': '0.2',  # Increase from 0.1
        'Icp/VoxelSize': '0.05',  # Enable voxel filtering
        'Icp/MaxTranslation': '1.0',  # Allow larger translation
        'Icp/MaxRotation': '1.0',  # Allow larger rotation
        'Odom/GuessSmoothingDelay': '0.3',  # Increase smoothing
        'Odom/GuessMotion': 'true',
    }
    icp_odom = Node(
        condition=UnlessCondition(vision),
        package='rtabmap_odom', executable='icp_odometry', output='screen',
        parameters=[icp_parameters, shared_parameters],
        remappings=remappings,
        arguments=["--ros-args", "--log-level", 'icp_odometry:=warn'])

    # SLAM Mode:
    slam = Node(
        condition=UnlessCondition(localization),
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=[rtabmap_parameters, shared_parameters],
        remappings=remappings,
        arguments=['-d'])
        
    # Localization mode:
    localization = Node(
        condition=IfCondition(localization),
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=[rtabmap_parameters, shared_parameters,
            {'Mem/IncrementalMemory':'False',
            'Mem/InitWMWithAllNodes':'True'}],
        remappings=remappings)

    rtabmap_viz = Node(
        condition=IfCondition(rtabmap_viz),
        package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        parameters=[rtabmap_parameters, shared_parameters],
        remappings=remappings)
    
    # rgbd to laserscan node 
    # This node converts depth images to laser scans, which can be used for navigation.
    depth_to_scan = Node(
        condition=IfCondition(vision),
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='rgbd_to_scan',
        parameters=[{
            'scan_height': 10,          # Number of pixel rows to use
            'range_min': 0.1,           # Minimum range (meters)
            'range_max': 20.,           # Maximum range (meters)
            'output_frame': 'ackmann/base_footprint',
            'angle_min': -3.1415,       # -π radians
            'angle_max': 3.1415,        # π radians
            'angle_increment': 0.0087,  # ~1 degree resolution
        }],
        remappings=[
            ('depth', '/ackmann/depth_camera/depth_image'),
            ('depth_camera_info', '/ackmann/depth_camera/camera_info'),
            ('scan', '/scan')
        ],
    )

    # Obstacle detection with the camera for nav2 local costmap.
    # First, we need to convert depth image to a point cloud.
    rgbd_to_points = Node(
        package='rtabmap_util', executable='point_cloud_xyz', output='screen',
        parameters=[{'decimation': 2,
                     'max_depth': 20.0,
                     'voxel_size': 0.02}],
        remappings=remappings)
    
    # Second, we segment the floor from the obstacles.
    parameters={
          'frame_id':'ackmann/base_footprint',
          'use_sim_time':use_sim_time,
          'subscribe_depth':True,
          'use_action_for_goal':True,
          'Reg/Force3DoF':'true',
          'Grid/RayTracing':'true', # Fill empty space
          'Grid/3D':'false', # Use 2D occupancy
          'Grid/RangeMax':'3',
          'Grid/NormalsSegmentation':'true', # Use passthrough filter to detect obstacles
          'Grid/MaxGroundHeight':'0.1', # All points above 5 cm are obstacles
          'Grid/MaxObstacleHeight':'0.8',  # All points over 1 meter are ignored
          'Optimizer/GravitySigma':'0' # Disable imu constraints (we are already in 2D)
    }

    obstacle_detection = Node(
        package='rtabmap_util', executable='obstacles_detection', output='screen',
        parameters=[parameters],
        remappings=[('obstacles', '/camera/obstacles'),
                    ('ground', '/camera/ground')])
    
    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(rgbd_sync)
    ld.add_action(depth_to_scan)
    ld.add_action(ekf_filter_node)
    #ld.add_action(imu_filter_node)
    ld.add_action(visual_odom)
    ld.add_action(icp_odom)
    #ld.add_action(rgbd_to_points)
    #ld.add_action(obstacle_detection)
    ld.add_action(slam)
    ld.add_action(localization)
    ld.add_action(rtabmap_viz)
    return ld
