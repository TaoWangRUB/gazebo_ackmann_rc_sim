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
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

ARGUMENTS = [
    # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='false', choices=['true', 'false'],
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'localization', default_value='false', choices=['true', 'false'],
            description='Launch rtabmap in localization mode (a map should have been already created).'),
        
        DeclareLaunchArgument(
            'rtabmap_viz', default_value='true', choices=['true', 'false'],
            description='Launch rtabmap_viz for visualization.'),
]

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')
    rtabmap_viz = LaunchConfiguration('rtabmap_viz')

    icp_parameters={
          'odom_frame_id':'icp_odom',
          'guess_frame_id':'odom'
    }

    rtabmap_parameters={
          'subscribe_rgbd':True,
          'subscribe_scan':True,
          'use_action_for_goal':True,
          'odom_sensor_sync': True,
          # RTAB-Map's parameters should be strings:
          'Mem/NotLinkedNodesKept':'false'
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
          ('scan', '/scan'),
          ('odom', '/odom'),
          ('rgb/image', '/ackmann/depth_camera/image'),
          ('rgb/camera_info', '/ackmann/depth_camera/camera_info'),
          ('depth/image', '/ackmann/depth_camera/depth_image'),
          ('depth/camera_info', '/ackmann/depth_camera/camera_info')]
    
    # Nodes to launch
    rgbd_sync = Node(
        package='rtabmap_sync', executable='rgbd_sync', output='screen',
        parameters=[{'approx_sync':False, 'use_sim_time':use_sim_time}],
        remappings=remappings)

    icp_odom = Node(
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
            'angle_increment': 0.0175,  # ~1 degree resolution
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
                     'max_depth': 3.0,
                     'voxel_size': 0.02}],
        remappings=remappings)
    
    # Second, we segment the floor from the obstacles.
    parameters={
          'frame_id':'base_footprint',
          'use_sim_time':use_sim_time,
          'subscribe_depth':True,
          'use_action_for_goal':True,
          'Reg/Force3DoF':'true',
          'Grid/RayTracing':'true', # Fill empty space
          'Grid/3D':'false', # Use 2D occupancy
          'Grid/RangeMax':'3',
          'Grid/NormalsSegmentation':'false', # Use passthrough filter to detect obstacles
          'Grid/MaxGroundHeight':'0.05', # All points above 5 cm are obstacles
          'Grid/MaxObstacleHeight':'0.4',  # All points over 1 meter are ignored
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
    #ld.add_action(icp_odom)
    ld.add_action(depth_to_scan)
    #ld.add_action(rgbd_to_points)
    #ld.add_action(obstacle_detection)
    ld.add_action(slam)
    ld.add_action(localization)
    ld.add_action(rtabmap_viz)
    return ld
