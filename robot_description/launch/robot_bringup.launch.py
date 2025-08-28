#
# Note: Make sure you have this fix for turtlebot4_description https://github.com/turtlebot/turtlebot4/pull/434,
#       otherwise, the lidar and camera point cloud won't be aligned correctly.
#
# Example:
#   1) Launch simulator (turtlebot4, nav2 and rtabmap):
#     $ ros2 launch rtabmap_demos turtlebot4_sim_demo.launch.py
#
#   2) Click on "Play" button on bottom-left of gazebo.
#
#   3) Click on double points ".." button on top-right next to power button to undock.
#
#   4) Move the robot:
#     b) By sending goals with RVIZ's "Nav2 Goal" button in action bar.
#     a) By teleoperating:
#        $ ros2 run teleop_twist_keyboard teleop_twist_keyboard
#     c) By using autonomous exploration node (tested with https://github.com/robo-friends/m-explore-ros2):
#        $ ros2 launch explore_lite explore.launch.py
#

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition, UnlessCondition

ARGUMENTS = [
    DeclareLaunchArgument('rviz', default_value='true',
                          choices=['true', 'false'], description='Start rviz.'),
    DeclareLaunchArgument('rtabmap_viz', default_value='true',
                          choices=['true', 'false'], description='Start rtabmap_viz.'),
    DeclareLaunchArgument('localization', default_value='false',
                          choices=['true', 'false'], description='Start rtabmap in localization mode (a map should have been already created).'),
    DeclareLaunchArgument('nav2', default_value='true',
                          choices=['true', 'false'], description='Start nav2.'),
    DeclareLaunchArgument('world', default_value='warehouse',
                          description='Ignition World'),
    DeclareLaunchArgument('sim', default_value='true',
                          choices=['true', 'false'], description='start real or simulated robot'),
]

def generate_launch_description():
    # Directories
    pkg_robot_ignition_bringup = get_package_share_directory(
        'robot_description')
    pkg_rtabmap_demos = get_package_share_directory(
        'robot_description')

    # Paths
    ignition_launch = PathJoinSubstitution(
        [pkg_robot_ignition_bringup, 'launch', 'ackmann_ign.launch.py'])
    
    realsense_launch = PathJoinSubstitution(
        [pkg_robot_ignition_bringup, 'launch', 'realsense_d435i.launch.py'])
    rtabmap_launch = PathJoinSubstitution(
        [pkg_rtabmap_demos, 'launch', 'rtabmap_slam.launch.py'])

    # Ignition Gazebo
    ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ignition_launch]),
        launch_arguments=[
            ('world', LaunchConfiguration('world')),
            ('slam', 'false'),
            ('localization', LaunchConfiguration('localization')),
            ('nav2', LaunchConfiguration('nav2')),
            ('rviz', LaunchConfiguration('rviz')),
            ('y', '-2')
        ],
        condition=IfCondition(LaunchConfiguration('sim'))
    )

    # Real robot
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([realsense_launch]),
        launch_arguments=[
            ('rviz', LaunchConfiguration('rviz')),
            ('namespace', ''),
            ('use_sim_time', 'false')
        ],
        condition=UnlessCondition(LaunchConfiguration('sim'))
    )

    launch_arguments=[('rgb_image_topic', '/ackmann/depth_camera/image'),
                      ('rgb_camera_info_topic', '/ackmann/depth_camera/camera_info'),
                      ('depth_image_topic', '/ackmann/depth_camera/depth_image'),
                      ('depth_camera_info_topic', '/ackmann/depth_camera/camera_info'),
                      ('imu_raw_topic', '/l515/imu/raw')
                     ] if LaunchConfiguration('sim') else [
                        ('rgb_image_topic', '/d435i/color/image_raw'),
                        ('rgb_camera_info_topic', '/d435i/color/camera_info'),
                        ('depth_image_topic', '/d435i/depth/image_rect_raw'),
                        ('depth_camera_info_topic', '/d435i/depth/camera_info'),
                        ('imu_raw_topic', '/d435i/imu/raw')
                     ]
    
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rtabmap_launch]),
        launch_arguments=[
            ('rtabmap_viz', LaunchConfiguration('rtabmap_viz')),
            ('localization', LaunchConfiguration('localization')),
            ('use_sim_time', 'true'),
            ('rgb_image_topic', PythonExpression([
                '"/l515/image" if "true" == "', LaunchConfiguration('sim'), '" else "/d435i/color/image_raw"'
            ])),
            ('rgb_camera_info_topic', PythonExpression([
                '"/l515/camera_info" if "true" == "', LaunchConfiguration('sim'), '" else "/d435i/color/camera_info"'
            ])),
            ('depth_image_topic', PythonExpression([
                '"/l515/depth_image" if "true" == "', LaunchConfiguration('sim'), '" else "/d435i/depth/image_rect_raw"'
            ])),
            ('depth_camera_info_topic', PythonExpression([
                '"/l515/camera_info" if "true" == "', LaunchConfiguration('sim'), '" else "/d435i/depth/camera_info"'
            ])),
            ('imu_raw_topic', PythonExpression([
                '"/l515/imu/raw" if "true" == "', LaunchConfiguration('sim'), '" else "/d435i/imu"'
            ])),
        ]
    )

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(rtabmap) # put it first so that localization arg is not overwritten by the same used by ignition
    ld.add_action(ignition)
    ld.add_action(realsense)
    return ld
