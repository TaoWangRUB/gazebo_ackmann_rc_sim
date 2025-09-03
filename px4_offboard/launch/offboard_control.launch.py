#!/usr/bin/env python

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os

ARGUMENTS = [
    DeclareLaunchArgument('odom_topic', default_value='/odom',
                          description='input odometry topic'),
]

def generate_launch_description():
    package_dir = get_package_share_directory('px4_offboard')
    visualizer_node = Node(
        package='px4_offboard',
        namespace='px4_offboard',
        executable='visualizer',
        name='visualizer'
    )
    
    pose_control_dds_node = Node(
        package='px4_offboard',
        namespace='px4_offboard',
        executable='offboard_control',
        name='dds_offboard_controller',
        parameters= [{'radius': 10.0},{'altitude': 5.0},{'omega': 0.5}]
    )
    
    velocity_control_dds_node = Node(
        package='px4_offboard',
        namespace='px4_offboard',
        executable='velocity_control',
        name='dds_offboard_controller',
        parameters= [{'radius': 10.0},{'altitude': 5.0},{'omega': 0.5}]
    )
    
    rc_control_dds_node = Node(
        package='px4_offboard',
        namespace='px4_offboard',
        executable='rc_control_dds',
        name='mavros_offboard_controller_4',
        parameters= [],
        remappings={
            ('/fmu/out/vehicle_status', '/fmu/out/vehicle_status_v1'),
        }
    )
    
    vio_to_px4_dds_node = Node(
        package='px4_offboard',
        namespace='px4_offboard',
        executable='vio_to_px4',
        name='vio_to_dds',
        parameters= [],
        remappings={
            ('/odom', LaunchConfiguration('odom_topic')),
        }
    )
    
    velocity_control_mavros_node = Node(
        package='px4_offboard',
        namespace='px4_offboard',
        executable='velocity_control_mavros',
        name='mavros_offboard_controller_1',
        parameters= []
    )
    
    pose_control_mavros_node = Node(
        package='px4_offboard',
        namespace='px4_offboard',
        executable='pose_control_mavros',
        name='mavros_offboard_controller_2',
        parameters= []
    )
    
    rc_control_mavros_node = Node(
        package='px4_offboard',
        namespace='px4_offboard',
        executable='rc_control_mavros',
        name='mavros_offboard_controller_3',
        parameters= []
    )

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    #ld.add_action(visualizer_node),
    #ld.add_action(pose_control_dds_node),
    #ld.add_action(velocity_control_dds_node),
    ld.add_action(rc_control_dds_node),
    ld.add_action(vio_to_px4_dds_node),
    #ld.add_action(pose_control_mavros_node),
    #ld.add_action(velocity_control_mavros_node),
    #ld.add_action(rc_control_mavros_node),
    
    return ld
