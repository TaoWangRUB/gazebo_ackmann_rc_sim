#!/usr/bin/env python

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


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
        parameters= []
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
    
    rviz2_node = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', [os.path.join(package_dir, 'visualize.rviz')]]
    )
    
    return LaunchDescription([
        #visualizer_node,
        #pose_control_dds_node,
        #velocity_control_dds_node,
        rc_control_dds_node,
        vio_to_px4_dds_node,
        #pose_control_mavros_node,
        #velocity_control_mavros_node,
        #rc_control_mavros_node,
        #rviz2_node
    ])
