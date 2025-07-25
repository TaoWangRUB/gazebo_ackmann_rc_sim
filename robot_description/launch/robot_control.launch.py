from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import LaunchConfigurationNotEquals
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
]


def generate_launch_description():
    pkg_control = get_package_share_directory('robot_description')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    control_params_file = PathJoinSubstitution(
        [pkg_control, 'config', 'ackermann_controller.yaml'])
    
    # Controller manager (ros2_control_node)
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=namespace,
        parameters=[{
            '~/robot_description': '/robot_description',
            'use_sim_time': use_sim_time
            }, control_params_file],
        output='screen',
    )

    # Joint state broadcaster
    # This node is responsible for publishing the joint states of the robot
    # It is a part of the ros2_control framework
    # and is used to publish the joint states of the robot to the /joint_states topic
    # It is required for the robot to be able to move
    # and to be able to visualize the robot in RViz
    joint_state_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )
    
    # Ackermann steering controller
    # This controller is used to control the steering of the robot
    # It is a part of the ros2_control framework
    # and is responsible for controlling the steering angle of the front wheels
    # It is a required controller for the ackermann drive system
    # It is used to control the steering angle of the front wheels
    ackermann_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ackermann_steering_controller', 
                   '--param-file',
                   control_params_file],
        output='screen',
        remappings=[
            # Remap the ackermann controller's input topic to cmd_vel
            ('/reference_unstamped', '/cmd_vel'),
        ],
    )

    # Ensure diffdrive_controller_node starts after joint_state_broadcaster_spawner
    ackmann_controller_callback = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_controller_node,
            on_exit=[ackermann_controller_node],
        )
    )

    # Static transform from <namespace>/odom to odom
    # See https://github.com/ros-controls/ros2_controllers/pull/533
    tf_namespaced_odom_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_namespaced_odom_publisher',
        arguments=['0', '0', '0',
                   '0', '0', '0',
                   'odom', [namespace, '/odom']],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ],
        output='screen',
        condition=LaunchConfigurationNotEquals('namespace', '')
    )

    # Static transform from <namespace>/base_link to base_link
    tf_namespaced_base_link_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_namespaced_base_link_publisher',
        arguments=['0', '0', '0',
                   '0', '0', '0',
                   [namespace, '/base_link'], 'base_link'],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ],
        output='screen',
        condition=LaunchConfigurationNotEquals('namespace', '')
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(ackmann_controller_callback)
    # ld.add_action(controller_manager)
    ld.add_action(joint_state_controller_node)
    ld.add_action(tf_namespaced_odom_publisher)
    ld.add_action(tf_namespaced_base_link_publisher)

    return ld