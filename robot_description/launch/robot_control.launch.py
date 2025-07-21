from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import LaunchConfigurationNotEquals
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
]


def generate_launch_description():
    pkg_create3_control = get_package_share_directory('robot_description')

    namespace = LaunchConfiguration('namespace')

    control_params_file = PathJoinSubstitution(
        [pkg_create3_control, 'config', 'ackermann_controller.yaml'])

    ackermann_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,  # Namespace is not pushed when used in EventHandler
        parameters=[control_params_file],
        arguments=['ackermann_steering_controller', '-c', 'controller_manager'],
        output='screen',
    )

    joint_state_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', 'controller_manager'],
        output='screen',
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

    ld.add_action(joint_state_controller_node)
    ld.add_action(ackmann_controller_callback)
    ld.add_action(tf_namespaced_odom_publisher)
    ld.add_action(tf_namespaced_base_link_publisher)

    return ld