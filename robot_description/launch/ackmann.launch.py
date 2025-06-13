import os, xacro
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

robot_model = 'donkey_sensors'
robot_ns = 'r1' # Robot namespace (robot name)
pose = ['0.0', '-2.0', '5.0', '0.0'] #Initial robot pose: x,y,z,th
robot_base_color = '0.0 0.0 1.0 0.95' #Ign and Rviz color of the robot's main body (rgba)


def generate_launch_description():


    pkg = os.path.join(get_package_share_directory('robot_description'))
    #world_file = os.path.join(pkg, 'models', 'warehouse', 'model.sdf')
    world_file = os.path.join(pkg, 'worlds', 'warehouse.sdf')
    # Set Gazebo model path
    model_path = os.path.join(pkg, 'models')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = f"{model_path}:{os.environ['GAZEBO_MODEL_PATH']}"
    else:
        os.environ['GAZEBO_MODEL_PATH'] = model_path
    
    simu_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')
    
    # Set ign sim resource path
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(pkg, 'worlds'), ':' + str(Path(pkg).parent.resolve())
        ]
    )

    open_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', str(pkg+"/rviz/ns_robot.rviz")],
    )

    gz_sim = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
            launch_arguments=[
                ('gz_args', [world_file, ' -v 4', ' -r'])

        ]
    )
    
    xacro_file = os.path.join(pkg, 'urdf', robot_model+'.urdf') #.urdf
    robot_desc = xacro.process_file(xacro_file).toxml()
    
    doc = xacro.process_file(xacro_file, 
                             mappings={'base_color' : robot_base_color, 'ns' : robot_ns})
    
    robot_desc = doc.toprettyxml(indent='  ') 
    
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
                   #'-string', robot_desc,
                   '-topic', robot_ns+'/robot_description',
                   '-x', pose[0], '-y', pose[1], '-z', pose[2],
                   '-R', '0.0', '-P', '0.0', '-Y', pose[3],
                   '-name', robot_ns,
                   '-allow_renaming', 'false'],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=robot_ns,
        output="screen",
        parameters=[{'robot_description': robot_desc}]
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[             # ign topic -t <topic_name> --info
            '/model/'+robot_ns+'/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/'+robot_ns+'/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/world/world_model/model/'+robot_ns+'/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model',

            # '/'+robot_ns+'/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/'+robot_ns+'/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            
            '/'+robot_ns+'/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/'+robot_ns+'/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/'+robot_ns+'/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/'+robot_ns+'/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/'+robot_ns+'/depth_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/'+robot_ns+'/depth_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/model/'+robot_ns+'/battery/linear_battery/state@sensor_msgs/msg/BatteryState@gz.msgs.BatteryState',
            '/'+robot_ns+'/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
        ],
        parameters=[{
            #'qos_overrides./model/'+robot_ns+'.subscriber.reliability': 'reliable'
        }],
        output='screen',
        remappings=[            # ign topic -l
            ('/model/'+robot_ns+'/cmd_vel', '/'+robot_ns+'/cmd_vel'),
            ('/model/'+robot_ns+'/odometry', '/'+robot_ns+'/odom'),
            ('/world/world_model/model/'+robot_ns+'/joint_state', '/'+robot_ns+'/joint_states'),
            # ('/'+robot_ns+'/imu', '/'+robot_ns+'/imu'),
            ('/'+robot_ns+'/lidar', '/'+robot_ns+'/scan'), 
            ('/model/'+robot_ns+'/battery/linear_battery/state', '/'+robot_ns+'/battery/state'),
            ('/'+robot_ns+'/tf', '/tf'), 
        ]
    ) 

    return LaunchDescription(
        [
            simu_time,
            gz_resource_path,
            open_rviz,
            robot_state_publisher,
            gz_spawn_entity,
            gz_sim,
            bridge
        ]
    )