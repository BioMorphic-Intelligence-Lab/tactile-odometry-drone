"""Launch a Gazebo simulation spawning a PX4 drone communicating over ROS2."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """Launch Gazebo with a drone running PX4 communicating over ROS 2."""
    HOME = os.environ.get('HOME')
    PX4_RUN_DIR = HOME + '/Desktop/PX4-Autopilot'

    os.makedirs(PX4_RUN_DIR, exist_ok=True)
    
    # GZ - ROS Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock (GZ -> ROS2)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Joint states (GZ -> ROS2)
            #'/world/cyberzoo/model/x500withArms_0/model/dummy_arm0/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            # Base Pose Ground Truth (GZ->ROS2)
            '/world/cyberzoo/model/x500_0/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose',
           ],
        remappings=[
            ('/world/cyberzoo/model/x500_0/pose','/MocapPose'),
            #('/world/cyberzoo/model/x500withArms_0/model/dummy_arm0/joint_state','/squid/arm0/joint_state'),
        ],
        output='screen'
    )

    # Visual Odometry Forwarder
    mocap_forwarder = Node(
        package='simulation',
        executable='mocap_forwarder'
    )
    
    # Publisher for fake jointstate data
    fake_jointstate_publisher = Node(
        package='simulation',
        executable='fake_jointstate_publisher'
    )

    return LaunchDescription([
        # Set required environment variables
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH',
                               HOME + '/Desktop/tactile-odom-drone/offboard/build/simulation/'),
        SetEnvironmentVariable('PX4_GZ_MODEL',
                               'x500odometry'),
        SetEnvironmentVariable('PX4_GZ_WORLD',
                               'cyberzoo'),
        SetEnvironmentVariable('PX4_GZ_MODEL_POSE',
                               '0,0,0.75,0,0,1.5707963267948966'),
        
        # Launch MicroXRCEAgent to communicate between PX4 and ROS2
        ExecuteProcess(
            cmd=[
                'MicroXRCEAgent', 'udp4', '-p', '8888'
            ],
            prefix="bash -c 'sleep 5s; $0 $@'",
           output='screen'),
       
        # Launch PX4 GZ Sim
        ExecuteProcess(
            cmd=[
                PX4_RUN_DIR + '/build/px4_sitl_default/bin/px4',
                ]),

        # Launch GZ-ROS2 Bridge
        bridge,
        # Mocap Forwarder
        mocap_forwarder,
        # Fake JointState Publisher
        fake_jointstate_publisher
    ])