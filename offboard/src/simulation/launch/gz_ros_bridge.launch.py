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
            '/world/cyberzoo/model/x500/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose',
           ],
        remappings=[
            #('/world/cyberzoo/model/x500withArms_0/model/dummy_arm0/joint_state','/squid/arm0/joint_state'),
            ('/world/cyberzoo/model/x500/pose','/MocapPose'),
        ],
        output='screen'
    )

    return LaunchDescription([
        # Launch GZ-ROS2 Bridge
        bridge
    ])