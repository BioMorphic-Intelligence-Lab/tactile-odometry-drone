
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
 
def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("imu_filter_madgwick"), '/launch', '/imu_filter.launch.py'])
            ),
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("phidgets_spatial"), '/launch', '/spatial-launch.py'])
    ),
         launch_ros.actions.Node(
            package="trackball_interface",
            executable="trackball_interface",
            name="trackball_interface",
            output="screen",
            emulate_tty=True            
        )
    ])