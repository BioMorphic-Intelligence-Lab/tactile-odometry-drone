
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
            parameters=[
                {"trackball_name": "X19"},
                {"trackball_radius": 0.019},
                {"trackball_resolution": 425.0}
            ],
            name="trackball_interface",
            output="screen",
            emulate_tty=True            
        ),
         launch_ros.actions.Node(
            package="trackball_interface",
            executable="trackball_interface",
            parameters=[
                {"trackball_name": "X13"},
                {"trackball_radius": 0.013},
                {"trackball_resolution": 300.0}
            ],
            name="trackball_interface_2",
            output="screen",
            emulate_tty=True            
        )
    ])