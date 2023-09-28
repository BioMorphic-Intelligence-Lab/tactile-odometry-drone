import os
import sys
from ament_index_python.packages import get_package_share_directory

from launch.substitutions import TextSubstitution
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node

def generate_launch_description():
    
    pkg_name = "planners"
    ld = LaunchDescription()

    start_base_ref_publisher = Node(package = pkg_name,
                                    name = "ref_pos_repeater",
                                    executable = "ref_pos_repeater")

    # Get command line argument for which planner to start
    planner = "rectangle"
    for arg in sys.argv:
        if arg.startswith("planner:="):
            planner = str(arg.split(":=")[1])
    
    planner_launch_action = None

    match planner:
        case "rectangle":
            planner_launch_action = Node(package = pkg_name,
                                         name = "rectangle_planner",
                                         executable = "rectangle_planner",
                                         parameters = [os.path.join(
                                             get_package_share_directory(pkg_name),
                                             "config","rectangle_params.yaml")]
                                    )
        case "line":
            planner_launch_action = Node(package = pkg_name,
                                         name = "line_planner",
                                         executable = "line_planner",
                                         parameters = [os.path.join(
                                             get_package_share_directory(pkg_name),
                                             "config","line_params.yaml")]
                                    )
        case "circle":
            planner_launch_action = Node(package = pkg_name,
                                         name = "circle_planner",
                                         executable = "circle_planner",
                                         parameters = [os.path.join(
                                             get_package_share_directory(pkg_name),
                                             "config","circle_params.yaml")]
                                    )
        case "sin":
            planner_launch_action = Node(package = pkg_name,
                                         name = "sin_planner",
                                         executable = "sin_planner",
                                         parameters = [os.path.join(
                                             get_package_share_directory(pkg_name),
                                             "config","sin_params.yaml")]
                                    )
        case "zero":
            planner_launch_action = Node(package = pkg_name,
                                         name = "zero_planner",
                                         executable = "zero_planner"
            )
        case _:
            sys.exit(f"Error: This planner ({planner}) does not exist. Try again")
    
    
    ld.add_action(start_base_ref_publisher)
    ld.add_action(planner_launch_action)


    return ld