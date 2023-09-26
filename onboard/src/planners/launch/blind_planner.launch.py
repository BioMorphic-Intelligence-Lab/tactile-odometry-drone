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
    planner = "line"
    for arg in sys.argv:
        if arg.startswith("planner:="):
            planner = str(arg.split(":=")[1])
    
    planner_launch_action = None

    match planner:
        case "line":
            planner_launch_action = Node(package = pkg_name,
                                         name = "blind_line",
                                         executable = "blind_line")
        case "sine":
            planner_launch_action = Node(package = pkg_name,
                                         name = "blind_sine",
                                         executable = "blind_sine")
        case "circle":
            planner_launch_action = Node(package = pkg_name,
                                         name = "blind_circle",
                                         executable = "blind_circle")
        case _:
            sys.exit(f"Error: This planner ({planner}) does not exist. Try again")
    
    
    ld.add_action(start_base_ref_publisher)
    ld.add_action(planner_launch_action)


    return ld