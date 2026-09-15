# View the Nori URDF in RViz with joint sliders (no physics, no controllers).
#   ros2 launch nori_description view.launch.py
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory("nori_description")
    xacro_file = os.path.join(pkg, "urdf", "nori.urdf.xacro")
    robot_desc = ParameterValue(
        Command(["xacro ", xacro_file, " use_mock_hardware:=true"]), value_type=str)

    return LaunchDescription([
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_desc}]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui"),
        Node(package="rviz2", executable="rviz2", output="screen"),
    ])
