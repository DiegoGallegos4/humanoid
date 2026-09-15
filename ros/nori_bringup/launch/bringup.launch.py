# Nori ros2_control bring-up.
#   ros2 launch nori_bringup bringup.launch.py                 # MuJoCo-backed
#   ros2 launch nori_bringup bringup.launch.py use_mock_hardware:=true  # no physics
#   ... rviz:=true   to also open RViz
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_mock = LaunchConfiguration("use_mock_hardware")
    use_rviz = LaunchConfiguration("rviz")

    desc_pkg = get_package_share_directory("nori_description")
    mjcf_path = os.path.join(
        os.path.dirname(desc_pkg), "..", "..", "..", "simulation", "mujoco", "scene.xml")
    # allow override; default resolves the repo scene.xml at runtime
    xacro_file = os.path.join(desc_pkg, "urdf", "nori.urdf.xacro")

    robot_description = ParameterValue(
        Command(["xacro ", xacro_file,
                 " use_mock_hardware:=", use_mock,
                 " mjcf_path:=", LaunchConfiguration("mjcf_path")]),
        value_type=str)

    controllers = PathJoinSubstitution(
        [FindPackageShare("nori_bringup"), "config", "nori_controllers.yaml"])

    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               parameters=[{"robot_description": robot_description}])

    cm = Node(package="controller_manager", executable="ros2_control_node",
              parameters=[{"robot_description": robot_description}, controllers],
              output="screen")

    def spawner(name):
        return Node(package="controller_manager", executable="spawner",
                    arguments=[name, "--controller-manager", "/controller_manager"])

    jsb = spawner("joint_state_broadcaster")
    body = spawner("body_controller")
    wheels = spawner("diff_drive_controller")

    rviz = Node(package="rviz2", executable="rviz2", condition=IfCondition(use_rviz),
                output="screen")

    # spawners self-wait for /controller_manager; a small delay avoids racing the
    # node's interface export. Broadcaster first, then the controllers.
    return LaunchDescription([
        DeclareLaunchArgument("use_mock_hardware", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("mjcf_path", default_value=mjcf_path),
        rsp, cm, rviz,
        TimerAction(period=3.0, actions=[jsb]),
        TimerAction(period=5.0, actions=[body, wheels]),
    ])
