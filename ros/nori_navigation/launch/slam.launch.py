# slam_toolbox async mapping for Nori (Stage 7A).
#   ros2 launch nori_navigation slam.launch.py
# Assumes the base is already up (ros2 launch nori_bringup bringup.launch.py),
# so /scan and the odom->base_link TF are being published. Drive the robot
# (ros/teleop.py) and a map builds on /map; slam_toolbox emits the map->odom TF.
#
# In Jazzy async_slam_toolbox_node is a LIFECYCLE node: it must be configured
# then activated before it advertises /map. We auto-drive those transitions here
# (autostart), mirroring slam_toolbox's own online_async_launch.py.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    params = PathJoinSubstitution(
        [FindPackageShare("nori_navigation"), "config", "slam_async.yaml"])
    autostart = LaunchConfiguration("autostart")

    slam = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        parameters=[params,
                    {"use_lifecycle_manager": False,
                     "use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    configure = EmitEvent(
        event=ChangeState(lifecycle_node_matcher=matches_action(slam),
                          transition_id=Transition.TRANSITION_CONFIGURE),
        condition=IfCondition(autostart),
    )
    activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam,
            start_state="configuring", goal_state="inactive",
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(slam),
                transition_id=Transition.TRANSITION_ACTIVATE))],
        ),
        condition=IfCondition(autostart),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        slam, configure, activate,
    ])
