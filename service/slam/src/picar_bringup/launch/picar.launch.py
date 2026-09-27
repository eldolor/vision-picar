"""The whole ROS side, in one launch -- PLAN-ros-alignment.md R4.

    twist_mux -> diff_drive_controller -> picar_sim_hardware -> HTTP -> robot/server.py
    robot_state_publisher (URDF, R3) . joint_state_broadcaster . picar_bridge
    slam_toolbox (R5): /scan + odom -> /map and map -> odom
    nav2 (R6): planner, controller, behaviours, bt_navigator -> cmd_vel/nav
    collision_monitor (R6): twist_mux -> /cmd_vel_mux -> collision_monitor
                            -> diff_drive_controller  (the collars in series)
    foxglove_bridge (2026-09-27): a READ-ONLY window for Foxglove on :8765 --
        every topic, including the brain's (/brain/status, /diagnostics,
        /brain/markers), and no way to publish, call services or set params
"""
import os

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_url = os.environ.get("ROBOT_URL", "http://host.docker.internal:8000")
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("picar_description"), "urdf", "picar.urdf.xacro"])
    description = ParameterValue(
        Command(["xacro ", xacro_file, " robot_url:=", robot_url]), value_type=str)
    controllers = PathJoinSubstitution(
        [FindPackageShare("picar_bringup"), "config", "controllers.yaml"])
    mux = PathJoinSubstitution(
        [FindPackageShare("picar_bringup"), "config", "twist_mux.yaml"])
    slam = PathJoinSubstitution(
        [FindPackageShare("picar_bringup"), "config", "slam.yaml"])
    nav2 = PathJoinSubstitution(
        [FindPackageShare("picar_bringup"), "config", "nav2.yaml"])
    # nav2's controller and behaviours publish cmd_vel; theirs is the `nav`
    # input of twist_mux, below a person's.
    to_nav = [("cmd_vel", "cmd_vel/nav")]
    nav_nodes = ["controller_server", "planner_server", "behavior_server", "bt_navigator"]

    return LaunchDescription([
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": description}]),
        Node(package="controller_manager", executable="ros2_control_node",
             parameters=[{"robot_description": description}, controllers],
             output="both"),
        Node(package="controller_manager", executable="spawner",
             arguments=["joint_state_broadcaster"]),
        Node(package="controller_manager", executable="spawner",
             arguments=["diff_drive_controller"]),
        Node(package="twist_mux", executable="twist_mux", parameters=[mux],
             # R6: into collision_monitor, which feeds diff_drive_controller.
             remappings=[("cmd_vel_out", "/cmd_vel_mux")]),
        Node(package="picar_bridge", executable="bridge", output="both"),
        # R5: the map, and the robot's place on it.
        Node(package="slam_toolbox", executable="async_slam_toolbox_node",
             name="slam_toolbox", parameters=[slam], output="both"),
        # R6: nav2, and collision_monitor between the mux and the base.
        Node(package="nav2_controller", executable="controller_server",
             parameters=[nav2], remappings=to_nav, output="both"),
        Node(package="nav2_planner", executable="planner_server",
             parameters=[nav2], output="both"),
        Node(package="nav2_behaviors", executable="behavior_server",
             parameters=[nav2], remappings=to_nav, output="both"),
        Node(package="nav2_bt_navigator", executable="bt_navigator",
             parameters=[nav2], output="both"),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation",
             parameters=[{"autostart": True, "node_names": nav_nodes}]),
        Node(package="nav2_collision_monitor", executable="collision_monitor",
             parameters=[nav2], output="both"),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_collision",
             parameters=[{"autostart": True, "node_names": ["collision_monitor"]}]),
        # Read-only on purpose: foxglove_bridge's default capabilities let a
        # client PUBLISH, call services and set parameters -- a second door
        # into the wheels that skips robot/server.py's arbitration. Only the
        # connection graph is offered, so it can look and never touch.
        Node(package="foxglove_bridge", executable="foxglove_bridge",
             parameters=[{"port": 8765, "address": "0.0.0.0",
                          "capabilities": ["connectionGraph"]}],
             output="log"),
    ])
