"""The whole ROS side, in one launch -- PLAN-ros-alignment.md R4.

    twist_mux -> diff_drive_controller -> picar_sim_hardware -> HTTP -> robot/server.py
    robot_state_publisher (URDF, R3) . joint_state_broadcaster . picar_bridge
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
             remappings=[("cmd_vel_out", "/diff_drive_controller/cmd_vel_unstamped")]),
        Node(package="picar_bridge", executable="bridge", output="both"),
    ])
