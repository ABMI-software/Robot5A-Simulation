"""
@file moveit_control_gui.launch.py
@brief Launch file for the GUI node controlling the robot.

This launch file initializes the MoveIt configurations and launches the GUI node
that controls the robot using the MoveIt planning framework.
"""

import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """
    @brief Generates the launch description for the GUI node.

    This function loads the MoveIt configurations and initializes the GUI node
    that interfaces with the robot control package.

    @return LaunchDescription object containing the GUI node.
    """

    # Specify the name of the package and path to xacro file within the package
    pkg_name = "robot_description"  # Name of the robot description package
    share_dir = get_package_share_directory(
        pkg_name
    )  # Get the share directory of the package

    # Use xacro to process the file
    xacro_file = os.path.join(
        share_dir, "urdf", "r5a_v_ros.urdf.xacro"
    )  # Full path to the XACRO file

    # MoveIt configuration using MoveItConfigsBuilder
    moveit_config = (
        MoveItConfigsBuilder("robot_moveit_config", package_name="robot_moveit_config")
        .robot_description(
            file_path=xacro_file, mappings={"use_sim_time": "true"}
        )
        .robot_description_semantic("config/armr5.srdf")
        .robot_description_kinematics("config/kinematics.yaml")
        .joint_limits("config/joint_limits.yaml")
        .trajectory_execution("config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    config_dict = moveit_config.to_dict()

    # Launch the new C++ GUI node that controls the robot
    gui_node = Node(
        package="robot_control",  # Package name containing the GUI node
        executable="moveit_control_gui",  # Executable name of the GUI node
        output="screen",
        parameters=[
            config_dict,
            {"use_sim_time": True},
        ],  # Pass MoveIt config to the GUI node
    )

    return LaunchDescription([gui_node])
