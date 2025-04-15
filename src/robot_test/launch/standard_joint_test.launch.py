#!/usr/bin/env python3
"""
@file standard_joint_test.launch.py
@brief Launch file for running standard joint tests with visual servoing components.

This launch file initializes the robot simulation in Gazebo with a custom world that includes a spotlight,
spawns the robot entity, sets up the MoveIt configuration, and starts the necessary nodes and controllers
for the robot operation, including visual servoing components like the ArUco detector and aruco error logger nodes.
It accepts a parameter 'num_cameras' to determine whether to launch the single or double ArUco detector.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
    DeclareLaunchArgument,
    Shutdown,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter
import xacro
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    """
    @brief Generates the launch description for running standard joint tests with visual servoing components.

    This function sets up the robot description, launches Gazebo with a custom world file,
    spawns the robot entity, configures MoveIt, and starts the necessary nodes and controllers,
    including the ArUco detector nodes and aruco error logger node.

    It accepts a parameter 'num_cameras' to determine whether to launch the single or double ArUco detector.

    @return LaunchDescription object containing all the nodes and configurations to launch.
    """

    # Declare the 'num_cameras' launch argument
    num_cameras_arg = DeclareLaunchArgument(
        'num_cameras',
        default_value='1',
        description='Number of cameras (1 or 2)'
    )

    # Launch configuration to access 'num_cameras' argument
    num_cameras = LaunchConfiguration('num_cameras')

    # Package Directories
    pkg_name = "robot_description"  # Name of the robot description package
    robot_moveit_config = "robot_moveit_config"  # Name of the MoveIt configuration package
    share_dir = get_package_share_directory(pkg_name)  # Path to the robot description package
    moveit_config_pkg_path = get_package_share_directory(robot_moveit_config)  # Path to the MoveIt config package

    # Load and process URDF/XACRO file
    xacro_file = os.path.join(share_dir, "urdf", "r5a_v_ros.urdf.xacro")  # Path to the XACRO file
    robot_description_config = xacro.process_file(xacro_file)  # Process the XACRO file
    robot_description = {"robot_description": robot_description_config.toxml()}  # Convert to XML format

    # Joint positions parameter (Define your joint positions here)
    joint_positions = [
        # Position 0: Home position
        0.0, 0.0, 0.0, 0.0, 0.0,

        # Position 1: Extend first joint (Base rotation)
        0.5, 0.0, 0.0, 0.0, 0.0,

        # Position 2: Extend second joint (Shoulder)
        0.0, 0.5, 0.0, 0.0, 0.0,

        # Position 3: Extend third joint (Elbow)
        0.0, 0.0, 0.5, 0.0, 0.0,

        # Position 4: Extend fourth joint (Wrist pitch)
        0.0, 0.0, 0.0, 0.5, 0.0,

        # Position 5: Extend fifth joint (Wrist roll)
        0.0, 0.0, 0.0, 0.0, 0.5,

        # Position 6: All joints at positive quarter range
        0.25, 0.25, 0.25, 0.25, 0.25,

        # Position 7: All joints at negative quarter range
        -0.25, -0.25, -0.25, -0.25, -0.25,

        # Position 8: Alternate joints positive and negative
        0.5, -0.5, 0.5, -0.5, 0.5,

        # Position 9: Arm Pointing towards Camera 1
        1.5, -1.5, 0.0, 0.0, 0.0,
    ]

    # Test type parameter
    test_type = "standard joint test"

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    # Gazebo launch with a custom world file that includes a spotlight
    world_file_path = os.path.join(share_dir, "worlds", "spotlight.world")  # Path to the custom world file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                )
            ]
        ),
        launch_arguments={"use_sim_time": "true", "world": world_file_path}.items(),
    )

    # Spawn Entity Node
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create.py",
        arguments=["-topic", "/robot_description", "-name", "armr5"],
        output="screen",
    )

    # MoveIt Configuration
    moveit_config = (
        MoveItConfigsBuilder(robot_moveit_config, package_name=robot_moveit_config)
        .robot_description(file_path=xacro_file, mappings={"use_sim_time": "true"})
        .robot_description_semantic(
            os.path.join(moveit_config_pkg_path, "config", "armr5.srdf")
        )
        .robot_description_kinematics(
            os.path.join(moveit_config_pkg_path, "config", "kinematics.yaml")
        )
        .trajectory_execution(
            os.path.join(moveit_config_pkg_path, "config", "moveit_controllers.yaml")
        )
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Move Group Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
    )

    # Load Controllers
    load_joint_state_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    load_arm_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "arm_controller",
        ],
        output="screen",
    )

    # Standard Joint Test Node
    standard_joint_test_node = Node(
        package="robot_test",
        executable="standard_joint_test_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
            {"moveit_current_state_monitor.joint_state_qos": "sensor_data"},
            {"joint_positions": joint_positions},
        ],
    )

    # Aruco Error Logger Node
    aruco_error_logger_node = Node(
        package="robot_test",
        executable="aruco_error_logger_node",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"joint_positions": joint_positions},
            {"test_type": test_type},
        ],
    )

    # Aruco Detector Single Node
    aruco_detector_single_node = Node(
        package="robot_control",
        executable="aruco_detector_single",
        output="screen",
        parameters=[
            {"use_sim_time": True},
        ],
        condition=UnlessCondition(PythonExpression(['"', num_cameras, '" == "2"']))
    )

    # Aruco Detector Double Node
    aruco_detector_double_node = Node(
        package="robot_control",
        executable="aruco_detector_double",
        output="screen",
        parameters=[
            {"use_sim_time": True},
        ],
        condition=IfCondition(PythonExpression(['"', num_cameras, '" == "2"']))
    )

    # Launch Actions
    actions = [
        num_cameras_arg,  # Added the launch argument
        SetParameter(name="use_sim_time", value=True),

        # Event handlers for controller loading
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_arm_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_arm_controller,
                on_exit=[move_group_node],
            )
        ),

        # Event handler to start other nodes after move_group_node starts
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=move_group_node,
                on_start=[
                    aruco_detector_single_node,
                    aruco_detector_double_node,
                    standard_joint_test_node,
                    aruco_error_logger_node,
                ],
            )
        ),

        # Event handler to shutdown when standard_joint_test_node exits
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=standard_joint_test_node,
                on_exit=[
                    Shutdown(reason='Standard joint test completed'),
                ],
            )
        ),

        # Nodes to be launched
        gazebo,
        robot_state_publisher_node,
        spawn_entity,
    ]

    return LaunchDescription(actions)
