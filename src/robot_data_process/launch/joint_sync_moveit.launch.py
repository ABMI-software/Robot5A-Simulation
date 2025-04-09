"""
@file joint_sync_moveit.launch.py
@brief Launch file for setting up the robot simulation with visual servoing components and joint synchronization node.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
    DeclareLaunchArgument,
    TimerAction,
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
    @brief Generates the launch description for the robot simulation with visual servoing and joint sync components.

    This function sets up the robot description, launches Gazebo, spawns the robot, configures MoveIt, and starts
    the joint synchronization node with a delay to ensure all dependencies are ready.
    
    @return LaunchDescription object containing all the nodes and configurations to launch.
    """

    # Declare the 'num_cameras' launch argument
    num_cameras_arg = DeclareLaunchArgument(
        'num_cameras',
        default_value='1',
        description='Number of cameras (1 or 2)'
    )
    num_cameras = LaunchConfiguration('num_cameras')

    # Package Directories
    pkg_name = "robot_description"
    robot_moveit_config = "robot_moveit_config"
    share_dir = get_package_share_directory(pkg_name)
    moveit_config_pkg_path = get_package_share_directory(robot_moveit_config)

    # Load and process URDF/XACRO file with dynamic camera configuration
    xacro_file = os.path.join(share_dir, "urdf", "r5a_v_ros.urdf.xacro")
    robot_description_config = xacro.process_file(
        xacro_file,
        mappings={'num_cameras': num_cameras}
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    # Gazebo launch with a custom world file
    world_file_path = os.path.join(share_dir, "worlds", "spotlight.world")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")]
        ),
        launch_arguments={"use_sim_time": "true", "world": world_file_path}.items(),
    )

    # Spawn Entity Node
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "/robot_description", "-entity", "armr5"],
        output="screen",
    )

    # MoveIt Configuration
    moveit_config = (
        MoveItConfigsBuilder(robot_moveit_config, package_name=robot_moveit_config)
        .robot_description(file_path=xacro_file, mappings={"use_sim_time": "true", "num_cameras": num_cameras})
        .robot_description_semantic(os.path.join(moveit_config_pkg_path, "config", "armr5.srdf"))
        .robot_description_kinematics(os.path.join(moveit_config_pkg_path, "config", "kinematics.yaml"))
        .trajectory_execution(os.path.join(moveit_config_pkg_path, "config", "moveit_controllers.yaml"))
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
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
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "joint_state_broadcaster"],
        output="screen",
    )

    load_arm_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "arm_controller"],
        output="screen",
    )

    load_gripper_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "gripper_controller"],
        output="screen",
    )

    # Visual Joint State Publisher Node
    visual_joint_state_publisher_node = Node(
        package="robot_control",
        executable="visual_joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # Joint State Bridge Node
    joint_state_bridge_node = Node(
        package="robot_control",
        executable="joint_state_bridge",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # ArUco Detector Single Node
    aruco_detector_single_node = Node(
        package="robot_control",
        executable="aruco_detector_single",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=UnlessCondition(PythonExpression(['"', num_cameras, '" == "2"']))
    )

    # ArUco Detector Double Node
    aruco_detector_double_node = Node(
        package="robot_control",
        executable="aruco_detector_double",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(PythonExpression(['"', num_cameras, '" == "2"']))
    )

    # Joint Sync MoveIt Node (with a 15-second delay)
    joint_sync_moveit_node = Node(
        package="robot_data_process",
        executable="joint_sync_moveit_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # Launch Description
    return LaunchDescription([
        num_cameras_arg,
        SetParameter(name="use_sim_time", value=True),
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
                on_exit=[load_gripper_controller, move_group_node],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=move_group_node,
                on_start=[
                    aruco_detector_single_node,
                    aruco_detector_double_node,
                    visual_joint_state_publisher_node,
                    joint_state_bridge_node,
                    TimerAction(
                        period=15.0,  # 15-second delay
                        actions=[joint_sync_moveit_node],
                    ),
                ],
            )
        ),
        gazebo,
        robot_state_publisher_node,
        spawn_entity,
    ])

if __name__ == "__main__":
    generate_launch_description()