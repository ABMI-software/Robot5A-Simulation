"""
@file visual_sim.launch.py
@brief Launch file for setting up the robot simulation with direct joint control.

This launch file initializes the robot simulation in Gazebo with a custom world that includes a spotlight,
spawns the robot entity, sets up the MoveIt configuration, and starts the necessary nodes and controllers
for the robot operation. Visual joint state publisher has been removed as we now use direct joint angle control.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
    DeclareLaunchArgument,
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
    @brief Generates the launch description for the robot simulation with direct joint control.

    This function sets up the robot description, launches Gazebo with a custom world file,
    spawns the robot entity, configures MoveIt, and starts the necessary nodes and controllers.
    The visual joint state publisher has been removed as we now control joints directly.

    @return LaunchDescription object containing all the nodes and configurations to launch.
    """
    # Force un timestamp unique pour éviter le cache
    import time
    timestamp = str(int(time.time()))
    
    # Declare the 'num_cameras' launch argument
    num_cameras_arg = DeclareLaunchArgument(
        'num_cameras',
        default_value='1',
        description='Number of cameras (1 or 2)'
    )

    # Launch configuration to access 'num_cameras' argument
    num_cameras = LaunchConfiguration('num_cameras')

    # Package Directories
    pkg_name = "robot_description"
    robot_moveit_config = "robot_moveit_config"
    share_dir = get_package_share_directory(pkg_name)
    moveit_config_pkg_path = get_package_share_directory(robot_moveit_config)

    # Load and process URDF/XACRO file
    xacro_file = os.path.join(share_dir, "urdf", "r5a_v_ros.urdf.xacro")
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {"robot_description": robot_description_config.toxml()}

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    # Gazebo Launch
    world_file_path = os.path.join(share_dir, "worlds", "spotlight.world")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gazebo.launch.py",
            )
        ]),
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
            publish_robot_description=True, 
            publish_robot_description_semantic=True
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

    load_gripper_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "gripper_controller",
        ],
        output="screen",
    )

    # Launch the GUI Node
    gui_node = Node(
        package="robot_control",
        executable="moveit_control_gui",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
    )

    # ArUco Detector Single Node (optionnel - pour visualisation uniquement)
    aruco_detector_single_node = Node(
        package="robot_control",
        executable="aruco_detector_single",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=UnlessCondition(PythonExpression(['"', num_cameras, '" == "2"']))
    )

    # ArUco Detector Double Node (optionnel - pour visualisation uniquement)
    aruco_detector_double_node = Node(
        package="robot_control",
        executable="aruco_detector_double",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(PythonExpression(['"', num_cameras, '" == "2"']))
    )

    # SUPPRIMÉ: visual_joint_state_publisher_node n'est plus nécessaire
    # Le contrôle se fait maintenant directement par angles de joints
    
    # SUPPRIMÉ: joint_state_publisher standard pour éviter les conflits
    # Gazebo fournit déjà les joint states via joint_state_broadcaster

    # Return the LaunchDescription
    return LaunchDescription([
        num_cameras_arg,
        
        # Paramètre global pour le temps de simulation
        SetParameter(name="use_sim_time", value=True),
        
        # Séquence de lancement des composants principaux
        gazebo,
        robot_state_publisher_node,
        spawn_entity,
        
        # Chargement séquentiel des contrôleurs
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
                on_exit=[load_gripper_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_gripper_controller,
                on_exit=[move_group_node],
            )
        ),
        
        # Démarrage des nœuds applicatifs après MoveIt
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=move_group_node,
                on_start=[
                    gui_node,
                    aruco_detector_single_node,  # Optionnel - seulement pour visualisation
                    aruco_detector_double_node,  # Optionnel - seulement pour visualisation
                ],
            )
        ),
    ])