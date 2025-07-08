"""
@file gazebo.launch.py
@brief Launch file for setting up the robot simulation in Gazebo with custom materials and lighting.

This launch file initializes the robot simulation in Gazebo with custom spotlight world,
custom materials, spawns the robot entity, and starts the necessary nodes and controllers.
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """
    @brief Generates the launch description for the robot simulation.

    This function sets up the robot description, launches Gazebo with custom spotlight world,
    loads custom materials, spawns the robot entity, and starts the necessary controllers.

    @return LaunchDescription object containing all the nodes and configurations to launch.
    """

    # Package configuration
    pkg_name = "robot_description"
    share_dir = get_package_share_directory(pkg_name)

    print("🚀 Initializing Robot Simulation with Custom Materials and Lighting")
    print("=" * 70)

    # Configure paths for custom materials
    materials_path = os.path.join(share_dir, "materials")
    world_file = os.path.join(share_dir, "worlds", "spotlight.world")
    xacro_file = os.path.join(share_dir, "urdf", "r5a_v_ros.urdf.xacro")

    # Check materials
    if os.path.exists(materials_path):
        print(f"✅ Custom materials found: {materials_path}")
        if os.path.exists(os.path.join(materials_path, "scripts", "robot_materials_PLA.material")):
            print("✅ PLA materials file found")
        else:
            print("⚠️  robot_materials_PLA.material not found in scripts/")
    else:
        print(f"⚠️  Materials directory not found: {materials_path}")
        print("   Robot will use default Gazebo materials")

    # Check world file
    if os.path.exists(world_file):
        print(f"✅ Custom spotlight world found: {world_file}")
        use_custom_world = True
    else:
        print(f"⚠️  Spotlight world not found: {world_file}")
        print("   Using default Gazebo world")
        use_custom_world = False

    # Check URDF file
    if os.path.exists(xacro_file):
        print(f"✅ Robot URDF found: {xacro_file}")
    else:
        print(f"❌ Robot URDF not found: {xacro_file}")

    print("=" * 70)

    # Process robot description
    robot_description_xacro = xacro.process_file(xacro_file)
    robot_urdf = robot_description_xacro.toxml()

    # Environment variables for custom materials
    set_gazebo_resource_path = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value=[
            materials_path,
            ':',
            os.environ.get('GAZEBO_RESOURCE_PATH', '')
        ]
    )

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[
            share_dir,
            ':',
            os.environ.get('GAZEBO_MODEL_PATH', '')
        ]
    )

    # Robot state publisher node
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_urdf},
            {"use_sim_time": True},
        ],
    )

    # Gazebo launch configuration
    gazebo_launch_args = {
        'verbose': 'true',
        'pause': 'false',
        'use_sim_time': 'true',
    }

    # Add custom world if available
    if use_custom_world:
        gazebo_launch_args['world'] = world_file
        print("🌟 Gazebo will launch with custom spotlight world for enhanced reflections")

    # Gazebo server and client launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("gazebo_ros"), "launch"),
            "/gazebo.launch.py",
        ]),
        launch_arguments=gazebo_launch_args.items()
    )

    # Robot spawn node
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_entity",
        arguments=[
            "-topic", "/robot_description",
            "-entity", "armr5",
            "-x", "0.0",
            "-y", "0.0", 
            "-z", "0.1",
        ],
        output="screen",
    )

    # Joint state broadcaster controller
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2", "control", "load_controller",
            "--set-state", "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    # Arm controller
    load_arm_controller = ExecuteProcess(
        cmd=[
            "ros2", "control", "load_controller", 
            "--set-state", "active",
            "arm_controller",
        ],
        output="screen",
    )

    # Joint state publisher (for manual control if needed)
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{"use_sim_time": True}],
        condition=lambda: not os.path.exists("/tmp/gazebo_controllers_active")
    )

    # Startup info node
    info_node = ExecuteProcess(
        cmd=[
            "bash", "-c", 
            "sleep 5 && echo '✅ Robot simulation ready!' && "
            "echo '🎮 Test robot movement:' && "
            "echo 'ros2 topic pub /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \"{joint_names: [R0_Yaw, R1_Pitch, R2_Pitch, R3_Yaw, R4_Pitch], points: [{positions: [0.5, 0.5, 0.5, 0.5, 0.5], time_from_start: {sec: 3}}]}\" --once'"
        ],
        output="screen",
    )

    # Launch description with proper sequencing
    return LaunchDescription([
        # Environment setup
        set_gazebo_resource_path,
        set_gazebo_model_path,
        
        # Core simulation
        gazebo,
        node_robot_state_publisher,
        joint_state_publisher,
        
        # Sequential robot spawning and controller loading
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gazebo,
                on_exit=[
                    ExecuteProcess(cmd=["sleep", "3"], output="screen"),  # Wait for Gazebo
                ]
            )
        ),
        
        spawn_entity,
        
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_arm_controller],
            )
        ),
        
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_arm_controller,
                on_exit=[info_node],
            )
        ),
    ])