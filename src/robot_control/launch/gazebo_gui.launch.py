import os
import xacro
import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetLaunchConfiguration,
    IncludeLaunchDescription,
    RegisterEventHandler,
    LogInfo,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_pkg_name = "robot_moveit_config"
    control_pkg_name = "robot_control"

    moveit_config_pkg_path = get_package_share_directory(moveit_pkg_name)
    control_config_path = get_package_share_directory(control_pkg_name)

    xacro_file = "/home/rothvichea/ros2_jazzy/src/robot_description/urdf/r5a_v_ros.urdf.xacro"
    sdf_path = "/home/rothvichea/ros2_jazzy/src/robot_description/urdf/robot.sdf"
    controllers_yaml = os.path.join(control_config_path, 'config', 'controller.yaml')
    joint_limits_path = os.path.join(moveit_config_pkg_path, "config", "joint_limits.yaml")

    print("📄 Controller YAML path:", controllers_yaml)
    print("📄 File exists?", os.path.exists(controllers_yaml))
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {"robot_description": robot_description_config.toxml()}

    try:
        with open(joint_limits_path, 'r') as f:
            joint_limits = yaml.safe_load(f)
        print("✅ Loaded joint_limits.yaml")
    except Exception as e:
        print(f"⚠️ Failed to load joint_limits.yaml: {e}")
        joint_limits = {}

    try:
        with open(controllers_yaml, 'r') as f:
            controller_params = yaml.safe_load(f)
        print("✅ Loaded controller.yaml")
    except Exception as e:
        print(f"⚠️ Failed to load controller.yaml: {e}")
        controller_params = {}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description["robot_description"]},
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-file", sdf_path, "-name", "armr5"],
        output="screen",
    )

    moveit_config = (
        MoveItConfigsBuilder(robot_name=moveit_pkg_name, package_name=moveit_pkg_name)
        .robot_description(file_path=xacro_file, mappings={"use_sim_time": "true"})
        .robot_description_semantic(os.path.join(moveit_config_pkg_path, "config", "armr5.srdf"))
        .robot_description_kinematics(os.path.join(moveit_config_pkg_path, "config", "kinematics.yaml"))
        .trajectory_execution(os.path.join(moveit_config_pkg_path, "config", "moveit_controllers.yaml"))
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    moveit_config_dict = moveit_config.to_dict()
    moveit_config_dict["robot_description_planning"] = joint_limits

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config_dict, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    gui_node = Node(
        package=control_pkg_name,
        executable="moveit_control_gui",
        output="screen",
        parameters=[
            moveit_config_dict,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"moveit_current_state_monitor.joint_state_qos": "sensor_data"},
        ],
    )

    load_joint_state_controller = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_joint_state_broadcaster",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--param-file", controllers_yaml,
        ],
    )

    load_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_arm_controller",
        output="screen",
        arguments=[
            "arm_controller",
            "--controller-manager", "/controller_manager",
            "--param-file", controllers_yaml,
        ],
    )

    load_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_gripper_controller",
        output="screen",
        arguments=[
            "gripper_controller",
            "--controller-manager", "/controller_manager",
            "--param-file", controllers_yaml,
        ],
    )

    log_joint_state = LogInfo(msg="🛠️ Spawning joint_state_broadcaster...")
    log_arm_controller = LogInfo(msg="🛠️ Spawning arm_controller...")
    log_gripper_controller = LogInfo(msg="🛠️ Spawning gripper_controller...")

    controller_event_sequence = [
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_entity,
                on_exit=[log_joint_state, load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[log_arm_controller, load_arm_controller],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=load_arm_controller,
                on_exit=[log_gripper_controller, load_gripper_controller],
            )
        ),
        TimerAction(
            period=6.0,
            actions=[
                LogInfo(msg="🌟 Launching MoveGroup and GUI..."),
                move_group_node,
                gui_node,
            ],
        ),
    ]

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        SetLaunchConfiguration("use_sim_time", "true"),
        robot_state_publisher_node,
        gazebo,
        spawn_entity,
        *controller_event_sequence,
    ])
