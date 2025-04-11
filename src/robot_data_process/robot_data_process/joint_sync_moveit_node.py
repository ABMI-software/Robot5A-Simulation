import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
import csv
import os
import time
from ament_index_python.packages import get_package_share_directory
from message_filters import ApproximateTimeSynchronizer, Subscriber

class JointSyncMoveItNode(Node):
    def __init__(self):
        super().__init__('joint_sync_moveit_node')

        # Define joint names for arm and gripper (matches SRDF)
        self.arm_joint_names = ["R0_Yaw", "R1_Pitch", "R2_Pitch", "R3_Yaw", "R4_Pitch"]
        self.gripper_joint_names = ["ServoGear", "LeftGripper", "RightGripper", "LeftPivotArm", "RightPivotArm", "PassifGear"]
        self.all_joint_names = self.arm_joint_names + self.gripper_joint_names
        self.expected_command_length = 6  # 5 arm joints + 1 ServoGear
        self.commands = []
        self.current_command_index = 0
        self.is_executing_arm = False
        self.is_executing_gripper = False
        self.command_delay = 1.0  # Delay in seconds between commands
        self.last_command_time = None
        self.enable_logging = True  # Flag to control logging

        # Set CSV file path explicitly to workspace src directory
        self.csv_file_path = "/home/chipmunk-151/Robot5A/src/robot_data_process/data_analysis/logs/joint_sync_log.csv"
        self.get_logger().info(f"CSV file path set to: {self.csv_file_path}")

        # QoS profile
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)

        # Synchronized subscribers
        self.combined_sub = Subscriber(self, JointState, '/combined_joint_states', qos_profile=qos)
        self.true_sub = Subscriber(self, JointState, '/true_joint_states', qos_profile=qos)
        self.ts = ApproximateTimeSynchronizer([self.combined_sub, self.true_sub], queue_size=10, slop=0.05)
        self.ts.registerCallback(self.sync_callback)

        # Standalone subscribers for debugging
        self.combined_sub_standalone = self.create_subscription(
            JointState, '/combined_joint_states', self.combined_callback, qos)
        self.true_sub_standalone = self.create_subscription(
            JointState, '/true_joint_states', self.true_callback, qos)

        # Connect to MoveGroup action server
        possible_action_servers = ['/move_action', '/move_group', '/armr5/move_group']
        self.move_group_action_name = None
        max_attempts = 10
        timeout_per_attempt = 10.0

        self.get_logger().info("Searching for MoveGroup action server...")
        for action_name in possible_action_servers:
            self.get_logger().info(f"Checking action server: {action_name}")
            self.arm_move_group_client = ActionClient(self, MoveGroup, action_name)
            self.gripper_move_group_client = ActionClient(self, MoveGroup, action_name)
            
            for attempt in range(max_attempts):
                if self.arm_move_group_client.wait_for_server(timeout_sec=timeout_per_attempt):
                    self.get_logger().info(f"Connected to action server at {action_name} for arm")
                    self.move_group_action_name = action_name
                    break
                self.get_logger().warn(f"Attempt {attempt + 1}/{max_attempts}: {action_name} not available")
                time.sleep(2.0)
            else:
                self.get_logger().warn(f"Could not connect to {action_name} after {max_attempts} attempts")
                continue

            for attempt in range(max_attempts):
                if self.gripper_move_group_client.wait_for_server(timeout_sec=timeout_per_attempt):
                    self.get_logger().info(f"Connected to action server at {action_name} for gripper")
                    break
                self.get_logger().warn(f"Attempt {attempt + 1}/{max_attempts}: {action_name} not available for gripper")
                time.sleep(2.0)
            else:
                self.get_logger().warn(f"Gripper client could not connect to {action_name}")
                self.move_group_action_name = None
                continue

            if self.move_group_action_name:
                break

        if not self.move_group_action_name:
            raise RuntimeError(f"Failed to connect to any MoveGroup action server: {possible_action_servers}")

        # Initialize CSV and load commands
        self.initialize_csv()
        self.load_commands('config/joint_commands.txt')

        # Timers
        self.execution_timer = self.create_timer(4.0, self.start_command_execution)
        self.check_timer = self.create_timer(0.1, self.check_joint_states)

    def initialize_csv(self):
        os.makedirs(os.path.dirname(self.csv_file_path), exist_ok=True)
        with open(self.csv_file_path, mode='w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                'Command', 'Time (s)', 'True Joints', 'True Positions', 'True Velocities',
                'Combined Joints', 'Combined Positions', 'Combined Velocities',
                'Arm MoveIt Success', 'Gripper MoveIt Success'
            ])
        self.get_logger().info(f"Initialized CSV log at {self.csv_file_path}")

    def load_commands(self, filename):
        package_share_directory = get_package_share_directory('robot_data_process')
        full_path = os.path.join(package_share_directory, filename)
        self.get_logger().info(f"Attempting to load commands from: {full_path}")
        if os.path.isfile(full_path):
            with open(full_path, 'r') as file:
                self.commands = [list(map(float, line.strip().split())) for line in file.readlines()]
            self.get_logger().info(f"Loaded {len(self.commands)} commands from {full_path}")
        else:
            self.get_logger().error(f"Command file {full_path} not found")

    def sync_callback(self, combined_msg, true_msg):
        self.combined_joint_states = combined_msg
        self.true_joint_states = true_msg
        self.get_logger().debug("Received synchronized joint states")
        self.log_to_csv()

    def combined_callback(self, msg):
        self.get_logger().debug(f"Received /combined_joint_states: {msg.name}")

    def true_callback(self, msg):
        self.get_logger().debug(f"Received /true_joint_states: {msg.name}")

    def start_command_execution(self):
        if self.commands and self.current_command_index < len(self.commands):
            self.get_logger().info("Starting command execution")
            self.execute_command()
            self.execution_timer.cancel()
        else:
            self.get_logger().info("No commands to execute or all commands completed.")

    def execute_command(self):
        if self.current_command_index >= len(self.commands):
            self.get_logger().info("All commands executed.")
            self.enable_logging = False  # Stop logging
            return

        if not self.is_executing_arm and not self.is_executing_gripper:
            current_time = time.time()
            # Temporarily disable delay for debugging
            # if self.last_command_time and (current_time - self.last_command_time < self.command_delay):
            #     self.get_logger().debug(f"Waiting for delay: {current_time - self.last_command_time:.2f}/{self.command_delay}s")
            #     return

            command = self.commands[self.current_command_index]
            self.get_logger().info(f"Executing command {self.current_command_index + 1}/{len(self.commands)}: {command}")
            self.log_to_csv()

            if len(command) != self.expected_command_length:
                self.get_logger().error(f"Command {command} has {len(command)} values, expected {self.expected_command_length}")
                self.current_command_index += 1
                self.execute_command()
                return

            arm_command = command[:5]
            servo_gear = command[5]
            gripper_command = [servo_gear, -servo_gear, servo_gear, -servo_gear, -servo_gear, -servo_gear]

            self.execute_group_command(self.arm_move_group_client, "arm", self.arm_joint_names, arm_command)
            self.is_executing_arm = True
        else:
            self.get_logger().debug("Waiting for current execution to complete")

    def execute_group_command(self, client, group_name, joint_names, positions):
        goal = MoveGroup.Goal()
        goal.request.group_name = group_name
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.start_state.is_diff = True

        constraints = Constraints()
        for name, pos in zip(joint_names, positions):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = name
            joint_constraint.position = float(pos)
            joint_constraint.tolerance_above = 0.05
            joint_constraint.tolerance_below = 0.05
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)

        goal.request.goal_constraints = [constraints]

        self.get_logger().info(f"Sending MoveGroup goal for {group_name} with joints: {joint_names}, positions: {positions}")
        client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        ).add_done_callback(lambda future, gn=group_name: self.goal_response_callback(future, gn))

    def feedback_callback(self, feedback_msg):
        pass

    def goal_response_callback(self, future, group_name):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"{group_name} MoveGroup goal rejected")
            if group_name == "arm":
                self.is_executing_arm = False
            else:
                self.is_executing_gripper = False
            self.current_command_index += 1
            self.execute_command()
            return

        self.get_logger().info(f"{group_name} MoveGroup goal accepted")
        goal_handle.get_result_async().add_done_callback(lambda future, gn=group_name: self.result_callback(future, gn))

    def result_callback(self, future, group_name):
        result = future.result().result
        if result.error_code.val == 1:  # SUCCESS
            self.get_logger().info(f"{group_name} MoveIt execution succeeded")
            self.log_to_csv()
        else:
            self.get_logger().error(f"{group_name} MoveIt execution failed with error code: {result.error_code.val}")

        if group_name == "arm":
            self.is_executing_arm = False
            command = self.commands[self.current_command_index]
            servo_gear = command[5]
            gripper_command = [servo_gear, -servo_gear, servo_gear, -servo_gear, -servo_gear, -servo_gear]
            self.get_logger().debug("Arm completed, starting gripper")
            self.execute_group_command(self.gripper_move_group_client, "gripper", self.gripper_joint_names, gripper_command)
            self.is_executing_gripper = True
        else:
            self.is_executing_gripper = False
            self.last_command_time = time.time()
            self.get_logger().debug(f"Gripper completed, advancing to command {self.current_command_index + 1}")
            self.current_command_index += 1
            self.log_to_csv()
            self.execute_command()

    def check_joint_states(self):
        if not hasattr(self, 'combined_joint_states') or not hasattr(self, 'true_joint_states'):
            self.get_logger().debug("Waiting for joint state messages...")
        else:
            self.get_logger().debug("Joint states received, monitoring...")

    def log_to_csv(self):
        if not self.enable_logging:
            self.get_logger().debug("Logging disabled, skipping CSV write.")
            return

        current_time = time.time()
        command = (str(self.commands[self.current_command_index]) if (self.is_executing_arm or self.is_executing_gripper) and
                   self.current_command_index < len(self.commands) else "Idle")
        arm_success = "Yes" if not self.is_executing_arm else "In Progress"
        gripper_success = "Yes" if not self.is_executing_gripper else "In Progress"

        true_joints = true_positions = true_velocities = "N/A"
        combined_joints = combined_positions = combined_velocities = "N/A"

        if hasattr(self, 'true_joint_states') and self.true_joint_states and self.true_joint_states.name:
            true_joints = ' '.join(self.true_joint_states.name)
            true_positions = ' '.join(map(str, self.true_joint_states.position)) if self.true_joint_states.position else "N/A"
            true_velocities = ' '.join(map(str, self.true_joint_states.velocity)) if self.true_joint_states.velocity else "N/A"

        if hasattr(self, 'combined_joint_states') and self.combined_joint_states and self.combined_joint_states.name:
            combined_joints = ' '.join(self.combined_joint_states.name)
            combined_positions = ' '.join(map(str, self.combined_joint_states.position)) if self.combined_joint_states.position else "N/A"
            combined_velocities = ' '.join(map(str, self.combined_joint_states.velocity)) if self.combined_joint_states.velocity else "N/A"

        try:
            with open(self.csv_file_path, mode='a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    command, current_time,
                    true_joints, true_positions, true_velocities,
                    combined_joints, combined_positions, combined_velocities,
                    arm_success, gripper_success
                ])
            self.get_logger().debug(f"Logged to CSV: {command}, {arm_success}, {gripper_success}")
        except Exception as e:
            self.get_logger().error(f"Failed to write to CSV: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    try:
        node = JointSyncMoveItNode()
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()