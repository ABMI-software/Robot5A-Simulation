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
        self.csv_file_path = os.path.join(
            get_package_share_directory('robot_data_process'), 'logs', 'joint_sync_log.csv'
        )

        # QoS profile
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)

        # Synchronized subscribers
        self.combined_sub = Subscriber(self, JointState, '/combined_joint_states', qos_profile=qos)
        self.true_sub = Subscriber(self, JointState, '/true_joint_states', qos_profile=qos)
        self.ts = ApproximateTimeSynchronizer([self.combined_sub, self.true_sub], queue_size=10, slop=0.05)
        self.ts.registerCallback(self.sync_callback)

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
            raise RuntimeError(f"Failed to connect to any MoveGroup action server: {possible_action_servers}. Ensure move_group node is running and check 'ros2 action list -t'.")

        # Initialize CSV and load commands
        self.initialize_csv()
        self.load_commands('config/joint_commands.txt')

        # Timers
        self.create_timer(4.0, self.start_command_execution)
        self.create_timer(0.1, self.check_command_completion)

    def initialize_csv(self):
        os.makedirs(os.path.dirname(self.csv_file_path), exist_ok=True)
        with open(self.csv_file_path, mode='w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                'Command', 'Time (s)', 'True Joints', 'True Positions', 'True Velocities',
                'Combined Joints', 'Combined Positions', 'Combined Velocities',
                'Arm MoveIt Success', 'Gripper MoveIt Success'
            ])

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
        self.log_to_csv()

    def start_command_execution(self):
        if self.commands:
            self.execute_command()
        else:
            self.get_logger().error("No commands loaded. Execution will not start.")

    def execute_command(self):
        if self.current_command_index >= len(self.commands):
            self.get_logger().info("All commands executed.")
            return

        if not self.is_executing_arm and not self.is_executing_gripper:
            # Check if enough time has elapsed since the last command
            current_time = time.time()
            if self.last_command_time and (current_time - self.last_command_time < self.command_delay):
                return

            command = self.commands[self.current_command_index]
            self.get_logger().info(f"Executing command {self.current_command_index + 1}/{len(self.commands)}: {command}")

            if len(command) != self.expected_command_length:
                self.get_logger().error(f"Command {command} has {len(command)} values, expected {self.expected_command_length}")
                self.current_command_index += 1
                return

            # Split command into arm and gripper parts
            arm_command = command[:5]  # First 5 joints for arm
            servo_gear = command[5]
            gripper_command = [
                servo_gear,      # ServoGear
                -servo_gear,     # LeftGripper
                servo_gear,      # RightGripper
                -servo_gear,     # LeftPivotArm
                -servo_gear,     # RightPivotArm
                -servo_gear      # PassifGear
            ]

            # Execute arm command first
            self.execute_group_command(self.arm_move_group_client, "arm", self.arm_joint_names, arm_command)
            self.is_executing_arm = True

    def execute_group_command(self, client, group_name, joint_names, positions):
        goal = MoveGroup.Goal()
        goal.request.group_name = group_name
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.start_state.is_diff = True

        # Create a Constraints object
        constraints = Constraints()
        for name, pos in zip(joint_names, positions):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = name
            joint_constraint.position = float(pos)
            joint_constraint.tolerance_above = 0.05
            joint_constraint.tolerance_below = 0.05
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)

        # Assign the Constraints object to goal_constraints as a list
        goal.request.goal_constraints = [constraints]

        self.get_logger().info(f"Sending MoveGroup goal for {group_name} with joints: {joint_names}, positions: {positions}")
        client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        ).add_done_callback(lambda future, gn=group_name: self.goal_response_callback(future, gn))

    def feedback_callback(self, feedback_msg):
        pass  # Log feedback if needed

    def goal_response_callback(self, future, group_name):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"{group_name} MoveGroup goal rejected")
            if group_name == "arm":
                self.is_executing_arm = False
            else:
                self.is_executing_gripper = False
            self.current_command_index += 1
            return

        self.get_logger().info(f"{group_name} MoveGroup goal accepted")
        goal_handle.get_result_async().add_done_callback(lambda future, gn=group_name: self.result_callback(future, gn))

    def result_callback(self, future, group_name):
        result = future.result().result
        if result.error_code.val == 1:  # SUCCESS
            self.get_logger().info(f"{group_name} MoveIt execution succeeded")
        else:
            self.get_logger().error(f"{group_name} MoveIt execution failed with error code: {result.error_code.val}")

        if group_name == "arm":
            self.is_executing_arm = False
            command = self.commands[self.current_command_index]
            servo_gear = command[5]
            gripper_command = [
                servo_gear,      # ServoGear
                -servo_gear,     # LeftGripper
                servo_gear,      # RightGripper
                -servo_gear,     # LeftPivotArm
                -servo_gear,     # RightPivotArm
                -servo_gear      # PassifGear
            ]
            self.execute_group_command(self.gripper_move_group_client, "gripper", self.gripper_joint_names, gripper_command)
            self.is_executing_gripper = True
        else:
            self.is_executing_gripper = False
            self.last_command_time = time.time()  # Record time when gripper completes

    def check_command_completion(self):
        if not hasattr(self, 'combined_joint_states') or not hasattr(self, 'true_joint_states'):
            self.get_logger().debug("Waiting for joint state messages...")
            return

        if not self.combined_joint_states.name or not self.combined_joint_states.position:
            self.get_logger().warn("Received empty joint state message (no names or positions)")
            return

        tolerance = 0.05
        if self.is_executing_arm:
            target_positions = self.commands[self.current_command_index][:5]
            if len(self.combined_joint_states.position) < len(target_positions):
                self.get_logger().warn(f"Insufficient joint positions for arm: got {len(self.combined_joint_states.position)}, need {len(target_positions)}")
                return
            current_positions = []
            for joint in self.arm_joint_names:
                if joint in self.combined_joint_states.name:
                    idx = self.combined_joint_states.name.index(joint)
                    current_positions.append(self.combined_joint_states.position[idx])
                else:
                    self.get_logger().warn(f"Joint {joint} not found in combined_joint_states")
                    return
            if len(current_positions) == len(target_positions):
                all_close = all(abs(current_positions[i] - target_positions[i]) < tolerance for i in range(len(target_positions)))
                if all_close:
                    self.get_logger().info(f"Arm command {self.current_command_index + 1} completed")
                    self.is_executing_arm = False

        elif self.is_executing_gripper:
            servo_gear_target = self.commands[self.current_command_index][5]
            target_positions = [
                servo_gear_target,   # ServoGear
                -servo_gear_target,  # LeftGripper
                servo_gear_target,   # RightGripper
                -servo_gear_target,  # LeftPivotArm
                -servo_gear_target,  # RightPivotArm
                -servo_gear_target   # PassifGear
            ]
            current_positions = []
            for joint in self.gripper_joint_names:
                if joint in self.combined_joint_states.name:
                    idx = self.combined_joint_states.name.index(joint)
                    current_positions.append(self.combined_joint_states.position[idx])
                else:
                    self.get_logger().warn(f"Joint {joint} not found in combined_joint_states")
                    return

            if len(current_positions) != len(target_positions):
                self.get_logger().warn(f"Gripper position mismatch: got {len(current_positions)}, expected {len(target_positions)}")
                return

            all_close = all(abs(current_positions[i] - target_positions[i]) < tolerance for i in range(len(target_positions)))
            if all_close:
                self.get_logger().info(f"Gripper command {self.current_command_index + 1} completed")
                self.is_executing_gripper = False
                self.current_command_index += 1
                self.last_command_time = time.time()  # Record time when gripper completes
                self.execute_command()  # Next command will wait for delay

    def log_to_csv(self):
        if not hasattr(self, 'combined_joint_states') or not hasattr(self, 'true_joint_states'):
            return

        current_time = time.time()
        true_joints = ' '.join(self.true_joint_states.name)
        true_positions = ' '.join(map(str, self.true_joint_states.position))
        true_velocities = ' '.join(map(str, self.true_joint_states.velocity)) if self.true_joint_states.velocity else "N/A"
        combined_joints = ' '.join(self.combined_joint_states.name)
        combined_positions = ' '.join(map(str, self.combined_joint_states.position))
        combined_velocities = ' '.join(map(str, self.combined_joint_states.velocity)) if self.combined_joint_states.velocity else "N/A"
        command = (self.commands[self.current_command_index] if (self.is_executing_arm or self.is_executing_gripper) and
                   self.current_command_index < len(self.commands) else "Idle")
        arm_success = "Yes" if not self.is_executing_arm else "In Progress"
        gripper_success = "Yes" if not self.is_executing_gripper else "In Progress"

        with open(self.csv_file_path, mode='a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                command, current_time,
                true_joints, true_positions, true_velocities,
                combined_joints, combined_positions, combined_velocities,
                arm_success, gripper_success
            ])

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