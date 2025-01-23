#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>

class JointStateBridge : public rclcpp::Node
{
public:
    JointStateBridge()
        : Node("joint_state_bridge")
    {
        // Initialize subscribers
        visual_joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/visual_joint_states",
            rclcpp::QoS(10).reliable(),
            std::bind(&JointStateBridge::visual_joint_state_callback, this, std::placeholders::_1));

        // Initialize subscribers for arm and gripper controller states
        arm_controller_subscriber_ = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
            "/arm_controller/controller_state",
            rclcpp::QoS(10).reliable(),
            std::bind(&JointStateBridge::arm_controller_callback, this, std::placeholders::_1));

        gripper_controller_subscriber_ = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
            "/gripper_controller/controller_state",
            rclcpp::QoS(10).reliable(),
            std::bind(&JointStateBridge::gripper_controller_callback, this, std::placeholders::_1));

        // Initialize publisher for joint states
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::QoS(10).reliable());

        // Initialize publisher for true joint states
        true_joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/true_joint_states", rclcpp::QoS(10).reliable());

        // Initialize publisher for combined joint states
        combined_joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/combined_joint_states", rclcpp::QoS(10).reliable());
        
        // Initialize publisher for true joint states
        robot_joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "robot/joint_states", rclcpp::QoS(10).reliable());
    }

private:
    void visual_joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        visual_joint_states_ = msg;

        if (true_joint_states_ != nullptr) {
            publish_combined_joint_states();
        }
    }

    void arm_controller_callback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
    {
        arm_joint_states_ = msg;
        recreate_joint_states();
    }

    void gripper_controller_callback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
    {
        gripper_joint_states_ = msg;
        recreate_joint_states();
    }

    void recreate_joint_states()
    {
        if (arm_joint_states_ == nullptr || gripper_joint_states_ == nullptr) {
            RCLCPP_WARN(this->get_logger(), "Arm or gripper joint states are null");
            return;
        }

        // Create a new JointState message for true joint states
        true_joint_states_ = std::make_shared<sensor_msgs::msg::JointState>();

        // Set the header from arm joint states
        true_joint_states_->header.stamp = this->now(); // Set the current time
        true_joint_states_->header.frame_id = ""; // Set frame_id if needed

        // Combine names and positions from arm and gripper states
        true_joint_states_->name = arm_joint_states_->joint_names; // Start with arm joint names
        true_joint_states_->position = arm_joint_states_->reference.positions; // Start with arm joint positions

        // Add gripper joint states
        for (size_t i = 0; i < gripper_joint_states_->joint_names.size(); ++i) {
            const auto& gripper_joint_name = gripper_joint_states_->joint_names[i];
            double gripper_position = gripper_joint_states_->reference.positions[i];

            // Only consider non-zero positions
            if (gripper_position != 0.0) {
                auto it = std::find(true_joint_states_->name.begin(), true_joint_states_->name.end(), gripper_joint_name);
                
                if (it == true_joint_states_->name.end()) {
                    // If the joint name from gripper states is not already in the combined list, add it
                    true_joint_states_->name.push_back(gripper_joint_name);
                    true_joint_states_->position.push_back(gripper_joint_states_->reference.positions[i]); // Add the corresponding position
                } else {
                    // If the joint name already exists, you might want to handle it (e.g., average positions, etc.)
                    size_t index = std::distance(true_joint_states_->name.begin(), it );
                    true_joint_states_->position[index] = (true_joint_states_->position[index] + gripper_joint_states_->reference.positions[i]) / 2.0; // Example: average positions
                }
            }
        }

        // Set velocity and effort to empty lists or nan values

        true_joint_states_->velocity.resize(true_joint_states_->name.size(), std::nan("")); // or use an empty vector

        true_joint_states_->effort.resize(true_joint_states_->name.size(), std::nan("")); // or use an empty vector


        // Publish the true joint states
        true_joint_state_publisher_->publish(*true_joint_states_);
    }

    void publish_combined_joint_states()
    {
        if (true_joint_states_ == nullptr || visual_joint_states_ == nullptr) {
            RCLCPP_WARN(this->get_logger(), "True or visual joint states are null");
            return;
        }

        // Create a new JointState message to publish combined joint states
        sensor_msgs::msg::JointState combined_joint_states;

        // Set the header from visual joint states
        combined_joint_states.header = visual_joint_states_->header;

        // Combine names and positions
        combined_joint_states.name = true_joint_states_->name; // Start with true joint names
        combined_joint_states.position = true_joint_states_->position; // Start with true joint positions
        combined_joint_states.velocity = true_joint_states_->velocity; // Start with true joint velocities
        combined_joint_states.effort = true_joint_states_->effort; // Start with true joint efforts

        // Add visual joint states
        for (size_t i = 0; i < visual_joint_states_->name.size(); ++i) {
            const auto& visual_joint_name = visual_joint_states_->name[i];
            double visual_position = visual_joint_states_->position[i];

            // Only consider non-zero positions
            if (visual_position != 0.0) {
                auto it = std::find(combined_joint_states.name.begin(), combined_joint_states.name.end(), visual_joint_name);
                
                if (it == combined_joint_states.name.end()) {
                    // If the joint name from visual states is not already in the combined list, add it
                    combined_joint_states.name.push_back(visual_joint_name);
                    combined_joint_states.position.push_back(visual_joint_states_->position[i]); // Add the corresponding position
                } else {
                    // If the joint name already exists, you might want to handle it (e.g., average positions, etc.)
                    size_t index = std::distance(combined_joint_states.name.begin(), it);
                    combined_joint_states.position[index] = (combined_joint_states.position[index] + visual_joint_states_->position[i]) / 2.0; // Example: average positions
                }
            }
        }
        
        // Publish the combined joint states to the new topic
        combined_joint_state_publisher_->publish(combined_joint_states);
        // Also publish the combined joint states to the main joint states topic
        joint_state_publisher_->publish(combined_joint_states);

        // robot_joint_state_publisher_->publish(combined_joint_states);
    }

    // Member variables
    sensor_msgs::msg::JointState::SharedPtr true_joint_states_;
    sensor_msgs::msg::JointState::SharedPtr visual_joint_states_;
    control_msgs::msg::JointTrajectoryControllerState::SharedPtr arm_joint_states_;
    control_msgs::msg::JointTrajectoryControllerState::SharedPtr gripper_joint_states_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr visual_joint_state_subscriber_;
    rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr arm_controller_subscriber_;
    rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr gripper_controller_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr combined_joint_state_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr true_joint_state_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr robot_joint_state_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStateBridge>());
    rclcpp::shutdown();
    return 0;
}