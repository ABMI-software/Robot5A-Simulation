#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class JointStateBridge : public rclcpp::Node
{
public:
    JointStateBridge()
        : Node("joint_state_bridge")
    {
        // Initialize subscribers
        joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states",
            rclcpp::QoS(10).reliable(),
            std::bind(&JointStateBridge::joint_state_callback, this, std::placeholders::_1));

        visual_joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/visual_joint_states",
            rclcpp::QoS(10).reliable(),
            std::bind(&JointStateBridge::visual_joint_state_callback, this, std::placeholders::_1));

        // Initialize publisher for joint states
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::QoS(10).reliable());

        // Initialize publisher for true joint states
        true_joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/true_joint_states", rclcpp::QoS(10).reliable()); // New topic for true joint states

        // Initialize publisher for combined joint states
        combined_joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/combined_joint_states", rclcpp::QoS(10).reliable()); // New topic for combined joint states
    }

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        true_joint_states_ = msg;

        // Publish the true joint states immediately
        true_joint_state_publisher_->publish(*true_joint_states_);


        if (visual_joint_states_ != nullptr) {
            publish_combined_joint_states();
        }
    }

    void visual_joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        visual_joint_states_ = msg;

        if (true_joint_states_ != nullptr) {
            publish_combined_joint_states();
        }
    }

    void publish_combined_joint_states()
    {
        if (true_joint_states_ == nullptr || visual_joint_states_ == nullptr) {
            RCLCPP_WARN(this->get_logger(), "true or visual joint states are null");
            return;
        }

        // Create a new JointState message to publish
        sensor_msgs::msg::JointState combined_joint_states;

        // Set the header from visual joint states
        combined_joint_states.header = visual_joint_states_->header;

        // Combine names and positions
        combined_joint_states.name = true_joint_states_->name; // Start with true joint names
        combined_joint_states.position = true_joint_states_->position; // Start with true joint positions

        // Add visual joint states
        for (size_t i = 0; i < visual_joint_states_->name.size(); ++i) {
            const auto& visual_joint_name = visual_joint_states_->name[i];
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

        // Publish the combined joint states to the new topic
        combined_joint_state_publisher_->publish(combined_joint_states);

        // Publish the combined joint states
        joint_state_publisher_->publish(combined_joint_states);

    }

 
    // Member variables
    sensor_msgs::msg::JointState::SharedPtr true_joint_states_;
    sensor_msgs::msg::JointState::SharedPtr visual_joint_states_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr visual_joint_state_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr combined_joint_state_publisher_; // New publisher
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr true_joint_state_publisher_; // New publisher
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStateBridge>());
    rclcpp::shutdown();
    return 0;
}