#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>

class ArmStateListener : public rclcpp::Node
{
public:
    ArmStateListener() : Node("arm_state_listener")
    {
        // Subscribe to the arm controller state topic for JointState
        arm_joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/arm_controller/state",
            rclcpp::QoS(10).reliable(),
            std::bind(&ArmStateListener::joint_state_callback, this, std::placeholders::_1));

        // // Subscribe to the arm controller state topic for JointTrajectoryControllerState
        // arm_trajectory_controller_subscriber_ = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
        //     "/arm_controller/state",
        //     rclcpp::QoS(10).reliable(),
        //     std::bind(&ArmStateListener::trajectory_controller_callback, this, std::placeholders::_1));
    }

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received JointState:");
        RCLCPP_INFO(this->get_logger(), "Header: %s", msg->header.frame_id.c_str());
        for (size_t i = 0; i < msg->name.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "Joint: %s, Position: %f", msg->name[i].c_str(), msg->position[i]);
        }
    }

    // void trajectory_controller_callback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
    // {
    //     RCLCPP_INFO(this->get_logger(), "Received JointTrajectoryControllerState:");
    //     RCLCPP_INFO(this->get_logger(), "Header: %s", msg->header.frame_id.c_str());
    //     for (size_t i = 0; i < msg->joint_names.size(); ++i) {
    //         RCLCPP_INFO(this->get_logger(), "Joint: %s, Position: %f", msg->joint_names[i].c_str(), msg->actual.positions[i]);
    //     }
    // }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr arm_joint_state_subscriber_;
    // rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr arm_trajectory_controller_subscriber_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmStateListener>());
    rclcpp::shutdown();
    return 0;
}