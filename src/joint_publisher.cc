#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <unitree_go/msg/low_state.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

class JointPublisher : public rclcpp::Node
{
public:
  JointPublisher() : Node("go2_joint_publisher") {
    RCLCPP_INFO(this->get_logger(), "Go2 Joint Publisher");
    joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    low_state_subscription_ = this->create_subscription<unitree_go::msg::LowState>(
      "lowstate", 10, std::bind(&JointPublisher::low_state_callback, this, std::placeholders::_1));
  }
  ~JointPublisher() = default;

private:
  void low_state_callback(const unitree_go::msg::LowState::SharedPtr msg) {
      sensor_msgs::msg::JointState joint_state;
      joint_state.header.stamp = this->now();
      joint_state.name = {
        "FL_hip_joint",
        "FL_thigh_joint",
        "FL_calf_joint",
        "FR_hip_joint",
        "FR_thigh_joint",
        "FR_calf_joint",
        "RL_hip_joint",
        "RL_thigh_joint",
        "RL_calf_joint",
        "RR_hip_joint",
        "RR_thigh_joint",
        "RR_calf_joint",
      };
      joint_state.position = {
        msg->motor_state[3].q, msg->motor_state[4].q, msg->motor_state[5].q,
        msg->motor_state[0].q, msg->motor_state[1].q, msg->motor_state[2].q,
        msg->motor_state[9].q, msg->motor_state[10].q, msg->motor_state[11].q,
        msg->motor_state[6].q, msg->motor_state[7].q, msg->motor_state[8].q,
      };
      joint_state_publisher_->publish(joint_state);
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_state_subscription_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointPublisher>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}