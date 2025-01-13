#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

class SenderNode : public rclcpp::Node
{
 public:
  SenderNode() : Node("SenderNode")
  {
    m_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("/topic", 10);
    m_timer = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&SenderNode::t_callback, this));
  }

 private:
  void t_callback()
  {
    auto msg = std_msgs::msg::Float32MultiArray();
    msg.data.push_back(1.0);
    msg.data.push_back(2.0);
    m_pub->publish(msg);
  }
  rclcpp::TimerBase::SharedPtr m_timer;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr m_pub;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SenderNode>());
  rclcpp::shutdown();
  return 0;
}