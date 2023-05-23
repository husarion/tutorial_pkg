#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("my_node");
  rclcpp::Rate loop_rate(10);  

  RCLCPP_INFO(node->get_logger(), "Node started!");

  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  return 0;
}