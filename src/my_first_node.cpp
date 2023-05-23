#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

void imageCallback(const sensor_msgs::msg::Image::SharedPtr image)
{
    long long sum = 0;
    for (uint8_t value : image->data)
    {
        sum += value;
    }
    int avg = sum / image->data.size();
    RCLCPP_INFO(rclcpp::get_logger("my_node"), "Brightness: %d", avg);
}

int main(int argc, char **argv)
{
   rclcpp::init(argc, argv);
   auto node = rclcpp::Node::make_shared("my_node");
   rclcpp::Rate loop_rate(30);  

   auto subscriber = node->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image", 3, imageCallback);

   RCLCPP_INFO(node->get_logger(), "Node started!");
   while (rclcpp::ok())
   {
      rclcpp::spin_some(node);
      loop_rate.sleep();
   }
   return 0;
}