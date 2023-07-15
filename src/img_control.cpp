#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>

// Set value according to your image number in your img_data folder.
#define ARROW_RIGHT 1
#define ARROW_UP 2
#define ARROW_LEFT 3

using namespace std::placeholders;

class ImgControlNode : public rclcpp::Node
{
public:
  ImgControlNode() : Node("img_control")
  {
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

    obj_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/objects", 1, std::bind(&ImgControlNode::objectCallback, this, _1));

    RCLCPP_INFO(get_logger(), "Node started!");
  }

private:
  void objectCallback(const std_msgs::msg::Float32MultiArray::SharedPtr object)
  {
    geometry_msgs::msg::Twist vel_msg;

    if (object->data.size() > 0) {
      int id = object->data[0];

      switch (id) {
        case ARROW_RIGHT:
          vel_msg.linear.x = 0;
          vel_msg.angular.z = -1;
          break;
        case ARROW_UP:
          vel_msg.linear.x = 0.5;
          vel_msg.angular.z = 0;
          break;
        case ARROW_LEFT:
          vel_msg.linear.x = 0;
          vel_msg.angular.z = 1;
          break;
      }
    } else {
      vel_msg.linear.x = 0;
      vel_msg.angular.z = 0;
    }

    vel_pub_->publish(vel_msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr obj_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImgControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
