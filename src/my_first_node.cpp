#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("my_node")
  {
    declare_parameter("timer_period_s", 5);
    auto timer_period_s = std::chrono::seconds(get_parameter("timer_period_s").as_int());

    subscriber_ = create_subscription<sensor_msgs::msg::Image>(
      "/image", rclcpp::SensorDataQoS(), std::bind(&MyNode::image_callback, this, _1));
    publisher_ = create_publisher<std_msgs::msg::UInt8>("/brightness", rclcpp::SensorDataQoS());
    timer_ = create_wall_timer(timer_period_s, std::bind(&MyNode::timer_callback, this));
    client_ = create_client<std_srvs::srv::Empty>("/save");
    server_ = create_service<std_srvs::srv::Trigger>(
      "/image_counter", std::bind(&MyNode::counter_callback, this, _1, _2));

    RCLCPP_INFO(get_logger(), "Node started!");
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr image)
  {
    long long sum = 0;
    for (uint8_t value : image->data) {
      sum += value;
    }
    int avg = sum / image->data.size();

    std_msgs::msg::UInt8 brightness_msg;
    brightness_msg.data = avg;
    publisher_->publish(brightness_msg);
  }

  void timer_callback()
  {
    RCLCPP_INFO(get_logger(), "Timer activate");

    if (!client_->wait_for_service(1s)) {
      RCLCPP_ERROR(get_logger(), "Failed to connect to the image save service");
      return;
    }

    saved_imgs_++;
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    auto future = client_->async_send_request(request);
  }

  void counter_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr req,
    const std_srvs::srv::Trigger::Response::SharedPtr res)
  {
    res->success = 1;
    res->message = "Saved images: " + std::to_string(saved_imgs_);
  }

  uint saved_imgs_ = 0;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr server_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}