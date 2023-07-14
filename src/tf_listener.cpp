#include <chrono>
#include <functional>
#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

class TF_Listener : public rclcpp::Node
{
public:
  TF_Listener() : Node("tf_listener")
  {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    timer_ = create_wall_timer(10ms, std::bind(&TF_Listener::timer_callback, this));

    RCLCPP_INFO(get_logger(), "Node started!");
  }

private:
  void timer_callback()
  {
    try {
      geometry_msgs::msg::TransformStamped transform =
        tf_buffer_->lookupTransform("map", "robot", tf2::TimePointZero);

      geometry_msgs::msg::Quaternion q_msg = transform.transform.rotation;
      tf2::Quaternion q_tf(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
      tf2::Matrix3x3 m(q_tf);

      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      RCLCPP_INFO(this->get_logger(), "Yaw: %f", yaw);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform: %s", ex.what());
    }
  }

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TF_Listener>());
  rclcpp::shutdown();
  return 0;
}
