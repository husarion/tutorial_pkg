#include <chrono>
#include <functional>
#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class TF_Brodcaster : public rclcpp::Node
{
public:
  TF_Brodcaster() : Node("tf_broadcaster")
  {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    timer_ = create_wall_timer(10ms, std::bind(&TF_Brodcaster::timer_callback, this));

    RCLCPP_INFO(get_logger(), "Node started!");
  }

private:
  void timer_callback()
  {
    geometry_msgs::msg::TransformStamped transformStamped;

    transformStamped.header.stamp = this->get_clock()->now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "robot";

    double t = this->get_clock()->now().seconds();

    // Set translation
    transformStamped.transform.translation.x = std::cos(t);
    transformStamped.transform.translation.y = std::sin(t);
    transformStamped.transform.translation.z = 0.0;

    // Set rotation using roll, pitch, and yaw
    tf2::Quaternion q;
    q.setRPY(0, 0, std::fmod(t, 2 * M_PI) + M_PI_2);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(transformStamped);
  }

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TF_Brodcaster>());
  rclcpp::shutdown();
  return 0;
}