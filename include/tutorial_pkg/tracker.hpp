#ifndef TRACK_OBJ_NODE_HPP
#define TRACK_OBJ_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/tracking.hpp>

class Tracker : public rclcpp::Node
{
public:
  constexpr static float MIN_ANG_VEL = 0.15f;
  constexpr static float MAX_ANG_VEL = 0.5f;
  constexpr static float ANGULAR_GAIN = 1.7f;

  Tracker();

private:
  void _imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void _initTracker(cv::Mat frame, cv::Rect obj);

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _vel_pub;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _visualization_pub;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _img_sub;
  cv::Ptr<cv::Tracker> _tracker;
  bool _is_tracker_initialized;
  bool _visualization;
};

#endif // TRACK_OBJ_NODE_HPP
