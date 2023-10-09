#include "tutorial_pkg/tracker.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace std::placeholders;

Tracker::Tracker() : Node("tracker"), _is_tracker_initialized(false)
{
  // Subscribers
  _img_sub = create_subscription<sensor_msgs::msg::Image>("/image", rclcpp::SensorDataQoS(), bind(&Tracker::_imageCallback, this, _1));

  // Publishers
  _visualization_pub = create_publisher<sensor_msgs::msg::Image>("/visualization", rclcpp::SensorDataQoS());
  _vel_pub = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(get_logger(), "Node started!");
}

void Tracker::_imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  // Convert the image message to an OpenCV Mat
  cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(msg, "bgr8");
  cv::Mat frame = cv_image->image;
  cv::Rect obj;
  geometry_msgs::msg::Twist vel_msg;

  if (!_is_tracker_initialized)
  {
    _initTracker(frame, obj);
  }

  bool ok = _tracker->update(frame, obj);

  if (ok) {
    // Calculate angular speed based on the position of the object
    _designateControl(vel_msg, obj, msg->width);
    RCLCPP_INFO(get_logger(), "Angular velocity: %0.2f", vel_msg.angular.z);
  }
  else {
    // Log a warning message if tracking fails and display it on the image
    RCLCPP_WARN(get_logger(), "Tracking failure detected. Stop vehicle!");
    putText(frame, "Tracking failure detected", cv::Point(100, 80), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 255), 2);
  }

  _vel_pub->publish(vel_msg);

  // Publish visualization with rectangle around the tracked object
  rectangle(frame, obj, cv::Scalar(255, 0, 0), 2, 1);

  cv_image->image = frame;
  auto img_msg = cv_image->toImageMsg();
  _visualization_pub->publish(*img_msg);
}

void Tracker::_initTracker(cv::Mat frame, cv::Rect obj)
{
  obj = selectROI("ROI selector", frame, false);
  _tracker = cv::TrackerKCF::create();
  _tracker->init(frame, obj);
  _is_tracker_initialized = true;
  cv::destroyWindow("ROI selector");
  cv::waitKey(1);
}

void Tracker::_designateControl(geometry_msgs::msg::Twist &vel_msg, cv::Rect obj, uint32_t img_width)
{
    int obj_x_center = obj.x + obj.width / 2;
    int px_to_center = img_width / 2 - obj_x_center;
    float ang_vel = ANGULAR_GAIN * px_to_center / static_cast<float>(img_width);

    // Ensure angular velocity is within bounds
    if ((ang_vel >= -MAX_ANG_VEL && ang_vel <= -MIN_ANG_VEL) || (ang_vel >= MIN_ANG_VEL && ang_vel <= MAX_ANG_VEL)) {
      vel_msg.angular.z = ang_vel;
    }
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Tracker>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}