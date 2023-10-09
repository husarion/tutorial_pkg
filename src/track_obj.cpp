#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>

// Constants
constexpr float MIN_ANG_VEL = 0.15f;
constexpr float MAX_ANG_VEL = 0.5f;
constexpr float ANGULAR_GAIN = 1.7f;  // 1.7 rad/s * the distance of the object from the center of the image

using namespace cv;
using namespace std;
using namespace std::placeholders;

class TrackObjNode : public rclcpp::Node
{
public:
  TrackObjNode() : Node("track_obj"), is_tracker_initialized_(false)
  {
    // Parameters
    declare_parameter<bool>("visualization", false);
    get_parameter("visualization", visualization_);

    // Publishers
    vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::SystemDefaultsQoS());
    if (visualization_) {
      visualization_pub_ = create_publisher<sensor_msgs::msg::Image>("/visualization", rclcpp::SensorDataQoS());
    }

    // Subscribers
    img_sub_ = create_subscription<sensor_msgs::msg::Image>("/image", rclcpp::SensorDataQoS(), bind(&TrackObjNode::objectCallback, this, _1));

    RCLCPP_INFO(get_logger(), "Node started!");
  }

private:
  void objectCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // Convert the image message to an OpenCV Mat
    cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(msg, "bgr8");
    Mat frame = cv_image->image;
    Rect obj;
    geometry_msgs::msg::Twist vel_msg;

    // Select the object and initialize tracker
    if (!is_tracker_initialized_)
    {
      obj = selectROI("ROI selector", frame, false);
      tracker_ = TrackerKCF::create();
      tracker_->init(frame, obj);
      is_tracker_initialized_ = true;
      destroyWindow("ROI selector");
      waitKey(1);
    }

    // Update the tracker
    bool ok = tracker_->update(frame, obj);

    if (ok) {
      // Calculate and set angular speed based on the position of the object
      int obj_x_center = obj.x + obj.width / 2;
      int px_to_center = msg->width / 2 - obj_x_center;
      float ang_vel = ANGULAR_GAIN * px_to_center / static_cast<float>(msg->width);

      // Ensure angular velocity is within bounds
      if ((ang_vel >= -MAX_ANG_VEL && ang_vel <= -MIN_ANG_VEL) || (ang_vel >= MIN_ANG_VEL && ang_vel <= MAX_ANG_VEL)) {
        vel_msg.angular.z = ang_vel;
      }

      RCLCPP_INFO(get_logger(), "Angular velocity: %f", vel_msg.angular.z);
    }
    else {
      // Log a warning message if tracking fails and display it on the image
      RCLCPP_WARN(get_logger(), "Tracking failure detected. Stop vehicle!");
      putText(frame, "Tracking failure detected", Point(100, 80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 0, 255), 2);
    }

    // Log and publish the calculated angular velocity
    vel_pub_->publish(vel_msg);

    // Publish visualization with rectangle around the tracked object
    if (visualization_) {
      rectangle(frame, obj, Scalar(255, 0, 0), 2, 1);

      cv_image->image = frame;
      auto img_msg = cv_image->toImageMsg();
      visualization_pub_->publish(*img_msg);
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr visualization_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  Ptr<Tracker> tracker_;
  bool is_tracker_initialized_;
  bool visualization_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = make_shared<TrackObjNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
