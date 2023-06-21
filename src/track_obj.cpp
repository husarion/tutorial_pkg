#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/opencv.hpp>

// highlight-start
// Choose object to track and type camera width.
#define OBJECT_TO_FOLLOW 1
#define CAMERA_WIDTH 320  // left 0, right 319
// highlight-end

#define MIN_ANG_VEL 0.15f
#define MAX_ANG_VEL 0.5f
#define ANGULAR_GAIN 1.7  // 1.7 rad/s * the distance of the object from the center of the image

using namespace std::placeholders;

class TrackObjNode : public rclcpp::Node
{
public:
  TrackObjNode() : Node("track_obj")
  {
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

    obj_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/objects", 1, std::bind(&TrackObjNode::objectCallback, this, _1));

    RCLCPP_INFO(get_logger(), "Node started!");
  }

private:
  void objectCallback(const std_msgs::msg::Float32MultiArray::SharedPtr object)
  {
    geometry_msgs::msg::Twist vel_msg;

    if (object->data.size() > 0) {
      int id = object->data[0];
      float objectWidth = object->data[1];
      float objectHeight = object->data[2];

      cv::Mat cvHomography(3, 3, CV_32F);
      std::vector<cv::Point2f> inPts, outPts;
      if (id == OBJECT_TO_FOLLOW) {
        // Matrix completion
        for (int i = 0; i < 9; i++) {
          cvHomography.at<float>(i % 3, i / 3) = object->data[i + 3];
        }

        // Save corners to vector
        inPts.push_back(cv::Point2f(0, 0));
        inPts.push_back(cv::Point2f(objectWidth, 0));
        inPts.push_back(cv::Point2f(0, objectHeight));
        inPts.push_back(cv::Point2f(objectWidth, objectHeight));
        cv::perspectiveTransform(inPts, outPts, cvHomography);

        int obj_x_pos = (outPts.at(0).x + outPts.at(1).x + outPts.at(2).x + outPts.at(3).x) / 4;
        float ang_vel = ANGULAR_GAIN * (CAMERA_WIDTH / 2 - obj_x_pos) / CAMERA_WIDTH;

        // Set angular speed
        if (ang_vel <= -MIN_ANG_VEL || ang_vel >= MIN_ANG_VEL) {
          vel_msg.angular.z = std::max(-MAX_ANG_VEL, std::min(ang_vel, MAX_ANG_VEL));
        }
        RCLCPP_INFO(get_logger(), "id: %d\t ang_vel: %f", id, vel_msg.angular.z);
      }
    }

    vel_pub_->publish(vel_msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr obj_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrackObjNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}