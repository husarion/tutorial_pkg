#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/UInt8.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>

uint saved_imgs = 0;

ros::Publisher brightness_pub;
ros::ServiceClient client;

void imageCallback(const sensor_msgs::ImageConstPtr &image)
{
   long long sum = 0;
   for (int value : image->data)
   {
      sum += value;
   }
   int avg = sum / image->data.size();

   std_msgs::UInt8 brightness_value;
   brightness_value.data = avg;
   brightness_pub.publish(brightness_value);
}

void timerCallback(const ros::TimerEvent&)
{
   ROS_INFO("Timer activate");

   std_srvs::Empty srv;
   client.call(srv);
   saved_imgs++;
}

bool saved_img(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
   res.success = 1;
   res.message = "Saved images: " + std::to_string(saved_imgs);
   return true;
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "my_first_node");
   ros::NodeHandle n("~");

   double timer_period_s;
   n.param("timer_period_s", timer_period_s, 5.0);
   
   ros::Subscriber sub = n.subscribe("/camera/color/image_raw", 10, imageCallback);
   brightness_pub = n.advertise<std_msgs::UInt8>("brightness", 1);
   ros::Timer timer = n.createTimer(ros::Duration(timer_period_s), timerCallback);
   client = n.serviceClient<std_srvs::Empty>("/image_saver/save");
   ros::ServiceServer service = n.advertiseService("image_counter", saved_img);

   ROS_INFO("Node started!");
   ros::spin();
}