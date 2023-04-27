
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/UInt8.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>

bool print_b;
ros::Publisher brightness_pub;
int frames_passed = 0;
int saved_imgs = 0;

void imageCallback(const sensor_msgs::ImageConstPtr &image)
{
   long long sum = 0;
   for (int value : image->data)
   {
      sum += value;
   }
   int avg = sum / image->data.size();
   if (print_b)
   {
      std::cout << "Brightness: " << avg << std::endl;
   }
   std_msgs::UInt8 brightness_value;
   brightness_value.data = avg;
   brightness_pub.publish(brightness_value);
   frames_passed++;
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
   ros::Subscriber sub = n.subscribe("/camera/color/image_raw", 10, imageCallback);
   n.param<bool>("print_brightness", print_b, false);
   brightness_pub = n.advertise<std_msgs::UInt8>("brightness", 1);
   ros::ServiceClient client = n.serviceClient<std_srvs::Empty>("/image_saver/save");
   std_srvs::Empty srv;
   ros::ServiceServer service = n.advertiseService("saved_images", saved_img);
   ros::Rate loop_rate(30);

   ROS_INFO("Node started!");
   while (ros::ok())
   {
      ros::spinOnce();
      if (frames_passed > 100)
      {
         client.call(srv);
         frames_passed = 0;
         saved_imgs++;
      }
      loop_rate.sleep();
   }
}