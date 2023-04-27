#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>

// Set value according to your image number in your img_data folder.
#define ARROW_RIGHT 4
#define ARROW_UP 5
#define ARROW_LEFT 6

ros::Publisher vel_pub;
geometry_msgs::Twist vel_msg;

void objectCallback(const std_msgs::Float32MultiArrayPtr &object)
{
    if (object->data.size() > 0){
        int id = object->data[0];

        switch (id){
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
        default: // other object
            vel_msg.linear.x = 0;
            vel_msg.angular.z = 0;
        }
    }
    else{   
        vel_msg.linear.x = 0;
        vel_msg.angular.z = 0;
    }

    vel_pub.publish(vel_msg);
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "img_control");
   ros::NodeHandle n("~");
   ros::Rate loop_rate(30);
   ros::Subscriber sub = n.subscribe("/objects", 1, objectCallback);
   vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
   while (ros::ok())
   {
      ros::spinOnce();
      loop_rate.sleep();
   }
}