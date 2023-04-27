#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <math.h>


void sendTransformation()
{
   static tf::TransformBroadcaster tf_broadcaster;
   tf::Transform transform;

   double t = ros::Time::now().toSec();

   // Set translation
   transform.setOrigin(tf::Vector3(cos(t), sin(t), 0.0));

   // Set rotation using roll, pitch. yaw
   tf::Quaternion q;
   q.setRPY(0, 0, fmod(t, 2*M_PI)+M_PI_2); 
   transform.setRotation(q);

   tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "robot"));
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "tf_broadcaster");
   ros::NodeHandle n("~");
   ros::Rate loop_rate(100);
   while (ros::ok())
   {
      ros::spinOnce();
      
      sendTransformation();

      loop_rate.sleep();
   }
}