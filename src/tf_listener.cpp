#include <ros/ros.h>
#include <tf/transform_listener.h>


void listenTransformation(tf::TransformListener& tf_listener)
{
   // Listen transformation
   tf::StampedTransform transform;
   tf_listener.lookupTransform("map", "robot", ros::Time(0), transform);

   // Get rotation: roll, pitch, yaw
   tf::Quaternion q = transform.getRotation();
   tf::Matrix3x3 m(q);
   double roll, pitch, yaw;
   m.getRPY(roll, pitch, yaw);

   // Calcualte number of rotation
   static int num_of_rotations;
   static float prev_yaw;
   if(yaw-prev_yaw < 0){
      num_of_rotations++;
      ROS_INFO("Number of rotations: %d", num_of_rotations);
   }
   prev_yaw = yaw;
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "tf_listener");
   ros::NodeHandle n("~");
   ros::Rate loop_rate(100);
   
   tf::TransformListener tf_listener;
   tf_listener.waitForTransform("map", "robot", ros::Time(0), ros::Duration(2.0));
   while (ros::ok())
   {
      ros::spinOnce();
      listenTransformation(tf_listener);
      loop_rate.sleep();
   }
}