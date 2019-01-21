#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <frontier_exploration/ExploreTaskAction.h>

frontier_exploration::ExploreTaskGoal createExplorationGoal()
{
  frontier_exploration::ExploreTaskGoal goal;
  geometry_msgs::PointStamped center;
  center.header.frame_id = "map";
  center.point.x = 1.0;
  center.point.y = 0;
  center.point.z = 0;
  goal.explore_center = center;
  geometry_msgs::PolygonStamped square;
  square.header.frame_id = "map";
  std::vector<geometry_msgs::Point32> square_points;
  geometry_msgs::Point32 point_a;
  point_a.x = 30;
  point_a.y = 30;
  point_a.z = 0;
  geometry_msgs::Point32 point_b;
  point_b.x = -30;
  point_b.y = 30;
  point_b.z = 0;
  geometry_msgs::Point32 point_c;
  point_c.x = -30;
  point_c.y = -30;
  point_c.z = 0;
  geometry_msgs::Point32 point_d;
  point_d.x = 30;
  point_d.y = -30;
  point_d.z = 0;
  square_points.push_back(point_a);
  square_points.push_back(point_b);
  square_points.push_back(point_c);
  square_points.push_back(point_d);
  square.polygon.points = square_points;
  goal.explore_boundary = square;
  return goal;
}

int main(int argc, char **argv)
{
  ROS_INFO("Ros init");
  ros::init(argc, argv, "explore_trigger");
  ros::NodeHandle node("~");
  // ros::Subscriber exploration_status_sub = node.subscribe("/explore_server/status", 1, explore_status_callback);
  ros::Publisher explore_canceller;
  explore_canceller = node.advertise<actionlib_msgs::GoalID>("/explore_server/cancel", 1);
  ROS_INFO("Create action client");
  actionlib::SimpleActionClient<frontier_exploration::ExploreTaskAction> exploreClient("explore_server", true);
  ROS_INFO("Waiting for server...");
  exploreClient.waitForServer();
  ROS_INFO("Sending goal");
  exploreClient.sendGoal(createExplorationGoal());
  ROS_INFO("Exploration triggered");
  return 0;
}