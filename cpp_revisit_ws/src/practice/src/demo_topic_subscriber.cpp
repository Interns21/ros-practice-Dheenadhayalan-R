#include "ros/ros.h"
#include "std_msgs/Int32.h"


void num_callback(const std_msgs::Int32::ConstPtr& msg){
  ROS_INFO("Received %d", msg->data);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "demo_topic_subscriber");
  ros::NodeHandle n;
  ros::Subscriber num_subscribe = n.subscribe("numbers", 10, num_callback);
  ros::spin();

  return 0;
}
