#include <ros/ros.h>
#include <std_msgs/String.h>


void chatterLoopback(const std_msgs::String::ConstPtr& msg){
  ROS_INFO("I heard: %s", msg->data.c_str());
}

int main(int args, char **argv){
  ros::init(args, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber chatter_sub = n.subscribe("chatter", 1000, chatterLoopback);
  ros::spin();
}
