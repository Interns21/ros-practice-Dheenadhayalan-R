#include "ros/ros.h"
#include "practice/demo_srv.h"


bool demo_service_callback(practice::demo_srv::Request &req, practice::demo_srv::Response &res){
  ROS_INFO("Received: %d, %d", req.num_1, req.num_2);
  res.sum = req.num_1 + req.num_2;
  ROS_INFO("Sending: %d", res.sum);
  return true;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "demo_service_server");
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("demo_service", demo_service_callback);
  ROS_INFO("Ready to receive requests");
  ros::spin();

  return 0;
}
