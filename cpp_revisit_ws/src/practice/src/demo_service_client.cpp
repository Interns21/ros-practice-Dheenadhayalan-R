#include "ros/ros.h"
#include "practice/demo_srv.h"


int main(int argc, char **argv){
  ros::init(argc, argv, "demo_service_client");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  ros::ServiceClient client = n.serviceClient<practice::demo_srv>("demo_service");
  int a = 200, b = 40;
  practice::demo_srv srv;
  for(int i=0; i < 100 && ros::ok; i++){
    srv.request.num_1 = a - i;
    srv.request.num_2 = b + i * 2;
    ROS_INFO("Calling service");
    if(client.call(srv)){
      ROS_INFO("Received: %d", srv.response.sum);
    }
    else{
      ROS_INFO("Failed to call service");
      return 1;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
