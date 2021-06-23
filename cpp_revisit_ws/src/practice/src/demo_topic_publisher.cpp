#include "ros/ros.h"
#include "std_msgs/Int32.h"


int main(int argc, char** argv){
  ros::init(argc, argv, "demo_topic_publisher");
  ros::NodeHandle n;
  ros::Publisher num_publish = n.advertise<std_msgs::Int32>("numbers", 10);
  int count=1;
  ros::Rate loop_rate(10);
  std_msgs::Int32 msg;
  while(ros::ok){
    msg.data = count;
    ROS_INFO("Published %d", msg.data);
    if(count == 100){
      count = 1;
    }
    else{
      ++count;
    }
    num_publish.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
