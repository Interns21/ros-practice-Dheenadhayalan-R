#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>



int main(int args, char **argv){
  ros::init(args, argv, "talker");
  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);

  std_msgs::String msg;
  int c = 0;
  while(ros::ok()){
    std::stringstream ss;
    ss << "hello world" << c;
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());
    chatter_pub.publish(msg);
    ++c;
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
