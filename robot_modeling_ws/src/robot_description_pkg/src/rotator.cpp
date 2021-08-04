#include <ros/ros.h>
#include <sensor_msgs/JointState.h>


int main(int argc, char** argv){
  ros::init(argc, argv, "rotator");
  ros::NodeHandle node_handle("~");

  double init_angle = 0.0;
  node_handle.getParam("init_angle", init_angle);

  std::vector<std::string> joints{"rotating_base", "base_link_link_1", "link_1_link_2", "link_2_link_3", "link_3_link_4", "link_4_link_5", "link_5_link_6", "link_6_link_7", "hand_finger_1", "hand_finger_2"};
  std::vector<double> angles{init_angle, 0.0, 0.81328, -1.18188, 0.0, -1.29096, 0.0, -0.98172, 0.0, 0.0};
  double step = 0.02;

  ros::Publisher angle_pub = node_handle.advertise<sensor_msgs::JointState>("/rotate_state", 25);
  ros::Rate rate(10);
  sensor_msgs::JointState state;
  state.name = joints;
  state.position = angles;

  double wait_duration = 5.0;
  ros::Duration wait_time(wait_duration);
  ros::Time wait_start = ros::Time::now();

  while(ros::ok && ros::Time::now() - wait_start < wait_time){
    angle_pub.publish(state);
    rate.sleep();
  }

  while(ros::ok()){
    if(state.position[0] >= 3.14){
      state.position[0] = -3.14;
    }
    angle_pub.publish(state);
    ROS_INFO("Published angle: %lf", state.position[0]);
    state.position[0] += step;
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
