#include <moveit/move_group_interface/move_group_interface.h>


int main(int argc, char** argv){
  ros::init(argc, argv, "simple_test_1");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();


}
