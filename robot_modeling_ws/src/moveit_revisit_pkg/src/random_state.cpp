#include <moveit/move_group_interface/move_group_interface.h>


int main(int argc, char** argv){
  ros::init(argc, argv, "random_arm_state", ros::init_options::AnonymousName);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  moveit::planning_interface::MoveGroupInterface arm_group("penguin_arm");
  arm_group.setRandomTarget();
  arm_group.move();
  moveit::planning_interface::MoveGroupInterface hand_group("penguin_hand");
  hand_group.setRandomTarget();
  hand_group.move();
  ros::waitForShutdown();

  return 0;
}
