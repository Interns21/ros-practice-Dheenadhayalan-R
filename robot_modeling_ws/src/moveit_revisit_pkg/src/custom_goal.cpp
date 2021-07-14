#include <moveit/move_group_interface/move_group_interface.h>
//#include <moveit/planning_scene_interface/planning_scene_interface.h>
//#include <moveit_msgs/DisplayRobotState.h>
//#include <moveit_msgs/DisplayTrajectory.h>
//#include <moveit_msgs/AttachedCollisionObject.h>
//#include <moveit_msgs/CollisionObject.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "custom_goal");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface arm_group("penguin_arm");
  //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  //ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  //moveit_msgs::DisplayTrajectory display_trajectory;

  ROS_INFO("Reference frame: %s", arm_group.getPlanningFrame().c_str());
  ROS_INFO("End effetor link: %s", arm_group.getEndEffectorLink().c_str());

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 0.726282;
  target_pose1.orientation.x= 4.04423e-07;
  target_pose1.orientation.y = -0.687396;
  target_pose1.orientation.z = 4.81813e-07;

  target_pose1.position.x = 0.0261186;
  target_pose1.position.y = 4.50972e-07;
  target_pose1.position.z = 0.273659;
  arm_group.setPoseTarget(target_pose1);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::planning_interface::MoveItErrorCode success = arm_group.plan(my_plan);

  ROS_INFO("Visualizing plan 1 (pose goal) %s", success.val ? "":"FAILED");

  arm_group.move();
  ros::shutdown();
  return 0;
}
