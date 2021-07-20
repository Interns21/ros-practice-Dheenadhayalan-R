#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>


float get_pose_val(int i, char xz){
  float pose_x[] = {0.1, 0.1, -0.1, -0.1};
  float pose_z[] = {0.05, 0.25, 0.25, 0.05};
  return (xz == 'x')?pose_x[i]:pose_z[i];
}

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_test_1");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  const std::string PLANNING_GROUP="penguin_arm";

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  const robot_state::JointModelGroup* joint_model_group =
  move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  tf2::Quaternion pitch_n90;
  pitch_n90.setRPY(0, -1.57, 0);

  geometry_msgs::Pose pose;
  pose.orientation.x = pitch_n90[0];
  pose.orientation.y = pitch_n90[1];
  pose.orientation.z = pitch_n90[2];
  pose.orientation.w = pitch_n90[3];
  pose.position.x = -0.1;
  pose.position.y = 0.05;
  pose.position.z = 0.05;

  move_group.setPoseTarget(pose);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (!success){
    ROS_INFO("Planning failed for zeroth step");
    return 1;
  }

  move_group.execute(plan);

  std::vector<moveit::planning_interface::MoveGroupInterface::Plan> plan_to_follow;

  for(int i=0; i<4; i++){
    pose.position.x = get_pose_val(i, 'x');
    pose.position.y = get_pose_val(i, 'z');
    move_group.setPoseTarget(pose);
    success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(success){
      plan_to_follow.push_back(plan);
    }
    else{
      ROS_WARN("Planning failed for %d", i+1);
      return 1;
    }
    robot_state::RobotState start_state(*move_group.getCurrentState());
    start_state.setFromIK(joint_model_group, pose);
    move_group.setStartState(start_state);
  }

  while(ros::ok()){
    std::vector<moveit::planning_interface::MoveGroupInterface::Plan>::iterator plan_4_now;
    for(plan_4_now = plan_to_follow.begin(); plan_4_now < plan_to_follow.end(); plan_4_now++){
      move_group.execute(*plan_4_now);
    }
  }

  return 0;
}
