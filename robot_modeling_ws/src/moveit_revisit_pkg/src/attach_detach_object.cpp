#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "attach_detach");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface arm_group("penguin_arm");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::PlanningSceneInterface current_scene;

  ROS_INFO("Waiting 5 seconds for scene initialisation");

  sleep(5);

  moveit_msgs::CollisionObject grasping_object;
  grasping_object.id = "grasping_object";

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.03;
  primitive.dimensions[1] = 0.03;
  primitive.dimensions[2] = 0.08;

  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = 0.65;

  grasping_object.primitives.push_back(primitive);
  grasping_object.primitive_poses.push_back(pose);
  grasping_object.operation = grasping_object.ADD;
  grasping_object.header.frame_id = "base_link";

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(grasping_object);

  current_scene.addCollisionObjects(collision_objects);
  ROS_INFO("Adding grasping_object to the scene");
  sleep(4);

  arm_group.setNamedTarget("extended");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::planning_interface::MoveItErrorCode success = success = arm_group.plan(my_plan);

  ROS_INFO("Visualizing plan 1 (pose goal) %s", success.val ? "":"FAILED");

  arm_group.move();

  ROS_INFO("Attaching object grasping_object to robot's body");
  arm_group.attachObject(grasping_object.id);

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 0.726282;
  target_pose1.orientation.x= 4.04423e-07;
  target_pose1.orientation.y = -0.687396;
  target_pose1.orientation.z = 4.81813e-07;

  target_pose1.position.x = 0.0261186;
  target_pose1.position.y = 4.50972e-07;
  target_pose1.position.z = 0.273659;
  arm_group.setPoseTarget(target_pose1);

  arm_group.plan(my_plan);

  ROS_INFO("Visualizing plan 1 (pose goal) %s", success.val ? "":"FAILED");

  arm_group.move();

  arm_group.detachObject(grasping_object.id);

  arm_group.setNamedTarget("ready");
  success = arm_group.plan(my_plan);

  ROS_INFO("Visualizing plan 1 (pose goal) %s", success.val ? "":"FAILED");

  arm_group.move();

  return 0;
}
