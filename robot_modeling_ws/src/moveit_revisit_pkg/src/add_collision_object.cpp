#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>


int main(int argc, char** argv){
  ros::init(argc, argv, "object_addition");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::PlanningSceneInterface current_scene;

  ROS_INFO("Wating 5 seconds to obtain current planning scene");
  sleep(5.0);

  moveit_msgs::CollisionObject cylinder;
  cylinder.id = "collision_cylinder";

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.6;
  primitive.dimensions[1] = 0.2;
  primitive.dimensions[2] = 0.2;

  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;
  pose.position.x = 0.0;
  pose.position.y = -0.4;
  pose.position.z = -0.4;

  cylinder.primitives.push_back(primitive);
  cylinder.primitive_poses.push_back(pose);
  cylinder.operation = cylinder.ADD;
  cylinder.header.frame_id = "base_link";

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(cylinder);

  current_scene.addCollisionObjects(collision_objects);

  ROS_INFO("This is the end of the program");

  //ros::spin();

  return 0;
}
