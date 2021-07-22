#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit_msgs/AttachedCollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "demo_2");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();

  ros::Publisher planning_scene_diff_publisher =
  node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  ros::WallDuration sleep_t(0.5);
  while(planning_scene_diff_publisher.getNumSubscribers() < 1){
    sleep_t.sleep();
  }
  ROS_INFO("1-Proceeding to next...");

  moveit_msgs::AttachedCollisionObject attached_object;
  attached_object.link_name = "finger_1";
  attached_object.object.header.frame_id = "finger_1";
  attached_object.object.id = "box";

  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.05;
  primitive.dimensions[1] = 0.05;
  primitive.dimensions[2] = 0.05;

  attached_object.object.primitives.push_back(primitive);
  attached_object.object.primitive_poses.push_back(pose);

  attached_object.object.operation = attached_object.object.ADD;

  attached_object.touch_links = std::vector<std::string>{"hand", "finger_1", "finger_2"};

  ROS_INFO("Adding object to the scene");
  moveit_msgs::PlanningScene planning_scene;
  planning_scene.world.collision_objects.push_back(attached_object.object);
  planning_scene.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);
  ROS_INFO("2-Proceeding to next...");

  moveit_msgs::CollisionObject remove_object;
  remove_object.id = "box";
  remove_object.header.frame_id = "base_link";
  remove_object.operation = remove_object.REMOVE;

  ROS_INFO("Attaching object and removing process begin...");
  visual_tools.prompt("Press 'next' to continue...");
  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(remove_object);
  planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
  planning_scene_diff_publisher.publish(planning_scene);
  ROS_INFO("3-Proceeding to next...");

  moveit_msgs::AttachedCollisionObject detach_object;
  detach_object.object.id = "box";
  detach_object.link_name = "link_7";
  detach_object.object.operation = attached_object.object.REMOVE;

  ROS_INFO("Detaching object and adding process begin...");
  visual_tools.prompt("Press 'next' to continue...");
  planning_scene.robot_state.attached_collision_objects.clear();
  planning_scene.robot_state.attached_collision_objects.push_back(detach_object);
  planning_scene.robot_state.is_diff = true;
  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(attached_object.object);
  planning_scene.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);
  ROS_INFO("4-Proceeding to next...");

  ROS_INFO("Removing object from world");
  visual_tools.prompt("Press 'next' to continue...");
  planning_scene.robot_state.attached_collision_objects.clear();
  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(remove_object);
  planning_scene_diff_publisher.publish(planning_scene);

  ROS_INFO("Code came to the end");

  return 0;
}
