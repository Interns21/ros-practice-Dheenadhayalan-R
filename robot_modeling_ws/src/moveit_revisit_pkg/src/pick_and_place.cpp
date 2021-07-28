#include <ros/ros.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

const double OBJECT_Z = 0.2;
const double ALL_DIST = 0.3;
const double EEF_OFFSET = 0.145;
const double EXTRA_OFFSET = 0.01;
const double OBJECT_LENGTH = 0.06;
const double OBJECT_WIDTH = 0.03;

const double TABLE_HEIGHT = OBJECT_Z - OBJECT_LENGTH / 2;


void openGripper(trajectory_msgs::JointTrajectory& posture){
    ROS_INFO("Check 1");
    posture.joint_names.resize(2);
    posture.joint_names[0] = "hand_finger_1";
    posture.joint_names[1] = "hand_finger_2";

    ROS_INFO("Check 2");
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = -0.03;
    posture.points[0].positions[1] = 0.03;
    posture.points[0].time_from_start = ros::Duration(0.5);
}

void closedGripper(trajectory_msgs::JointTrajectory& posture){
    posture.joint_names.resize(2);
    posture.joint_names[0] = "hand_finger_1";
    posture.joint_names[1] = "hand_finger_2";

    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.00;
    posture.points[0].positions[1] = 0.00;
    posture.points[0].time_from_start = ros::Duration(0.5);
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group){
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    grasps[0].grasp_pose.header.frame_id = "base_link";
    tf2::Quaternion orientation;
    orientation.setRPY(-M_PI / 2, -M_PI / 4, -M_PI / 2);
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position.x = ALL_DIST - EEF_OFFSET - EXTRA_OFFSET;
    grasps[0].grasp_pose.pose.position.y = 0;
    grasps[0].grasp_pose.pose.position.z = OBJECT_Z;

    grasps[0].pre_grasp_approach.direction.header.frame_id = "base_link";
    grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.06;
    grasps[0].pre_grasp_approach.desired_distance = 0.07;

    grasps[0].post_grasp_retreat.direction.header.frame_id = "base_link";
    grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
    grasps[0].post_grasp_retreat.min_distance = 0.05;
    grasps[0].post_grasp_retreat.desired_distance = 0.07;

    ROS_INFO("Open gripper");
    openGripper(grasps[0].pre_grasp_posture);
    ros::WallDuration(1.0).sleep();

    ROS_INFO("Close gripper");
    closedGripper(grasps[0].grasp_posture);
    ros::WallDuration(1.0).sleep();

    ROS_INFO("Set support surface name");
    move_group.setSupportSurfaceName("table1");
    ros::WallDuration(1.0).sleep();

    ROS_INFO("Pick object");
    move_group.pick("object", grasps);
    ros::WallDuration(1.0).sleep();
}

void place(moveit::planning_interface::MoveGroupInterface& group){
    std::vector<moveit_msgs::PlaceLocation> place_location;
    place_location.resize(1);

    place_location[0].place_pose.header.frame_id = "base_link";
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, M_PI / 2);
    place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);
    place_location[0].place_pose.pose.position.x = 0;
    place_location[0].place_pose.pose.position.y = ALL_DIST - EEF_OFFSET - EXTRA_OFFSET;
    place_location[0].place_pose.pose.position.z = OBJECT_Z;

    place_location[0].pre_place_approach.direction.header.frame_id = "base_link";
    place_location[0].pre_place_approach.direction.vector.z = -1.0;
    place_location[0].pre_place_approach.min_distance = 0.05;
    place_location[0].pre_place_approach.desired_distance = 0.07;

    place_location[0].post_place_retreat.direction.header.frame_id = "base_link";
    place_location[0].post_place_retreat.direction.vector.y = -1.0;
    place_location[0].post_place_retreat.min_distance = 0.06;
    place_location[0].post_place_retreat.desired_distance = 0.07;

    openGripper(place_location[0].post_place_posture);

    group.setSupportSurfaceName("table2");

    group.place("object", place_location);
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface){
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(3);

    collision_objects[0].id = "table1";
    collision_objects[0].header.frame_id = "base_link";
    
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.1;
    collision_objects[0].primitives[0].dimensions[1] = 0.1;
    collision_objects[0].primitives[0].dimensions[2] = TABLE_HEIGHT;

    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;
    collision_objects[0].primitive_poses[0].position.x = ALL_DIST;
    collision_objects[0].primitive_poses[0].position.y = 0;
    collision_objects[0].primitive_poses[0].position.z = TABLE_HEIGHT / 2;

    collision_objects[0].operation = collision_objects[0].ADD;

    collision_objects[1].id = "table2";
    collision_objects[1].header.frame_id = "base_link";
    
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = 0.1;
    collision_objects[1].primitives[0].dimensions[1] = 0.1;
    collision_objects[1].primitives[0].dimensions[2] = TABLE_HEIGHT;

    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].orientation.w = 1.0;
    collision_objects[1].primitive_poses[0].position.x = 0;
    collision_objects[1].primitive_poses[0].position.y = ALL_DIST;
    collision_objects[1].primitive_poses[0].position.z = TABLE_HEIGHT / 2;

    collision_objects[1].operation = collision_objects[1].ADD;

    collision_objects[2].id = "object";
    collision_objects[2].header.frame_id = "base_link";
    
    collision_objects[2].primitives.resize(1);
    collision_objects[2].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[2].primitives[0].dimensions.resize(3);
    collision_objects[2].primitives[0].dimensions[0] = OBJECT_WIDTH;
    collision_objects[2].primitives[0].dimensions[1] = OBJECT_WIDTH;
    collision_objects[2].primitives[0].dimensions[2] = OBJECT_LENGTH;

    collision_objects[2].primitive_poses.resize(1);
    collision_objects[2].primitive_poses[0].orientation.w = 1.0;
    collision_objects[2].primitive_poses[0].position.x = ALL_DIST;
    collision_objects[2].primitive_poses[0].position.y = 0;
    collision_objects[2].primitive_poses[0].position.z = TABLE_HEIGHT + OBJECT_LENGTH / 2;

    collision_objects[2].operation = collision_objects[2].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "pick_and_place");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::WallDuration(1.0).sleep();
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface group("penguin_arm");
    group.setPlanningTime(45.0);

    addCollisionObjects(planning_scene_interface);

    ros::WallDuration(1.0).sleep();

    pick(group);

    ros::WallDuration(1.0).sleep();

    place(group);

    ros::waitForShutdown();

    return 0;
}
