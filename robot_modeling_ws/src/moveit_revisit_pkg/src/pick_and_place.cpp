#include <ros/ros.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


double TABLE_HEIGHT = 0.2;
double TABLE_DIST = 0.4;
double EEF_OFFSET = 0.145;
double EXTRA_OFFSET = 0.01;
double OBJECT_HEIGHT = 0.06;
double OBJECT_WIDTH = 0.03;

void setParameters(ros::NodeHandle& node_handle){
    if(node_handle.getParam("table_dist", TABLE_DIST)){
        ROS_INFO("Table spawning distance from origin is set to %lf", TABLE_DIST);
    }
    else{
        ROS_INFO("Default: Table spawning distance from origin is set to %lf", TABLE_DIST);
    }
    
    if(node_handle.getParam("table_height", TABLE_HEIGHT)){
        ROS_INFO("Height of the table is set to %lf", TABLE_HEIGHT);
    }
    else{
        ROS_INFO("Default: Height of the table is set to %lf", TABLE_HEIGHT);
    }
    
    if(node_handle.getParam("object_height", OBJECT_HEIGHT)){
        ROS_INFO("Height of the object is set to %lf", OBJECT_HEIGHT);
    }
    else{
        ROS_INFO("Default: Height of the object is set to %lf", OBJECT_HEIGHT);
    }
    
    if(node_handle.getParam("object_width", OBJECT_WIDTH)){
        ROS_INFO("Width of the object is set to %lf", OBJECT_WIDTH);
    }
    else{
        ROS_INFO("Default: Width of the object is set to %lf", OBJECT_WIDTH);
    }
    
    if(node_handle.getParam("eef_offset", EEF_OFFSET)){
        ROS_INFO("End effector offset value is set to %lf", EEF_OFFSET);
    }
    else{
        ROS_INFO("Default: End effector offset value is set to %lf", EEF_OFFSET);
    }
    
    if(node_handle.getParam("extra_offset", EXTRA_OFFSET)){
        ROS_INFO("Extra offset value is set to %lf", EXTRA_OFFSET);
    }
    else{
        ROS_INFO("Default: Extra offset value is set to %lf", EXTRA_OFFSET);
    }   
}

void openGripper(trajectory_msgs::JointTrajectory& posture){
    posture.joint_names.resize(2);
    posture.joint_names[0] = "hand_finger_1";
    posture.joint_names[1] = "hand_finger_2";

    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.00;
    posture.points[0].positions[1] = 0.00;
    posture.points[0].time_from_start = ros::Duration(0.5);
}

void closedGripper(trajectory_msgs::JointTrajectory& posture){
    posture.joint_names.resize(2);
    posture.joint_names[0] = "hand_finger_1";
    posture.joint_names[1] = "hand_finger_2";

    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = -0.03;
    posture.points[0].positions[1] = 0.03;
    posture.points[0].time_from_start = ros::Duration(0.5);
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group){
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    grasps[0].grasp_pose.header.frame_id = "base_link";
    tf2::Quaternion orientation;
    orientation.setRPY(-M_PI / 2, 0, -M_PI / 2);
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position.x = TABLE_DIST - EEF_OFFSET - EXTRA_OFFSET;
    grasps[0].grasp_pose.pose.position.y = 0;
    grasps[0].grasp_pose.pose.position.z = TABLE_HEIGHT + OBJECT_HEIGHT / 2;

    grasps[0].pre_grasp_approach.direction.header.frame_id = "base_link";
    grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.06;
    grasps[0].pre_grasp_approach.desired_distance = 0.07;

    grasps[0].post_grasp_retreat.direction.header.frame_id = "base_link";
    grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
    grasps[0].post_grasp_retreat.min_distance = 0.05;
    grasps[0].post_grasp_retreat.desired_distance = 0.07;

    openGripper(grasps[0].pre_grasp_posture);
    ros::WallDuration(1.0).sleep();

    closedGripper(grasps[0].grasp_posture);
    ros::WallDuration(1.0).sleep();

    move_group.setSupportSurfaceName("table1");
    ros::WallDuration(1.0).sleep();

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
    place_location[0].place_pose.pose.position.y = TABLE_DIST;  // - EEF_OFFSET - EXTRA_OFFSET;
    place_location[0].place_pose.pose.position.z = TABLE_HEIGHT + OBJECT_HEIGHT / 2;

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
    collision_objects[0].primitive_poses[0].position.x = TABLE_DIST;
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
    collision_objects[1].primitive_poses[0].position.y = TABLE_DIST;
    collision_objects[1].primitive_poses[0].position.z = TABLE_HEIGHT / 2;

    collision_objects[1].operation = collision_objects[1].ADD;

    collision_objects[2].id = "object";
    collision_objects[2].header.frame_id = "base_link";
    
    collision_objects[2].primitives.resize(1);
    collision_objects[2].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[2].primitives[0].dimensions.resize(3);
    collision_objects[2].primitives[0].dimensions[0] = OBJECT_WIDTH;
    collision_objects[2].primitives[0].dimensions[1] = OBJECT_WIDTH;
    collision_objects[2].primitives[0].dimensions[2] = OBJECT_HEIGHT;

    collision_objects[2].primitive_poses.resize(1);
    collision_objects[2].primitive_poses[0].orientation.w = 1.0;
    collision_objects[2].primitive_poses[0].position.x = TABLE_DIST;
    collision_objects[2].primitive_poses[0].position.y = 0;
    collision_objects[2].primitive_poses[0].position.z = TABLE_HEIGHT + OBJECT_HEIGHT / 2;

    collision_objects[2].operation = collision_objects[2].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "pick_and_place");
    ros::NodeHandle node_handle("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::WallDuration(1.0).sleep();
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface group("penguin_arm");
    group.setPlanningTime(45.0);

    setParameters(node_handle);
    ros::WallDuration(1.0).sleep();

    addCollisionObjects(planning_scene_interface);

    ros::WallDuration(1.0).sleep();

    ROS_INFO("Executing pick action");
    pick(group);

    ros::WallDuration(1.0).sleep();

    ROS_INFO("Executing place action");
    place(group);

    ros::WallDuration(1.0).sleep();

    group.setNamedTarget("ready");
    ROS_INFO("Moving to ready state");
    group.move();

    ROS_INFO("Program ended press ^C to exit");
    ros::waitForShutdown();

    return 0;
}
