#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "get_current_state");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle;

    std::string PLANNING_GROUP = "penguin_arm";

    if(node_handle.getParam("move_group", PLANNING_GROUP)){
        ROS_INFO_STREAM("Planning group: " << PLANNING_GROUP);
    }
    else{
        ROS_INFO_STREAM("(Default) Planning group: " << PLANNING_GROUP);
    }

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    geometry_msgs::Pose pose = (move_group.getCurrentPose()).pose;

    double pos[3];
    pos[0] = pose.position.x;
    pos[1] = pose.position.y;
    pos[2] = pose.position.z;
    
    double rpy[3];
    tf2::Quaternion quaternion;
    tf2::fromMsg(pose.orientation, quaternion);
    tf2::Matrix3x3(quaternion).getRPY(rpy[0], rpy[1], rpy[2]);

    ROS_INFO_STREAM("Position of the movegroup is" << std::endl << pos[0] << " " << pos[1] << " " << pos[2]);
    ROS_INFO_STREAM("Orientation of the movegroup is" << std::endl << rpy[0] << " " << rpy[1] << " " << rpy[2]);

    return 0;
}
