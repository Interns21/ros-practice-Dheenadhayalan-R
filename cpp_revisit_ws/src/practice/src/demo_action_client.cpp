#include <ros/ros.h>
#include <iostream>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "practice/demo_actionAction.h"


int main(int argc, char **argv){
  ros::init(argc, argv, "demo_action_client");
  if(argc != 3){
    ROS_INFO("%d", argc);
    ROS_WARN("Usage: demo_action_client <goal_number> <time_to_preempt_in_sec>");
    return 1;
  }

  actionlib::SimpleActionClient<practice::demo_actionAction> a_client("demo_action", true);

  ROS_INFO("Waiting for action server to start");
  a_client.waitForServer();

  ROS_INFO("Action server started, sending goal.");

  practice::demo_actionGoal goal;
  goal.goal_number = atoi(argv[1]);

  ROS_INFO("Sending Goal [%d] and Preempt time of [%d]", goal.goal_number, atoi(argv[2]));
  a_client.sendGoal(goal);

  bool finished_before_timeout = a_client.waitForResult(ros::Duration(atoi(argv[2])));
  ac.cancelGoal();

  if(finished_before_timeout){
    actionlib::SimpleClientGoalState state = a_client.getState();
    ROS_INFO("Action finished: %s", state.toString().c_str());
    ac.cancelGoal();
  }
  else{
    ROS_INFO("Action did not finish before the time out.");
  }

  return 0;
}
