#include <ros/ros.h>
#include "practice/demo_actionAction.h"
#include <actionlib/server/simple_action_server.h>

class demo_actionAction{
protected:
    ros::NodeHandle n;
    actionlib::SimpleActionServer<practice::demo_actionAction> a_server;
    practice::demo_actionFeedback feedback;
    practice::demo_actionResult result;
    std::string action_name;
    int progress;

  public:
    demo_actionAction(std::string name) :
    a_server(n, name, boost::bind(&demo_actionAction::executeCB, this, _1), false),
    action_name(name){
      a_server.registerPreemptCallback(boost::bind(&demo_actionAction::preemptCB, this));
      a_server.start();
    }

    ~demo_actionAction(void){

    }

    void preemptCB(){
      ROS_WARN("%s got preempted!", action_name.c_str());
      result.final_number = progress;
      a_server.setPreempted(result, "I got preempted");
    }

    void executeCB(const practice::demo_actionGoalConstPtr &goal){
      if(!a_server.isActive() || a_server.isPreemptRequested()) return;
      ros::Rate loop_rate(10);
      ROS_INFO("%s is processing the goal %d", action_name.c_str(), goal->goal_number);
      progress = 0;
      while(progress <= goal->goal_number){
        if(!ros::ok()){
          result.final_number = progress;
          a_server.setAborted(result, "I failed!");
          ROS_INFO("%s shutting down", action_name.c_str());
          break;
        }

        if(!a_server.isActive() || a_server.isPreemptRequested()){
          return;
        }

        if(progress < goal->goal_number){
          ROS_INFO("Setting to goal %d / %d", ++progress, goal->goal_number);
          feedback.current_number = progress;
          a_server.publishFeedback(feedback);
        }
        else{
          ROS_INFO("%s succeeded at getting to goal %d", action_name.c_str(), goal->goal_number);
          result.final_number = progress;
          a_server.setSucceeded(result);
          break;
        }
        loop_rate.sleep();
      }
    }
};

int main(int argc, char **argv){
  ros::init(argc, argv, "demo_action");
  ROS_INFO("Starting Demo Action Server");
  demo_actionAction demo_action_obj(ros::this_node::getName());
  ros::spin();

  return 0;
}
