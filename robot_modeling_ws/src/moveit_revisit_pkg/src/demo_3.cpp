#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>

#include <boost/scoped_ptr.hpp>


int main(int argc, char** argv){
  ros::init(argc, argv, "demo_2");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  const std::string PLANNING_GROUP = "penguin_arm";

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

  robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
  const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);

  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

  planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");

  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlanningManager>> planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  std::string planner_plugin_name;

  if(!node_handle.getParam("planning_plugin", planner_plugin_name))
    ROS_FATAL_STREAM("Could not find planner plugin name");
  try{
    planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
        "moveit_core", "planning_interface::PlannerManager"));
  }
  catch(pluginlib::PluginlibException& ex){
    ROS_FATAL_STREAM("Exception while creating plugin loader " << ex.what());
  }
  try{
    planner_instance.reset(planner_plugin_loader->create)
  }

  return 0;
}
