#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/LaserScan.h>

class LookAhead {
  private:
    std_msgs::Bool is_ahead;
    ros::Publisher obstacle_indicator;
    ros::Subscriber scan_sub;
    int field_of_vision = 2;
    double min_obstacle_distance = 0.8;
    ros:: NodeHandle n;

  public:
    LookAhead(int argc, char **argv){
      ros::init(argc, argv, "obstacle_indicator");
      ros::NodeHandle n;
      obstacle_indicator = n.advertise<std_msgs::Bool>("obstacle_ahead", 1000);
      scan_sub = n.subscribe("scan", 1000, &LookAhead::scan_callback, this);
    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan){
      int i, ranges_len;
      ranges_len = sizeof(scan->ranges);
      is_ahead.data = false;
      for(i=ranges_len-field_of_vision; i<ranges_len; i++){
        if(scan->ranges[i] < min_obstacle_distance){
          is_ahead.data = true;
        }
      }
      for(i=0; i<field_of_vision; i++){
        if(scan->ranges[i] < min_obstacle_distance){
          is_ahead.data = true;
        }
      }
    }

    void execute(){
      ros::Rate loop_rate(10);

      while(ros::ok()){
        obstacle_indicator.publish(is_ahead);
        ros::spinOnce();
        loop_rate.sleep();
      }
    }
};

int main(int argc, char **argv){
  LookAhead indicator(argc, argv);
  indicator.execute();
}
