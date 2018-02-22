#ifndef ROS_PATH_PLANNER_H
#define ROS_PATH_PLANNER_H

#include <ros/ros.h>
#include <rosplane_msgs/Waypoint.h>
#include <vector>

#include <std_srvs/Trigger.h>
#include <theseus/mapper.h>
#include <theseus/rand_gen.h>
#include <theseus/param_reader.h>


namespace theseus
{
class RosPathPlanner
{
public:
  RosPathPlanner();
  ~RosPathPlanner();
private:
  //********************* NODE HANDLES *********************//
  ros::NodeHandle nh_;         // public node handle for publishing, subscribing

  //************** SUBSCRIBERS AND PUBLISHERS **************//
  ros::Publisher waypoint_publisher_;
  ros::ServiceServer path_solver_service_;
  //******************** CLASS VARIABLES *******************//
  float Va_;

  //***************** CALLBACKS AND TIMERS *****************//

  //********************** FUNCTIONS ***********************//
  virtual bool solve_static(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res);
};// end class PathPlanner
} // end namespace rosplane

#endif // ROS_PATH_PLANNER_H
