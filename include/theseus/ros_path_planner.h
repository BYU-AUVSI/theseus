#ifndef ROS_PATH_PLANNER_H
#define ROS_PATH_PLANNER_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <rosplane_msgs/Waypoint.h>
#include <rosplane_msgs/State.h>
#include <uav_msgs/JudgeMission.h>
#include <theseus/AuvsiMap.h>
#include <theseus/AuvsiStaticObstacle.h>
#include <theseus/AuvsiBoundaries.h>
#include <theseus/RRT.h>
#include <vector>
#include <cmath>
#include <algorithm>
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
  ros::Subscriber state_subscriber_;
  ros::Publisher waypoint_publisher_;
  ros::Publisher marker_pub_;
  ros::ServiceServer path_solver_service_;
  ros::ServiceServer new_map_service_;
  RRT_input rrt_i_;
  mapper myWorld_;
  std::vector<std::vector<double> > fillet_mav_path(std::vector<double>, std::vector<double>, std::vector<double>,\
                                               std::vector<double>, std::vector<double>, std::vector<double>);
 std::vector<std::vector<double> > fillet_path(std::vector<double> x_path_data,\
                                                          std::vector<double> y_path_data,\
                                                          std::vector<double> d_path_data);
  std::vector<std::vector<double > > arc(double N, double E, double r, double aS, double aE);
  void state_callback(const rosplane_msgs::State &msg);
public:
  ros::Publisher mission_map_publisher_;
  //******************** CLASS VARIABLES *******************//
  float Va_;
  RandGen rg_;
  ParamReader input_file_;
private:
  RRT rrt_obj_;
  int state_id_;
  long unsigned int last_odom_index_;
  visualization_msgs::Marker odom_mkr_;
  //***************** CALLBACKS AND TIMERS *****************//
  std::vector<std::vector<double> > odometry_;
  ros::WallTimer update_viz_timer_;
  void updateViz(const ros::WallTimerEvent&);
  //********************** FUNCTIONS ***********************//
public:
  bool solve_static(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res);
  bool new_map(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res);
};// end class PathPlanner
} // end namespace rosplane

#endif // ROS_PATH_PLANNER_H
