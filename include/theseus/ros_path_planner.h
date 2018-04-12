#ifndef ROS_PATH_PLANNER_H
#define ROS_PATH_PLANNER_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <rosplane_msgs/Waypoint.h>
#include <rosplane_msgs/State.h>
#include <uav_msgs/JudgeMission.h>
#include <uav_msgs/GeneratePath.h>
#include <uav_msgs/UploadPath.h>
#include <theseus/AuvsiMap.h>
#include <theseus/AuvsiStaticObstacle.h>
#include <theseus/AuvsiBoundaries.h>
#include <theseus/RRT.h>
#include <vector>
#include <math.h>
#include <algorithm>
#include <std_srvs/Trigger.h>
#include <theseus/mapper.h>
#include <theseus/rand_gen.h>
#include <theseus/param_reader.h>
#include <theseus/gps_struct.h>

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
  ros::ServiceServer plan_mission_service_;
  ros::Subscriber state_subscriber_;
  ros::Publisher waypoint_publisher_;
  ros::Publisher marker_pub_;
  ros::ServiceServer path_solver_service_;
  ros::ServiceServer new_map_service_;
  ros::ServiceServer send_wps_service_;
  ros::ServiceServer replot_map_service_;
  map_s myWorld_;
  std::vector<std::vector<double> > filletMavPath(std::vector<double>, std::vector<double>, std::vector<double>,\
                                                  std::vector<double>, std::vector<double>, std::vector<double>);
  std::vector<std::vector<double> > filletPath(std::vector<double> x_path_data,\
                                               std::vector<double> y_path_data,\
                                               std::vector<double> d_path_data);
  std::vector<std::vector<double > > arc(double N, double E, double r, double aS, double aE);
  void stateCallback(const rosplane_msgs::State &msg);
public:
  ros::Publisher mission_map_publisher_;

  //******************** CLASS VARIABLES *******************//
  float Va_;
  RandGen rg_;
  ParamReader input_file_;
private:
  gps_struct gps_converter_;
  RRT rrt_obj_;
  float odometry_[3];
  float chi0_;
  visualization_msgs::Marker odom_mkr_;
  bool recieved_state_;
  bool has_map_;
  //***************** CALLBACKS AND TIMERS *****************//
  ros::WallTimer update_viz_timer_;
  void updateViz(const ros::WallTimerEvent&);
  bool sendWaypoints(uav_msgs::UploadPath::Request &req, uav_msgs::UploadPath::Response &res);

  //********************** FUNCTIONS ***********************//
public:
  bool solveStatic(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res);
  bool newRandomMap(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res);
  bool displayMapService(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res);
  bool planMission(uav_msgs::GeneratePath::Request &req, uav_msgs::GeneratePath::Response &res);
  void displayPath();
  void displayMap();
};// end class PathPlanner
} // end namespace rosplane

#endif // ROS_PATH_PLANNER_H
