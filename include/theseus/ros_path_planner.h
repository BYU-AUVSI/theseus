#ifndef ROS_PATH_PLANNER_H
#define ROS_PATH_PLANNER_H

#include <vector>
#include <math.h>
#include <algorithm>
#include <fstream>
#include <ros/ros.h>

#include <theseus/RRT.h>
#include <theseus/mapper.h>
#include <theseus/rand_gen.h>
#include <theseus/param_reader.h>
#include <theseus/gps_struct.h>
#include <theseus/fillet_s.h>
#include <theseus/rrt_plotter.h>

#include <rosplane_msgs/Waypoint.h>
#include <rosplane_msgs/NewWaypoints.h>
#include <rosplane_msgs/State.h>
#include <uav_msgs/JudgeMission.h>
#include <uav_msgs/GeneratePath.h>
#include <uav_msgs/UploadPath.h>
#include <std_srvs/Trigger.h>

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
  ros::ServiceClient waypoint_client_;
  ros::ServiceServer path_solver_service1_;
  ros::ServiceServer path_solver_service2_;
  ros::ServiceServer path_solver_service3_;
  ros::ServiceServer path_solver_service4_;
  ros::ServiceServer path_solver_service5_;
  ros::ServiceServer path_solver_service6_;
  ros::ServiceServer new_map_service_;
  ros::ServiceServer send_wps_service_;
  ros::ServiceServer replot_map_service_;
  ros::ServiceServer wp_distance_service_;
  map_s myWorld_;
  void stateCallback(const rosplane_msgs::State &msg);

public:
  ros::Publisher mission_map_publisher_;

  //******************** CLASS VARIABLES *******************//
  float Va_;
  RandGen rg_;
  ParamReader input_file_;
private:
  NED_s ending_point_;
  float ending_chi_;
  rrtPlotter plt;
  std::vector<NED_s> all_sent_wps_;
  std::vector<int> all_sent_priorities_;
  rrtColors clr;
  gps_struct gps_converter_;
  RRT rrt_obj_;
  float odometry_[3];
  float chi0_;
  bool recieved_state_;
  bool has_map_;

  std::vector<float> wp_distances_;
  std::vector<float> cyl_distances_;
  float min_cyl_dis_;
  //***************** CALLBACKS AND TIMERS *****************//
  ros::WallTimer update_viz_timer_;
  void updateViz(const ros::WallTimerEvent&);
  bool sendWaypoints(uav_msgs::UploadPath::Request &req, uav_msgs::UploadPath::Response &res);
  bool sendWaypointsCore(bool now);

  //********************** FUNCTIONS ***********************//
public:
  bool solveStatic(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res);
  bool addWps(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res);
  bool addLanding(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res);
  bool addTextfile(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res);
  bool landNow(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res);
  bool textfileNow(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res);

  bool newRandomMap(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res);
  bool displayMapService(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res);
  bool displayD2WP(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res);
  bool planMission(uav_msgs::GeneratePath::Request &req, uav_msgs::GeneratePath::Response &res);
private:
  void getInitialMap();

};// end class PathPlanner
} // end namespace rosplane

#endif // ROS_PATH_PLANNER_H
