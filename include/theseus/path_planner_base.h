#ifndef PATH_PLANNER_BASE_H
#define PATH_PLANNER_BASE_H

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
#include <theseus/GPS.h>
#include <theseus/ned2gps.h>

#include <rosplane_msgs/Waypoint.h>
#include <rosplane_msgs/NewWaypoints.h>
#include <rosplane_msgs/State.h>
#include <uav_msgs/JudgeMission.h>
#include <uav_msgs/GeneratePath.h>
#include <uav_msgs/UploadPath.h>
#include <std_srvs/Trigger.h>

namespace theseus
{
struct rrtOptions
{
  bool landing;
  bool direct_hit;
  bool now;
  bool check_wps;
  bool drop_bomb;
};
class PathPlannerBase
{
public:
  PathPlannerBase();
  ~PathPlannerBase();
private:
  //********************* NODE HANDLES *********************//
  ros::NodeHandle nh_;         // public node handle for publishing, subscribing

  //************** SUBSCRIBERS AND PUBLISHERS **************//
  ros::ServiceServer plan_mission_service_;
  ros::Subscriber state_subscriber_;
  ros::Subscriber fstate_subscriber_;
  ros::ServiceClient waypoint_client_;
  ros::ServiceServer path_solver_service1_;
  ros::ServiceServer path_solver_service2_;
  ros::ServiceServer path_solver_service3_;
  ros::ServiceServer path_solver_service4_;
  ros::ServiceServer path_solver_service5_;
  ros::ServiceServer path_solver_service6_;
  ros::ServiceServer path_solver_service7_;
  ros::ServiceServer path_solver_service8_;
  ros::ServiceServer new_map_service_;
  ros::ServiceServer send_wps_service_;
  ros::ServiceServer replot_map_service_;
  ros::ServiceServer wp_distance_service_;
  ros::ServiceServer translate_bdry_srv_;
  ros::ServiceServer translate_map_srv_;
  ros::ServiceServer convert_ned_srv_;
  ros::ServiceServer convert_gps_srv_;
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
  std::vector<NED_s> last_primary_wps_;
  std::vector<NED_s> all_sent_wps_;
  std::vector<int> all_sent_priorities_;
  rrtColors clr;
  gps_struct gps_converter_;
  RRT rrt_obj_;
  NED_s odometry_;
  float chi0_;
  bool recieved_state_;
  bool has_map_;
  std::vector<NED_s> all_primary_wps_;

  std::vector<float> wp_distances_;
  std::vector<float> cyl_distances_;
  float min_cyl_dis_;
  //***************** CALLBACKS AND TIMERS *****************//
  ros::WallTimer update_viz_timer_;
  void updateViz(const ros::WallTimerEvent&);
  bool sendWaypoints(uav_msgs::UploadPath::Request &req, uav_msgs::UploadPath::Response &res);
  bool sendWaypointsCore(bool now);

  double lat_ref_;
  double lon_ref_;
  double h_ref_;

  //********************** FUNCTIONS ***********************//
  bool solveStatic(rrtOptions options);
public:
  bool wpsNow(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res);
  bool addWps(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res);
  bool addLanding(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res);
  bool addTextfile(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res);
  bool landNow(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res);
  bool textfileNow(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res);
  bool bombNow(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res);
  bool addBomb(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res);

  bool newRandomMap(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res);
  bool displayMapService(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res);
  bool displayD2WP(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res);
  bool planMission(uav_msgs::GeneratePath::Request &req, uav_msgs::GeneratePath::Response &res);

  bool translateBoundaries(theseus::GPS::Request &req, theseus::GPS::Response &res);
  bool translateMap(theseus::GPS::Request &req, theseus::GPS::Response &res);
  bool convertNED(theseus::ned2gps::Request &req, theseus::ned2gps::Response &res);
  bool convertGPS(theseus::GPS::Request &req, theseus::GPS::Response &res);
private:
  bool landing(bool now);
  bool textfile(bool now);
  void getInitialMap();
  bool bomb(bool now);

};// end class PathPlanner
} // end namespace theseus

#endif // PATH_PLANNER_BASE_H
