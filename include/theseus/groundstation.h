#ifndef GROUNDSTATION_H
#define GROUNDSTATION_H
#include <ros/ros.h>

#include <theseus/map_s.h>
#include <rosplane_msgs/State.h>
#include <uav_msgs/MovingObstacle.h>
#include <uav_msgs/MovingObstacleCollection.h>
#include <visualization_msgs/Marker.h>
#include <theseus/gps_struct.h>

namespace theseus
{
class Groundstation
{
public:
  Groundstation();
  ~Groundstation();
private:
  //********************* NODE HANDLES *********************//
  ros::NodeHandle nh_;         // public node handle for publishing, subscribing

  //************** SUBSCRIBERS AND PUBLISHERS **************//
  ros::Subscriber state_subscriber_;
  ros::Subscriber fstate_subscriber_;
  ros::Subscriber mobs_subscriber_;
  map_s myWorld_;
  void stateCallback(const rosplane_msgs::State &msg);
  void movingObsCallback(const uav_msgs::MovingObstacleCollection &msg);
private:
  NED_s ending_point_;
  float ending_chi_;
  NED_s odometry_;
  //***************** CALLBACKS AND TIMERS *****************//
  ros::WallTimer update_viz_timer_;
  void updateViz(const ros::WallTimerEvent&);
  ros::Publisher ground_pub_;
  visualization_msgs::Marker odom_mkr_;
  visualization_msgs::Marker mobs_mkr_;
  visualization_msgs::Marker pose_mkr_;
  bool recieved_state_;
  gps_struct gps_converter_;
  double lat_ref_;
  double lon_ref_;
  double h_ref_;

  //********************** FUNCTIONS ***********************//

};// end class Groundstation
} // end namespace theseus

#endif // GROUNDSTATION_H
