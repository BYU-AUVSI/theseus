#ifndef ROTATE_VIZ_H
#define ROTATE_VIZ_H

#include <ros/ros.h>
#include <theseus/mapper.h>
#include <theseus/rrt_plotter.h>

#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Quaternion.h>

namespace theseus
{
class RotateViz
{
public:
  RotateViz();
  ~RotateViz();
private:
  //********************* NODE HANDLES *********************//
  ros::NodeHandle nh_;         // public node handle for publishing, subscribing

  //************** SUBSCRIBERS AND PUBLISHERS **************//
  tf::TransformBroadcaster pose_broadcaster_;
  geometry_msgs::TransformStamped odom_trans_;
  ros::Time initial_time_;
  ros::WallTimer rotate_viz_timer_;
  void rotateViz(const ros::WallTimerEvent&);
  rrtPlotter plt;
  rrtColors clr;
};// end class RotateViz
} // end namespace theseus

#endif // ROTATE_VIZ_H
