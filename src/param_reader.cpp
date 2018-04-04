#include <theseus/param_reader.h>

namespace theseus
{
ParamReader::ParamReader() :
  nh_(ros::NodeHandle())
{
  if (!(ros::param::get("pp/Va",Va)))
    ROS_WARN("No param named 'Va'");
  if (!(ros::param::get("pp/turn_radius",turn_radius)))
    ROS_WARN("No param named 'turn_radius'");
  if (!(ros::param::get("pp/climb_angle",climb_angle)))
    ROS_WARN("No param named 'climb_angle'");
  if (!(ros::param::get("pp/descend_angle",descend_angle)))
    ROS_WARN("No param named 'descend_angle'");
  if (!(ros::param::get("pp/max_climb_angle",max_climb_angle)))
    ROS_WARN("No param named 'max_climb_angle'");
  if (!(ros::param::get("pp/max_descend_angle",max_descend_angle)))
    ROS_WARN("No param named 'max_descend_angle'");
  if (!(ros::param::get("pp/clearance",clearance)))
    ROS_WARN("No param named 'clearance'");
  if (!(ros::param::get("pp/iters_limit",iters_limit)))
    ROS_WARN("No param named 'iters_limit'");
  if (!(ros::param::get("pp/seed",seed)))
    ROS_WARN("No param named 'seed'");
  if (!(ros::param::get("pp/simulating",simulating)))
    ROS_WARN("No param named 'simulating'");

  if (!(ros::param::get("ppsim/numWps",numWps)))
    ROS_WARN("No param named 'numWps'");
  if (!(ros::param::get("ppsim/is3D",is3D)))
    ROS_WARN("No param named 'is3D'");
  if (!(ros::param::get("ppsim/minCylRadius",minCylRadius)))
    ROS_WARN("No param named 'minCylRadius'");
  if (!(ros::param::get("ppsim/maxCylRadius",maxCylRadius)))
    ROS_WARN("No param named 'maxCylRadius'");
  if (!(ros::param::get("ppsim/minCylHeight",minCylHeight)))
    ROS_WARN("No param named 'minCylHeight'");
  if (!(ros::param::get("ppsim/maxCylHeight",maxCylHeight)))
    ROS_WARN("No param named 'maxCylHeight'");
  if (!(ros::param::get("ppsim/minFlyHeight",minFlyHeight)))
    ROS_WARN("No param named 'minFlyHeight'");
  if (!(ros::param::get("ppsim/maxFlyHeight",maxFlyHeight)))
    ROS_WARN("No param named 'maxFlyHeight'");
  if (!(ros::param::get("ppsim/waypoint_clearance",waypoint_clearance)))
    ROS_WARN("No param named 'waypoint_clearance'");
  if (!(ros::param::get("ppsim/nCyli",nCyli)))
    ROS_WARN("No param named 'nCyli'");
  bool testing;
  nh_.param<bool>("testing/init_references", testing, false);
  if (testing)
  {
    nh_.param<double>("lat_ref", lat_ref, 38.14326388888889);
    nh_.param<double>("lon_ref", lon_ref, -76.43075);
    nh_.param<double>("h_ref", h_ref, 6.701);
    nh_.param<float>("N_init", N_init, 0.0);
    nh_.param<float>("E_init", E_init, 0.0);
    nh_.param<float>("D_init", D_init, 0.0);
    chi0 = 0.0;
  }

	double deg2rad    = M_PI/180.0;
	climb_angle       = climb_angle*deg2rad;
	descend_angle     = descend_angle*deg2rad;
	max_climb_angle   = max_climb_angle*deg2rad;
	max_descend_angle = max_descend_angle*deg2rad;
}
ParamReader::~ParamReader()
{
}
}// end namesoace theseus
