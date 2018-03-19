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
  if (!(ros::param::get("ppsim/N0",N0)))
    ROS_WARN("No param named 'N0'");
  if (!(ros::param::get("ppsim/E0",E0)))
    ROS_WARN("No param named 'E0'");
  if (!(ros::param::get("ppsim/D0",D0)))
    ROS_WARN("No param named 'D0'");
  if (!(ros::param::get("ppsim/chi0",chi0)))
    ROS_WARN("No param named 'chi0'");
  if (!(ros::param::get("ppsim/boundaries_in_file",boundaries_in_file)))
    ROS_WARN("No param named 'boundaries_in_file'");
  if (!(ros::param::get("ppsim/latitude0",latitude0)))
    ROS_WARN("No param named 'latitude0'");
  if (!(ros::param::get("ppsim/longitude0",longitude0)))
    ROS_WARN("No param named 'longitude0'");
  if (!(ros::param::get("ppsim/height0",height0)))
    ROS_WARN("No param named 'height0'");
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

	double deg2rad    = M_PI/180.0;
	climb_angle       = climb_angle*deg2rad;
	descend_angle     = descend_angle*deg2rad;
	max_climb_angle   = max_climb_angle*deg2rad;
	max_descend_angle = max_descend_angle*deg2rad;
	chi0              = chi0*deg2rad;
}
ParamReader::~ParamReader()
{
}
}// end namesoace theseus
