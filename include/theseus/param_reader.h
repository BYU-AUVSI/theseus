#ifndef PARAMREADER_H
#define PARAMREADER_H

#include <ros/ros.h>
#include <string>
#include <math.h>

namespace theseus
{
class ParamReader
{
public:
	ParamReader();
	~ParamReader();

	// PARAMETERS

	// Simulation Settings
	int numWps;
	int seed;

	// Plane settings
  double Va;
	double turn_radius;
	double climb_angle;
	double descend_angle;
	double max_climb_angle;
	double max_descend_angle;

	// General Path Planning Algorithm Settings
	double clearance;
	int iters_limit;

	// Map Settings
  double lat_ref;
  double lon_ref;
  double h_ref;
  float N_init;
  float E_init;
  float D_init;
  bool chi0;
	bool   is3D;
	double minCylRadius;
	double maxCylRadius;
	double minCylHeight;
	double maxCylHeight;
	double minFlyHeight;
	double maxFlyHeight;
	double waypoint_clearance;
	int nCyli;

private:
  //********************* NODE HANDLES *********************//
  ros::NodeHandle nh_;         // public node handle for publishing, subscribing
};
}// end namespace theseus
#endif // PARAMREADER_H
