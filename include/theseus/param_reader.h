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
  bool simulating;

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
	double N0;
	double E0;
	double D0;
	double chi0;
	std::string boundaries_in_file;
	std::string latitude0;
	std::string longitude0;
	double height0;
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
