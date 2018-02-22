#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <vector>
#include <algorithm>
#include <cmath>
#include <string>
#include <fstream>

#include <theseus/map_s.h>
#include <theseus/rand_gen.h>
#include <theseus/param_reader.h>

#include <theseus/ros_path_planner.h>

namespace theseus
{
class PathPlannerBase : public RosPathPlanner
{
public:
	PathPlannerBase();										// Constructor, sets up some simulation parameters
	virtual ~PathPlannerBase();								//	Deconstructor, frees vector memory
	virtual void solve_static();						// Virtual Function (the child class, algorithm should handel it) Solves for the static path
	double total_path_length;							// Total path length
	int total_nWPS;										// Total number of waypoints
private:
	bool lineAndPoint2d(NED_s ls, NED_s le, double MinMax[], double Mandb[], NED_s p, double r);	// Function called by the LINE flyZoneCheck(NED,NED,radius)
	bool line_intersects_arc(double Ni, double Ei, NED_s cp, NED_s ps, NED_s pe, bool ccw);			// This function finds if the line intersects the arc or not.
protected:
	ParamReader *input_file;								// address of the input file
	std::vector<std::vector<NED_s> > all_wps;						// final path waypoints,
	std::vector<double> path_distances;						// Distances for the final path.
	map_s map;											// This is the terrain map that contains the boundary and obstacle information (static)
	void setup_flyZoneCheck();							// Function that does calculations on the boundary lines in preparation to flyZoneCheck()
	bool flyZoneCheck(const NED_s ps, const NED_s pe, const double r);									// Checks to see if a LINE is at least radius away from an obstacle.
	bool flyZoneCheck(const NED_s ps, const NED_s pe, const double aradius, const NED_s cp, const double r, const bool ccw);	// Checks to see if an ARC is at least radius away from an obstacle.
	bool flyZoneCheck(const NED_s NED, const double r);	// Checks to see if a POINT is at least radius away from an obstacle.
	void compute_performance();							// After an algorithm runs this computes some basic performance stats.
	unsigned int nBPts;									// Number of boundary points
	std::vector<std::vector<double> > lineMinMax;					// (N x 4) vector containing the (min N, max N, min E, max E) for each boundary line
	std::vector<std::vector<double> > line_Mandb;					// (N x 4) vector that contains the slope and intercept of the line (m, b, (-1/m), (m + 1/m)) from N = m*E + b ... not sure about E = constant lines yet.
	RandGen rg;											// Here is the random generator for the algorithm
	void ppSetup();										// This sets up some preliminary things like the below doubles
	double maxNorth;									// Maximum North coordinate inside the boundaries
	double minNorth;									// Minimum North coordinate inside the boundaries
	double maxEast;										// Maximum East  coordinate inside the boundaries
	double minEast;										// Minimum East  coordinate inside the boundaries
	double minFlyHeight;								// Minimum Fly Height (positive value)
	double maxFlyHeight;								// Maximum Fly Height (positive value)
	double clearance;									// The minimum clearance that the path will have away from any obstacles (can fluctuate up and down)
	double path_clearance;								// The minimum clearance that the path from waypoint i to i+1 will have. (only decreases until new waypoint is obtained)
	bool is3D;											// If the simulation is in 3D or 2D
	unsigned int iters_limit;							// This is the maximum number of iterations the solver will do before it will just move on
	bool taking_off;								// If the plane is currently taking off, this option will allow the path planner to ignor the height restricitons.
};
}
#endif
