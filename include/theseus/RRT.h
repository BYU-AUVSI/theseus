#ifndef RRT_H
#define RRT_H

#include <stack>
#include <vector>
#include <algorithm>
#include <cmath>
#include <string>
#include <fstream>
#include <ros/ros.h>
#include <theseus/map_s.h>
#include <theseus/rand_gen.h>
#include <theseus/param_reader.h>

// Input Options
namespace theseus
{
struct RRT_input
{
	// input options
	double D;             // This is the distance between each node
	bool uniform2P;       // If this is true the distance between each node is uniform between 0 and the distance to P
	bool gaussianD;       // If this is true the distance between each node is gaussian mean = D, std = gaussianSTD
	double gaussianSTD;   // The standard deviation if the gaussianD is set to true
	bool connect_to_end;  // If true the RRT will check each node to see if it can connect to the end
	int path_type;        // Path type from the parent, 0 = straight line, 1 = fillet, 2 = dubins
	RRT_input()           // Struct constructor, pust the default values here.
	{
		D = 25;
		uniform2P = false;
		gaussianD = false;
		gaussianSTD = 15;
		connect_to_end = true;
		path_type = 1;
	}
};

// Class Definition
class RRT
{
public:
	RRT(map_s map_in, unsigned int seed, ParamReader *input_file, RRT_input RRT_options);		// Constructor - input the terrain map and the random generator seed
	~RRT();                        // Deconstructor - deletes the tree
	void solve_static();  // Solves the static path
  std::vector<std::vector<NED_s> > all_wps; // final path waypoints
private:
	double total_path_length;      // Total path length
  int total_nWPS;                // Total number of waypoints
	RRT_input alg_input;           // This contains all of the options for simpleRRT
	struct node                    // This is the node struct for each spot on the tree
	{
		NED_s NED;                   // North, East Down of the node position
		std::vector<node*> children; // Vector of all of the children nodes
		node* parent;                // Pointer to the parent of this node
		double distance;             // Distance from this node to its parent
		int path_type;               // Path type from the parent, 0 = straight line, 1 = fillet, 2 = dubins
		double available_dist;       // This is the distance from the parent to this node that is linear
		NED_s line_start;            // This is where the line starts to get to this node
	};
	double D;                      // If used, this is the distance the algorithm uses between each node
	std::vector<std::vector<NED_s> > line_starts;										// Line starts for every waypoint
	node* find_closest_node(node* nin, NED_s P, node* minNode, double* minD)// This recursive function return the closes node to the input point P, for some reason it wouldn't go in the cpp...
	{// nin is the node to measure, P is the point, minNode is the closes found node so far, minD is where to store the minimum distance
		// Recursion
		double distance;													// distance to the point P
		for (unsigned int i = 0; i < nin->children.size(); i++)				// For all of the children figure out their distances
		{
			distance = sqrt(pow(P.N - nin->children[i]->NED.N, 2) + pow(P.E - nin->children[i]->NED.E, 2) + pow(P.D - nin->children[i]->NED.D, 2));	// Calculate the distance to the node
			if (distance < *minD)											// If we found a better distance, update it
			{
				minNode = nin->children[i];									// reset the minNode
				*minD = distance;											// reset the minimum distance
			}
			minNode = find_closest_node(nin->children[i], P, minNode, minD);// Recursion for each child
		}
		return minNode;														// Return the closest node
	}
	void delete_tree();														// Delete the entire tree
	void delete_node(node*);												// Recursively delete the nodes
	std::vector<node*> root_ptrs;												// Vector of all roots, each element is the start of the tree to reach the next primary waypoint
	node *closest_node;														// This is a variable that is used to find the closest node - if it is in here there are no memory leaks.
	node *closest_node_gchild;												// This is a variable that is the closest node off of a grandchild tree.
	bool check_fillet(NED_s par, NED_s mid, NED_s nex, double avail_dis, double* din, double* cangle, NED_s* line_start);	// Check to see if the fillet connecting lines clears the obstacles and boundary lines.
	bool check_create_fan(NED_s primary_wp, NED_s coming_from, node* next_root);	// This function checks to see if a fan can be created, and it also creates it.
	bool check_direct_fan(NED_s second_wp, NED_s primary_wp, NED_s coming_from, node* next_root, NED_s* cea_out, double* din, double* anglin);	// Check and create a direct round to straight path.
	bool check_slope(NED_s beg, NED_s en);
	bool check_max_slope(NED_s beg, NED_s en);
	void initialize_tree();
	bool direct_connection(unsigned int i, NED_s* second2last_post_smoothed, double* distance_in, double* fillet_angle, bool* direct_shot);// This function checks to see if there is a direct connection to the next waypoint
	void develop_tree(unsigned int i, bool reached_next_wp, node* second2last, NED_s* second2last_post_smoothed, double* distance_in, double* fillet_angle); // Develops the tree
	void smoother(bool skip_smoother, unsigned int i, double* distance_in, double* fillet_angle, NED_s* second2last_post_smoothed, bool direct_shot);
	void calc_path_distance(unsigned int i);

  bool lineAndPoint2d(NED_s ls, NED_s le, double MinMax[], double Mandb[], NED_s p, double r);	// Function called by the LINE flyZoneCheck(NED,NED,radius)
	bool line_intersects_arc(double Ni, double Ei, NED_s cp, NED_s ps, NED_s pe, bool ccw);			// This function finds if the line intersects the arc or not.
  ParamReader *input_file;								// address of the input file
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
