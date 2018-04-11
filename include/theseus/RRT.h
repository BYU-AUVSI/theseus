#ifndef RRT_H
#define RRT_H

#include <stack>
#include <vector>
#include <algorithm>
#include <math.h>
#include <ros/ros.h>

#include <theseus/map_s.h>
#include <theseus/fillet_s.h>
#include <theseus/rand_gen.h>
#include <theseus/param_reader.h>

namespace theseus
{
// Class Definition
class RRT
{
public:
  RRT(map_s map_in, unsigned int seed);
	RRT();
  ~RRT();                                                   // Deconstructor - deletes the tree
	void solveStatic(NED_s pos, float chi0, bool direct_hit); // Solves the static path
  void newMap(map_s map_in);                                // creates a new map
  void newSeed(unsigned int seed);
  std::vector<std::vector<NED_s> > all_wps_;                // final path waypoints

private:
  // core functions
  bool tryDirectConnect(node* ps, node* pe);
  int  developTree(unsigned int i);
  std::vector<node*> findMinimumPath(unsigned int i);
  std::vector<node*> smoothPath(unsigned int i);

  // secondary functions


  // Initialize and clear data functions
  void initializeTree(NED_s pos, float chi0);
  void clearForNewPath();
  void clearForNewMap();
  void deleteTree();                 // Delete the entire tree
	void deleteNode(node*);            // Recursively delete the nodes
  void clearTree();                  // Clear the entire tree
	void clearNode(node*);             // Recursively clear the nodes

  // Printing Functions
  void printRRTSetup(NED_s pos, float chi0); // used for debugging
  void printRoots();                         // prints all of the root nodes

  bool checkFillet(NED_s par, NED_s mid, NED_s nex, double avail_dis, double* din, double* cangle, NED_s* line_start);	// Check to see if the fillet connecting lines clears the obstacles and boundary lines.
	bool checkCreateFan(NED_s primary_wp, NED_s coming_from, node* next_root, bool direct_hit); // This function checks to see if a fan can be created, and it also creates it.
	bool checkDirectFan(NED_s second_wp, NED_s primary_wp, NED_s coming_from, node* next_root, NED_s* cea_out, double* din, double* anglin); // Check and create a direct round to straight path.
	bool directConnection(unsigned int i, NED_s* second2last_post_smoothed, double* distance_in, double* fillet_angle, bool* direct_shot, bool direct_hit);  // This function checks to see if there is a direct connection to the next waypoint
	void developTree(unsigned int i, bool reached_next_wp, node* second2last, NED_s* second2last_post_smoothed, double* distance_in, double* fillet_angle, bool direct_hit); // Develops the tree
	void smoother(bool skip_smoother, unsigned int i, double* distance_in, double* fillet_angle, NED_s* second2last_post_smoothed, bool direct_shot, bool direct_hit);



  float segment_length_;         // If used, this is the distance the algorithm uses between each node
  ParamReader input_file_;       // address of the input file
  CollisionDetection col_det_;   // collision detecter

  RandGen rg_;                   // Here is the random generator for the algorithm
  std::vector<node*> root_ptrs_; // Vector of all roots, each element is the start of the tree to reach the next primary waypoint

	float clearance_;              // The minimum clearance that the path will have away from any obstacles (can fluctuate up and down)
	float path_clearance_;         // The minimum clearance that the path from waypoint i to i+1 will have. (only decreases until new waypoint is obtained)
	unsigned int iters_limit_;     // This is the maximum number of iterations the solver will do before it will just move on
	bool taking_off_;              // If the plane is currently taking off, this option will allow the path planner to ignor the height restricitons.
  bool direct_hit_;              // when true the algorithm will hit primary waypoints dead on instead of filleting


	struct node                    // This is the node struct for each spot on the tree
	{
		NED_s p;                     // North, East Down of the node position
    fillet_s fil;                // fillet information for the PARENT
		std::vector<node*> children; // Vector of all of the children nodes
		node* parent;                // Pointer to the parent of this node
		float cost;                  // Distance from this node to its parent
    bool dontConnect;            // true means closest nodes generated won't connect to this node
    bool connects2wp;            // true if this node connects to the next waypoint
	};
	node* findClosestNode(node* nin, NED_s P, node* minNode, float* minD)// This recursive function return the closes node to the input point P, for some reason it wouldn't go in the cpp...
	{// nin is the node to measure, P is the point, minNode is the closes found node so far, minD is where to store the minimum distance
		// Recursion
		float distance;                                         // distance to the point P
		for (unsigned int i = 0; i < nin->children.size(); i++) // For all of the children figure out their distances
		{
      distance = (P - nin->children[i].p).norm();
			if (distance < *minD)          // If we found a better distance, update it
			{
				minNode = nin->children[i];  // reset the minNode
				*minD = distance;            // reset the minimum distance
			}
			minNode = findClosestNode(nin->children[i], P, minNode, minD); // Recursion for each child
		}
		return minNode;                  // Return the closest node
	}
};
}
#endif
