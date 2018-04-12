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
#include <theseus/collision_detection.h>

namespace theseus
{
// Class Definition
struct node                    // This is the node struct for each spot on the tree
{
  NED_s p;                     // North, East Down of the node position
  fillet_s fil;                // *fillet information for the PARENT
  std::vector<node*> children; // Vector of all of the children nodes
  node* parent;                // *Pointer to the parent of this node
  float cost;                  // Distance from this node to its parent
  bool dontConnect;            // true means closest nodes generated won't connect to this node
  bool connects2wp;            // true if this node connects to the next waypoint
};
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
  map_s map_;
private:
  // core functions
  bool tryDirectConnect(node* ps, node* pe, unsigned int i);
  int  developTree(unsigned int i);
  std::vector<node*> findMinimumPath(unsigned int i);
  std::vector<node*> smoothPath(std::vector<node*> rough_path);
  void addPath(std::vector<node*> smooth_path);

  // secondary functions
  node* findClosestNodeGChild(node* root, NED_s p);
  bool checkForCollision(node* ps, NED_s pe, unsigned int i, float clearance);
  NED_s randomPoint(unsigned int i);
  node* findClosestNode(node* nin, NED_s P, node* minNode, float* minD);
  node* findMinConnector(node* nin, node* minNode, float* minCost);


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

  float segment_length_;         // If used, this is the distance the algorithm uses between each node
  ParamReader input_file_;       // address of the input file
  CollisionDetection col_det_;   // collision detecter
  int num_paths_;                // number of paths to be genererated before choosing the optimal path
  RandGen rg_;                   // Here is the random generator for the algorithm
  std::vector<node*> root_ptrs_; // Vector of all roots, each element is the start of the tree to reach the next primary waypoint

	float clearance_;              // The minimum clearance that the path will have away from any obstacles (can fluctuate up and down)
	float path_clearance_;         // The minimum clearance that the path from waypoint i to i+1 will have. (only decreases until new waypoint is obtained)
	unsigned int iters_limit_;     // This is the maximum number of iterations the solver will do before it will just move on
	bool taking_off_;              // If the plane is currently taking off, this option will allow the path planner to ignor the height restricitons.
  bool direct_hit_;              // when true the algorithm will hit primary waypoints dead on instead of filleting
  node* most_recent_node_;       // pointer to the most recently added node
};
}
#endif
