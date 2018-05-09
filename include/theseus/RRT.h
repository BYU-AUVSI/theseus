#ifndef RRT_H
#define RRT_H

#include <stack>
#include <vector>
#include <algorithm>
#include <math.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/Marker.h>

#include <theseus/map_s.h>
#include <theseus/fillet_s.h>
#include <theseus/rand_gen.h>
#include <theseus/param_reader.h>
#include <theseus/collision_detection.h>
#include <theseus/rrt_plotter.h>
#include <theseus/node_s.h>

namespace theseus
{
// Class Definition
class RRT
{
public:
  RRT(map_s map_in, unsigned int seed);
	RRT();
  ~RRT();                                                                 // Deconstructor - deletes the tree
	void solveStatic(NED_s pos, float chi0, bool direct_hit, bool landing); // Solves the static path
  void newMap(map_s map_in);                                              // creates a new map
  void newSeed(unsigned int seed);
  bool checkPoint(NED_s point, float clearance);
  std::vector<NED_s> all_wps_;                // final path waypoints
  map_s map_;
  bool landing_now_;
  NED_s ending_point_;
  float ending_chi_;
private:
  ros::NodeHandle nh_;         // public node handle for publishing, subscribing
  ros::Publisher marker_pub_;
  // core functions
  bool tryDirectConnect(node* ps, node* pe, unsigned int i);
  int  developTree(unsigned int i);
  std::vector<node*> findMinimumPath(unsigned int i);
  std::vector<node*> smoothPath(std::vector<node*> rough_path, int i);
  void addPath(std::vector<node*> smooth_path, unsigned int i);
  NED_s findLoiterSpot(NED_s cp, float radius);

  // secondary functions
  void resetParent(node* nin, node* new_parent);
  node* findClosestNodeGChild(node* root, NED_s p);
  bool checkForCollision(node* ps, NED_s pe, unsigned int i, float clearance, bool connecting_to_end);
  NED_s randomPoint(unsigned int i);
  node* findClosestNode(node* nin, NED_s P, node* minNode, float* minD);
  node* findMinConnector(node* nin, node* minNode, float* minCost);
  void createFan(node* root, NED_s p, float chi, float clearance);
  float redoRandomDownPoint(unsigned int i, float closest_D);
  bool checkWholePath(node* snode, std::vector<node*> rough_path, int ptr, int i);
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
  void printNode(node* nin);                 // prints the node
  void printFillet(fillet_s fil);
  rrtPlotter plt;
  rrtColors clr;

  float segment_length_;          // If used, this is the distance the algorithm uses between each node
  ParamReader input_file_;        // address of the input file
  CollisionDetection col_det_;    // collision detecter
  int num_paths_;                 // number of paths to be genererated before choosing the optimal path
  RandGen rg_;                    // Here is the random generator for the algorithm
  std::vector<node*> root_ptrs_;  // Vector of all roots, each element is the start of the tree to reach the next primary waypoint
  std::vector<node*> smooth_rts_; // Vector of all roots, each element is the start of the tree to reach the nect primary waypoint

	float clearance_;               // The minimum clearance that the path will have away from any obstacles (can fluctuate up and down)
	float path_clearance_;          // The minimum clearance that the path from waypoint i to i+1 will have. (only decreases until new waypoint is obtained)
	bool taking_off_;               // If the plane is currently taking off, this option will allow the path planner to ignor the height restricitons.
  bool direct_hit_;               // when true the algorithm will hit primary waypoints dead on instead of filleting
  node* most_recent_node_;        // pointer to the most recently added node
};
}
#endif
