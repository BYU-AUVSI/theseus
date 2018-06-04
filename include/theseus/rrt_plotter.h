#ifndef RRT_PLOTTTER
#define RRT_PLOTTTER

#include <vector>
#include <math.h>
#include <ros/ros.h>

#include <theseus/map_s.h>
#include <theseus/fillet_s.h>
#include <theseus/node_s.h>
#include <theseus/param_reader.h>

#include <visualization_msgs/Marker.h>

namespace theseus
{
struct rrtColors
{
  NED_s gray;
  NED_s blue;
  NED_s green;
  NED_s orange;
  NED_s purple;
  NED_s red;
  rrtColors()
  {
    gray.N = 0.5f;   gray.E = 0.5f;    gray.D = 0.5f;
    blue.N = 0.0f;   blue.E = 1.0f;    blue.D = 1.0f;
    green.N = 0.0f;  green.E = 1.0f;   green.D = 0.0f;
    orange.N = 1.0f; orange.E = 0.55f; orange.D = 0.0f;
    purple.N = 0.5f; purple.E = 0.0f;  purple.D = 0.5f;
    red.N = 1.0f;    red.E = 0.0f;     red.D = 0.0f;
  }
};
class rrtPlotter
{
public:
	// Functions
  rrtPlotter();
	~rrtPlotter();

  bool increase_path_id_; // default true
  void displayMap(map_s map);
  void displayPrimaryWaypoints(std::vector<NED_s> wps);
  void odomCallback(geometry_msgs::Point p);
  void mobsCallback(std::vector<NED_s> mobs_in, std::vector<float> radius);
  void displayPath(std::vector<node*> path, NED_s color, float width);
  void displayPath(std::vector<NED_s> path, NED_s color, float width);
  void displayPath(NED_s ps, std::vector<NED_s> path, NED_s color, float width);
  void displayPath(NED_s ps, std::vector<node*> path, NED_s color, float width);
  void drawCircle(NED_s cp, float r);
  void clearRViz(map_s map);
  void clearRViz(map_s map, std::vector<NED_s> path, NED_s color, float width);
  void displayTree(node* root);
  void displayBoundaries(map_s map);
  void pingBoundaries();
  void pingPath();
  void addFinalPath(NED_s ps, std::vector<NED_s> stuff_in);

private:
  ros::NodeHandle nh_;
  ros::Publisher marker_pub_;
  visualization_msgs::Marker odom_mkr_;
  visualization_msgs::Marker mobs_mkr_;
  visualization_msgs::Marker planned_path_mkr_;
  int path_id_;
  int pWPS_id_;
  ParamReader input_file_;
  std::vector<node*> fringe_;
  std::vector<NED_s> tree_path_;
  rrtColors clr;
  visualization_msgs::Marker bds_mkr_;

  std::vector<std::vector<float > > arc(float N, float E, float r, float aS, float aE);
  void addFringe(node* nin);
  void addTreePath(node* root, node* nin);

};
}
#endif
