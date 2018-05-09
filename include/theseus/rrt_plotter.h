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
  rrtColors()
  {
    gray.N = 0.5f;   gray.E = 0.5f;    gray.D = 0.5f;
    blue.N = 0.0f;   blue.E = 1.0f;    blue.D = 1.0f;
    green.N = 0.0f;  green.E = 1.0f;   green.D = 0.0f;
    orange.N = 1.0f; orange.E = 0.55f; orange.D = 0.0f;
  }
};
class rrtPlotter
{
public:
	// Functions
  rrtPlotter();
	~rrtPlotter();

  void displayMap(map_s map);
  void odomCallback(geometry_msgs::Point p);
  void displayPath(std::vector<node*> path, NED_s color, float width);
  void displayPath(std::vector<NED_s> path, NED_s color, float width);
  void displayPath(NED_s ps, std::vector<NED_s> path, NED_s color, float width);
  void displayPath(NED_s ps, std::vector<node*> path, NED_s color, float width);
  void drawCircle(NED_s cp, float r);
  void clearRViz(map_s map);
  void displayTree(node* root);

private:
  ros::NodeHandle nh_;
  ros::Publisher marker_pub_;
  visualization_msgs::Marker odom_mkr_;
  int path_id_;
  ParamReader input_file_;
  std::vector<node*> fringe_;
  std::vector<NED_s> tree_path_;
  rrtColors clr;

  std::vector<std::vector<float > > arc(float N, float E, float r, float aS, float aE);
  void addFringe(node* nin);
  void addTreePath(node* root, node* nin);

};
}
#endif
