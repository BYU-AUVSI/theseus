#include <theseus/ros_path_planner.h>
#include <theseus/RRT.h>

namespace theseus
{
RosPathPlanner::RosPathPlanner() :
  nh_(ros::NodeHandle())
{
  //********************** PARAMETERS **********************//

  //************** SUBSCRIBERS AND PUBLISHERS **************//
  waypoint_publisher_    = nh_.advertise<rosplane_msgs::Waypoint>("waypoint_path", 1);
  path_solver_service_   = nh_.advertiseService("solve_static",&theseus::RosPathPlanner::solve_static, this);
  mission_map_publisher_ = nh_.advertise<theseus::AuvsiMap>("/auvsi_map",1);
  ROS_INFO("RosPathPlanner Constructor");
  //******************** CLASS VARIABLES *******************//

  //***************** CALLBACKS AND TIMERS *****************//

  //********************** FUNCTIONS ***********************//

}
RosPathPlanner::~RosPathPlanner()
{

}
bool RosPathPlanner::solve_static(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res)
{
  ROS_WARN("CHILD FUNCTION WAS NOT CALLED");
  res.success = false;
  return true;
}
} // end namespace rosplane

//********************************************************//
//************************ MAIN **************************//
//********************************************************//
int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_planner");
  theseus::ParamReader input_file;
  theseus::RandGen rg(input_file.seed);
  theseus::RRT_input rrt_i;
  theseus::mapper myWorld(rg.UINT(), &input_file);
  theseus::RosPathPlanner *ros_path_planner_obj;
  ros_path_planner_obj = new theseus::RRT(myWorld.map, input_file.seed, &input_file, rrt_i);

  ros::spin();

  return 0;
} // end main
