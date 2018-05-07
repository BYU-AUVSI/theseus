#include <theseus/ros_path_planner.h>

namespace theseus
{
RosPathPlanner::RosPathPlanner() :
  nh_(ros::NodeHandle())
{
  //********************** PARAMETERS **********************//

  //************** SUBSCRIBERS AND PUBLISHERS **************//
  recieved_state_         = false;
  has_map_                = false;
  state_subscriber_       = nh_.subscribe("/state",100,&theseus::RosPathPlanner::stateCallback, this);
  waypoint_publisher_     = nh_.advertise<rosplane_msgs::Waypoint>("/waypoint_path", 1);

  path_solver_service1_   = nh_.advertiseService("solve_static",&theseus::RosPathPlanner::solveStatic, this);
  path_solver_service2_   = nh_.advertiseService("add_wps",&theseus::RosPathPlanner::addWps, this);
  path_solver_service3_   = nh_.advertiseService("add_landing",&theseus::RosPathPlanner::addLanding, this);
  path_solver_service4_   = nh_.advertiseService("add_textfile",&theseus::RosPathPlanner::addTextfile, this);
  path_solver_service5_   = nh_.advertiseService("land_now",&theseus::RosPathPlanner::landNow, this);
  path_solver_service6_   = nh_.advertiseService("textfile_now",&theseus::RosPathPlanner::textfileNow, this);



  new_map_service_        = nh_.advertiseService("new_random_map",&theseus::RosPathPlanner::newRandomMap, this);
  plan_mission_service_   = nh_.advertiseService("plan_mission",&theseus::RosPathPlanner::planMission, this);
  send_wps_service_       = nh_.advertiseService("send_waypoints",&theseus::RosPathPlanner::sendWaypoints, this);
  replot_map_service_     = nh_.advertiseService("replot_map",&theseus::RosPathPlanner::displayMapService, this);
  wp_distance_service_    = nh_.advertiseService("display_wp_distance",&theseus::RosPathPlanner::displayD2WP, this);

  //******************** CLASS VARIABLES *******************//
  RandGen rg_in(input_file_.seed);
	rg_                          = rg_in;												    // Copy that random generator into the class.

  //***************** CALLBACKS AND TIMERS *****************//
  update_viz_timer_ = nh_.createWallTimer(ros::WallDuration(1.0/4.0), &RosPathPlanner::updateViz, this);

  //********************** FUNCTIONS ***********************//
  RRT rrt_obj(myWorld_, input_file_.seed);
  rrt_obj_ = rrt_obj;

  bool testing;
  nh_.param<bool>("testing/init_references", testing, false);
  if (testing)
  {
    double lat_ref, lon_ref, h_ref;
    float N_init, E_init, D_init;
    nh_.param<double>("testing/lat_ref", lat_ref, 38.14326388888889);
    nh_.param<double>("testing/lon_ref", lon_ref, -76.43075);
    nh_.param<double>("testing/h_ref", h_ref, 6.701);
    nh_.param<float>("testing/N_init", N_init, 0.0);
    nh_.param<float>("testing/E_init", E_init, 0.0);
    nh_.param<float>("testing/D_init", D_init, 0.0);
    ROS_WARN("testing = true, initializing reference and initial position");
    gps_converter_.set_reference(lat_ref, lon_ref, h_ref);
    recieved_state_ = true;
    odometry_[1]    = N_init;
    odometry_[0]    = E_init;
    odometry_[2]    = -D_init;
    chi0_           = 0.0f;
  }
}
RosPathPlanner::~RosPathPlanner()
{

}
bool RosPathPlanner::planMission(uav_msgs::GeneratePath::Request &req, uav_msgs::GeneratePath::Response &res)
{
  if (recieved_state_ == false)
  {
    ROS_ERROR("PATH PLANNER HAS NOT RECIEVED AN INITIAL STATE");
    return false;
  }
  int num_waypoints  = req.mission.waypoints.size();
  int num_boundaries = req.mission.boundaries.size();
  int num_obstacles  = req.mission.stationary_obstacles.size();

  map_s mission_map;
  double lat, lon, alt, N, E, D, cyl_h, cyl_r;
  NED_s ned;
  cyl_s cyl;
  for (int i = 0; i < num_waypoints; i++)
  {
    lat = req.mission.waypoints[i].point.latitude;
    lon = req.mission.waypoints[i].point.longitude;
    alt = req.mission.waypoints[i].point.altitude;
    gps_converter_.gps2ned(lat, lon, alt, N, E, D);
    ned.N = N;
    ned.E = E;
    ned.D = D;
    mission_map.wps.push_back(ned);
  }
  for (int i = 0; i < num_boundaries; i++)
  {
    lat = req.mission.boundaries[i].point.latitude;
    lon = req.mission.boundaries[i].point.longitude;
    alt = req.mission.boundaries[i].point.altitude;
    gps_converter_.gps2ned(lat, lon, alt, N, E, D);
    ned.N = N;
    ned.E = E;
    ned.D = D;
    mission_map.boundary_pts.push_back(ned);
  }
  for (int i = 0; i < num_obstacles; i++)
  {
    lat   = req.mission.stationary_obstacles[i].point.latitude;
    lon   = req.mission.stationary_obstacles[i].point.longitude;
    alt   = req.mission.stationary_obstacles[i].point.altitude;
    cyl_h = req.mission.stationary_obstacles[i].cylinder_height;
    cyl_r = req.mission.stationary_obstacles[i].cylinder_radius;
    gps_converter_.gps2ned(lat, lon, alt, N, E, D);
    cyl.N = N;
    cyl.E = E;
    cyl.R = cyl_r;
    cyl.H = cyl_h;
    mission_map.cylinders.push_back(cyl);
  }
  bool direct_hit;
  if (req.mission.mission_type == req.mission.MISSION_TYPE_WAYPOINT || req.mission.mission_type == req.mission.MISSION_TYPE_LAND)
    direct_hit = true;
  else
    direct_hit = false;
  double chi;
  bool landing = false;
  if (req.mission.mission_type == req.mission.MISSION_TYPE_LAND)
  {
    chi = req.mission.waypoints[0].point.chi;
    landing = true;
    num_waypoints++;
    float landing_strip = 400.0;
    ned.N = mission_map.wps.back().N + landing_strip*cosf(chi);
    ned.E = mission_map.wps.back().E + landing_strip*sinf(chi);
    ned.D = 0.0f;
    mission_map.wps.push_back(ned);
    num_waypoints++;
    landing_strip = 400.0;
    ned.N = mission_map.wps.back().N + landing_strip*cosf(chi);
    ned.E = mission_map.wps.back().E + landing_strip*sinf(chi);
    ned.D = 0.0f;
    mission_map.wps.push_back(ned);
  }

  myWorld_ = mission_map;
  rrt_obj_.newMap(mission_map);
  has_map_ = true;
  wp_distances_.clear();
  cyl_distances_.clear();
  for (int i = 0; i < rrt_obj_.map_.wps.size(); i++)
    wp_distances_.push_back(INFINITY);
  for (int i = 0; i < rrt_obj_.map_.cylinders.size(); i++)
    cyl_distances_.push_back(INFINITY);
  min_cyl_dis_ = INFINITY;
  ROS_INFO("RECIEVED JUDGES MAP");
  NED_s pos;
  pos.N =  odometry_[1];
  pos.E =  odometry_[0];
  pos.D = -odometry_[2];
  plt.displayMap(myWorld_);
  rrt_obj_.solveStatic(pos, chi0_, true, landing);
  plt.displayPath(pos, rrt_obj_.all_wps_, clr.green, 15.0);
  return true;
}
bool RosPathPlanner::newRandomMap(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  mapper myWorld(rg_.UINT(), &input_file_);
  myWorld_ = myWorld.map;
  rrt_obj_.newMap(myWorld_);
  has_map_ = true;
  wp_distances_.clear();
  cyl_distances_.clear();
  for (int i = 0; i < rrt_obj_.map_.wps.size(); i++)
    wp_distances_.push_back(INFINITY);
  for (int i = 0; i < rrt_obj_.map_.cylinders.size(); i++)
    cyl_distances_.push_back(INFINITY);
    min_cyl_dis_ = INFINITY;
  ROS_INFO("RECIEVED NEW RANDOM MAP");
  plt.clearRViz(myWorld_);
  res.success = true;
  return true;
}
bool RosPathPlanner::solveStatic(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res)
{
  if (recieved_state_ == false)
  {
    ROS_ERROR("PATH PLANNER HAS NOT RECIEVED AN INITIAL STATE");
    res.success = false;
    return false;
  }
  if (has_map_ == false)
  {
    ROS_ERROR("PATH PLANNER HAS NO MAP.");
    ROS_ERROR("Generating random map.");
    mapper myWorld(rg_.UINT(), &input_file_);
    myWorld_ = myWorld.map;
    rrt_obj_.newMap(myWorld_);
    has_map_ = true;
    wp_distances_.clear();
    cyl_distances_.clear();
    for (int i = 0; i < rrt_obj_.map_.wps.size(); i++)
      wp_distances_.push_back(INFINITY);
    for (int i = 0; i < rrt_obj_.map_.cylinders.size(); i++)
      cyl_distances_.push_back(INFINITY);
    min_cyl_dis_ = INFINITY;
  }
  NED_s pos;
  pos.N =  odometry_[1];
  pos.E =  odometry_[0];
  pos.D = -odometry_[2];
  bool direct_hit = true;
  bool landing = false;
  plt.displayMap(myWorld_);
  rrt_obj_.solveStatic(pos, chi0_, direct_hit, landing);
  plt.displayPath(pos, rrt_obj_.all_wps_, clr.green, 15.0);
  res.success = true;
  return true;
}
bool RosPathPlanner::addWps(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res)
{
  if (recieved_state_ == false)
  {
    ROS_ERROR("PATH PLANNER HAS NOT RECIEVED AN INITIAL STATE");
    res.success = false;
    return false;
  }
  if (has_map_ == false)
  {
    ROS_ERROR("PATH PLANNER HAS NO MAP.");
    ROS_ERROR("Generating random map.");
    mapper myWorld(rg_.UINT(), &input_file_);
    myWorld_ = myWorld.map;
    rrt_obj_.newMap(myWorld_);
    has_map_ = true;
    wp_distances_.clear();
    cyl_distances_.clear();
    for (int i = 0; i < rrt_obj_.map_.wps.size(); i++)
      wp_distances_.push_back(INFINITY);
    for (int i = 0; i < rrt_obj_.map_.cylinders.size(); i++)
      cyl_distances_.push_back(INFINITY);
    min_cyl_dis_ = INFINITY;
  }
  plt.displayMap(myWorld_);
  bool direct_hit = true;
  bool landing = false;
  NED_s pos;
  pos = rrt_obj_.ending_point_;
  rrt_obj_.solveStatic(pos, rrt_obj_.ending_chi_, direct_hit, landing);
  plt.displayPath(pos, rrt_obj_.all_wps_, clr.green, 15.0);
  res.success = true;
  return true;
}
bool RosPathPlanner::addLanding(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res)
{
  if (recieved_state_ == false)
  {
    ROS_ERROR("PATH PLANNER HAS NOT RECIEVED AN INITIAL STATE");
    res.success = false;
    return false;
  }
  if (has_map_ == false)
  {
    ROS_ERROR("PATH PLANNER HAS NO MAP.");
    ROS_ERROR("Generating random map.");
    mapper myWorld(rg_.UINT(), &input_file_);
    myWorld_ = myWorld.map;
    rrt_obj_.newMap(myWorld_);
    has_map_ = true;
    wp_distances_.clear();
    cyl_distances_.clear();
    for (int i = 0; i < rrt_obj_.map_.wps.size(); i++)
      wp_distances_.push_back(INFINITY);
    for (int i = 0; i < rrt_obj_.map_.cylinders.size(); i++)
      cyl_distances_.push_back(INFINITY);
    min_cyl_dis_ = INFINITY;
  }
  std::fstream fin;
  fin.open("landing.txt", std::ifstream::in);
  if (!(fin.is_open()))
  {
    ROS_ERROR("WAYPOINTS FILE DID NOT OPEN."); // try putting it in the ~/.ros directory.
    ros::shutdown();
  }
  float N, E, D, chi;
  fin >> N >> E >> D >> chi;
  fin.close();
  bool direct_hit = true;
  bool landing = true;
  rrt_obj_.map_.wps.clear();
  NED_s descend_point;
  descend_point.N = N;
  descend_point.E = E;
  descend_point.D = D;
  rrt_obj_.map_.wps.push_back(descend_point);
  descend_point.N = N;
  descend_point.E = E;
  descend_point.D = D;
  rrt_obj_.map_.wps.push_back(descend_point);
  float landing_strip = 400.0;
  NED_s ned;
  ned.N = rrt_obj_.map_.wps.back().N + landing_strip*cosf(chi);
  ned.E = rrt_obj_.map_.wps.back().E + landing_strip*sinf(chi);
  ned.D = 0.0f;
  rrt_obj_.map_.wps.push_back(ned);
  ned.N = rrt_obj_.map_.wps.back().N + landing_strip*cosf(chi);
  ned.E = rrt_obj_.map_.wps.back().E + landing_strip*sinf(chi);
  ned.D = 0.0f;
  rrt_obj_.map_.wps.push_back(ned);
  myWorld_ = rrt_obj_.map_;
  plt.displayMap(myWorld_);
  ROS_INFO("ENDING POINT N: %f, E: %f, D: %f, chi: %f", rrt_obj_.ending_point_.N, rrt_obj_.ending_point_.E, rrt_obj_.ending_point_.D, rrt_obj_.ending_chi_);
  NED_s pos;
  pos = rrt_obj_.ending_point_;
  rrt_obj_.solveStatic(pos, rrt_obj_.ending_chi_, direct_hit, landing);
  plt.displayPath(pos, rrt_obj_.all_wps_, clr.green, 15.0);
  res.success = true;
  return true;
}
bool RosPathPlanner::addTextfile(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res)
{
  if (recieved_state_ == false)
  {
    ROS_ERROR("PATH PLANNER HAS NOT RECIEVED AN INITIAL STATE");
    res.success = false;
    return false;
  }
  if (has_map_ == false)
  {
    ROS_ERROR("PATH PLANNER HAS NO MAP.");
    ROS_ERROR("Generating random map.");
    mapper myWorld(rg_.UINT(), &input_file_);
    myWorld_ = myWorld.map;
    rrt_obj_.newMap(myWorld_);
    has_map_ = true;
    wp_distances_.clear();
    cyl_distances_.clear();
    for (int i = 0; i < rrt_obj_.map_.wps.size(); i++)
      wp_distances_.push_back(INFINITY);
    for (int i = 0; i < rrt_obj_.map_.cylinders.size(); i++)
      cyl_distances_.push_back(INFINITY);
    min_cyl_dis_ = INFINITY;
  }
  std::fstream fin;
  fin.open("path.txt", std::ifstream::in);
  if (!(fin.is_open()))
  {
    ROS_ERROR("WAYPOINTS FILE DID NOT OPEN."); // try putting it in the ~/.ros directory.
    ros::shutdown();
  }
  NED_s wp;
  myWorld_.wps.clear();
  while (fin.eof() == false)
  {
    fin >> wp.N >> wp.E >> wp.D;
    myWorld_.wps.push_back(wp);
  }
  fin.close();
  rrt_obj_.newMap(myWorld_);
  bool direct_hit = false;
  bool landing = false;
  plt.displayMap(myWorld_);
  NED_s pos;
  pos = rrt_obj_.ending_point_;
  rrt_obj_.solveStatic(pos, rrt_obj_.ending_chi_, direct_hit, landing);
  plt.displayPath(pos, rrt_obj_.all_wps_, clr.green, 15.0);
  res.success = true;
  return true;
}
bool RosPathPlanner::landNow(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res)
{
  if (recieved_state_ == false)
  {
    ROS_ERROR("PATH PLANNER HAS NOT RECIEVED AN INITIAL STATE");
    res.success = false;
    return false;
  }
  if (has_map_ == false)
  {
    ROS_ERROR("PATH PLANNER HAS NO MAP.");
    ROS_ERROR("Generating random map.");
    mapper myWorld(rg_.UINT(), &input_file_);
    myWorld_ = myWorld.map;
    rrt_obj_.newMap(myWorld_);
    has_map_ = true;
    wp_distances_.clear();
    cyl_distances_.clear();
    for (int i = 0; i < rrt_obj_.map_.wps.size(); i++)
      wp_distances_.push_back(INFINITY);
    for (int i = 0; i < rrt_obj_.map_.cylinders.size(); i++)
      cyl_distances_.push_back(INFINITY);
    min_cyl_dis_ = INFINITY;
  }
  std::fstream fin;
  fin.open("landing.txt", std::ifstream::in);
  if (!(fin.is_open()))
  {
    ROS_ERROR("WAYPOINTS FILE DID NOT OPEN."); // try putting it in the ~/.ros directory.
    ros::shutdown();
  }
  float N, E, D, chi;
  fin >> N >> E >> D >> chi;
  fin.close();
  NED_s pos;
  pos.N =  odometry_[1];
  pos.E =  odometry_[0];
  pos.D = -odometry_[2];
  bool direct_hit = true;
  bool landing = true;
  rrt_obj_.map_.wps.clear();
  NED_s descend_point;
  descend_point.N = N;
  descend_point.E = E;
  descend_point.D = D;
  rrt_obj_.map_.wps.push_back(descend_point);
  descend_point.N = N;
  descend_point.E = E;
  descend_point.D = D;
  rrt_obj_.map_.wps.push_back(descend_point);
  float landing_strip = 400.0;
  NED_s ned;
  ned.N = rrt_obj_.map_.wps.back().N + landing_strip*cosf(chi);
  ned.E = rrt_obj_.map_.wps.back().E + landing_strip*sinf(chi);
  ned.D = 0.0f;
  rrt_obj_.map_.wps.push_back(ned);
  ned.N = rrt_obj_.map_.wps.back().N + landing_strip*cosf(chi);
  ned.E = rrt_obj_.map_.wps.back().E + landing_strip*sinf(chi);
  ned.D = 0.0f;
  rrt_obj_.map_.wps.push_back(ned);
  myWorld_ = rrt_obj_.map_;
  plt.displayMap(myWorld_);
  rrt_obj_.solveStatic(pos, chi0_, direct_hit, landing);
  plt.displayPath(pos, rrt_obj_.all_wps_, clr.green, 15.0);
  res.success = true;
  return true;
}
bool RosPathPlanner::textfileNow(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res)
{
  if (recieved_state_ == false)
  {
    ROS_ERROR("PATH PLANNER HAS NOT RECIEVED AN INITIAL STATE");
    res.success = false;
    return false;
  }
  if (has_map_ == false)
  {
    ROS_ERROR("PATH PLANNER HAS NO MAP.");
    ROS_ERROR("Generating random map.");
    mapper myWorld(rg_.UINT(), &input_file_);
    myWorld_ = myWorld.map;
    rrt_obj_.newMap(myWorld_);
    has_map_ = true;
    wp_distances_.clear();
    cyl_distances_.clear();
    for (int i = 0; i < rrt_obj_.map_.wps.size(); i++)
      wp_distances_.push_back(INFINITY);
    for (int i = 0; i < rrt_obj_.map_.cylinders.size(); i++)
      cyl_distances_.push_back(INFINITY);
    min_cyl_dis_ = INFINITY;
  }
  std::fstream fin;
  fin.open("path.txt", std::ifstream::in);
  if (!(fin.is_open()))
  {
    ROS_ERROR("WAYPOINTS FILE DID NOT OPEN."); // try putting it in the ~/.ros directory.
    ros::shutdown();
  }
  NED_s wp;
  myWorld_.wps.clear();
  while (fin.eof() == false)
  {
    fin >> wp.N >> wp.E >> wp.D;
    myWorld_.wps.push_back(wp);
  }
  fin.close();
  NED_s pos;
  pos.N =  odometry_[1];
  pos.E =  odometry_[0];
  pos.D = -odometry_[2];
  rrt_obj_.newMap(myWorld_);
  bool direct_hit = false;
  bool landing = false;
  plt.displayMap(myWorld_);
  rrt_obj_.solveStatic(pos, chi0_, direct_hit, landing);
  plt.displayPath(pos, rrt_obj_.all_wps_, clr.green, 15.0);
  res.success = true;
  return true;
}
bool RosPathPlanner::displayMapService(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res)
{
  if (has_map_ == false)
  {
    ROS_ERROR("PATH PLANNER HAS NO MAP.");
    ROS_ERROR("Generating random map.");
    mapper myWorld(rg_.UINT(), &input_file_);
    myWorld_ = myWorld.map;
    rrt_obj_.newMap(myWorld_);
    has_map_ = true;
    wp_distances_.clear();
    cyl_distances_.clear();
    for (int i = 0; i < rrt_obj_.map_.wps.size(); i++)
      wp_distances_.push_back(INFINITY);
    for (int i = 0; i < rrt_obj_.map_.cylinders.size(); i++)
      cyl_distances_.push_back(INFINITY);
    min_cyl_dis_ = INFINITY;
  }
  plt.displayMap(myWorld_);
  res.success = true;
  return true;
}
bool RosPathPlanner::displayD2WP(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res)
{
  float avg_min_wp_dis_ = 0.0;
  float waypoint_count = 0.0;
  for (int i = 0; i < wp_distances_.size(); i++)
  {
    ROS_INFO("min distance to wp %i: %f", i, wp_distances_[i]);
    avg_min_wp_dis_ += wp_distances_[i];
    waypoint_count++;
  }
  avg_min_wp_dis_ = avg_min_wp_dis_/waypoint_count;
  ROS_INFO("average minimum distance to wp: %f", avg_min_wp_dis_);
  for (int i = 0; i < cyl_distances_.size(); i++)
    ROS_INFO("min distance to cylinder number %i: %f", i, cyl_distances_[i]);
  ROS_INFO("minimum distance to cylinder (overall): %f", min_cyl_dis_);
  res.success = true;
  return true;
}
bool RosPathPlanner::sendWaypoints(uav_msgs::UploadPath::Request &req, uav_msgs::UploadPath::Response &res)
{
  for (long unsigned int i = 0; i < rrt_obj_.all_wps_.size(); i++)
  {
    ros::Duration(0.5).sleep();
    rosplane_msgs::Waypoint new_waypoint;
    new_waypoint.landing = false;
    if (rrt_obj_.landing_now_ && i >= rrt_obj_.all_wps_.size() - 1 - 1) // landing = true on the last 2 waypoints
      new_waypoint.landing = true;
    new_waypoint.w[0] = rrt_obj_.all_wps_[i].N;
    new_waypoint.w[1] = rrt_obj_.all_wps_[i].E;
    new_waypoint.w[2] = rrt_obj_.all_wps_[i].D;

    new_waypoint.Va_d = 16.0; // TODO find a good initial spot for this Va
    if (i == 0)
      new_waypoint.set_current = true;
    else
      new_waypoint.set_current = false;
    new_waypoint.clear_wp_list = false;
    waypoint_publisher_.publish(new_waypoint);
  }

  res.success = true;
  return true;
}
void RosPathPlanner::stateCallback(const rosplane_msgs::State &msg)
{
  if (recieved_state_ == false)
  {
    gps_converter_.set_reference(msg.initial_lat, msg.initial_lon, msg.initial_alt);
    ROS_INFO("REFERENCE POINT SET");
    recieved_state_ = true;
  }
  odometry_[0] =  msg.position[1];
  odometry_[1] =  msg.position[0];
  odometry_[2] = -msg.position[2];
  chi0_        =  msg.chi;
  NED_s p;
  p.N = msg.position[0];
  p.E = msg.position[1];
  p.D = msg.position[2];
  if (has_map_)
  {
    for (int i = 0; i < rrt_obj_.map_.wps.size(); i++)
    {
      float d = (p - rrt_obj_.map_.wps[i]).norm();
      if (d < wp_distances_[i])
        wp_distances_[i] = d;
    }
    for (int i = 0; i < rrt_obj_.map_.cylinders.size(); i++)
    {
      float d;
      NED_s c;
      c.N = rrt_obj_.map_.cylinders[i].N;
      c.E = rrt_obj_.map_.cylinders[i].E;
      c.D = p.D;
      d = (p - c).norm() - rrt_obj_.map_.cylinders[i].R;
      if (d < 0.0f)
        ROS_DEBUG("d %f, h_cyl: %f, h_p %f", d,rrt_obj_.map_.cylinders[i].H, -p.D);
      if (-p.D <= rrt_obj_.map_.cylinders[i].H)
      {
        if (d < 0.0f)
          d = 0.0f;
      }
      else if (d < 0.0f)
      {
        d = -p.D - rrt_obj_.map_.cylinders[i].H;
        ROS_DEBUG("d %f", d);
      }
      else
      {
        float h = -p.D - rrt_obj_.map_.cylinders[i].H;
        d = sqrtf(d*d + h*h);
      }
      if (d < cyl_distances_[i])
        cyl_distances_[i] = d;
      if (d < min_cyl_dis_)
        min_cyl_dis_ = d;
    }
  }
}
void RosPathPlanner::updateViz(const ros::WallTimerEvent&)
{
  geometry_msgs::Point p;
  p.x = odometry_[0];
  p.y = odometry_[1];
  p.z = odometry_[2];
  plt.odomCallback(p);
}
} // end namespace rosplane

//********************************************************//
//************************ MAIN **************************//
//********************************************************//
int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_planner");
  theseus::RosPathPlanner ros_path_planner_obj;

  ros::spin();
  return 0;
} // end main
