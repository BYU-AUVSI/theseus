#include <theseus/path_planner_base.h>

namespace theseus
{
PathPlannerBase::PathPlannerBase() :
  nh_(ros::NodeHandle())
{
  //********************** PARAMETERS **********************//

  //************** SUBSCRIBERS AND PUBLISHERS **************//
  recieved_state_         = false;
  has_map_                = false;
  state_subscriber_       = nh_.subscribe("/state",100,&theseus::PathPlannerBase::stateCallback, this);
  waypoint_client_        = nh_.serviceClient<rosplane_msgs::NewWaypoints>("/waypoint_path");

  path_solver_service1_   = nh_.advertiseService("wps_now",&theseus::PathPlannerBase::wpsNow, this);
  path_solver_service2_   = nh_.advertiseService("add_wps",&theseus::PathPlannerBase::addWps, this);
  path_solver_service3_   = nh_.advertiseService("add_landing",&theseus::PathPlannerBase::addLanding, this);
  path_solver_service4_   = nh_.advertiseService("add_textfile",&theseus::PathPlannerBase::addTextfile, this);
  path_solver_service5_   = nh_.advertiseService("land_now",&theseus::PathPlannerBase::landNow, this);
  path_solver_service6_   = nh_.advertiseService("textfile_now",&theseus::PathPlannerBase::textfileNow, this);



  new_map_service_        = nh_.advertiseService("new_random_map",&theseus::PathPlannerBase::newRandomMap, this);
  plan_mission_service_   = nh_.advertiseService("plan_mission",&theseus::PathPlannerBase::planMission, this);
  send_wps_service_       = nh_.advertiseService("send_waypoints",&theseus::PathPlannerBase::sendWaypoints, this);
  replot_map_service_     = nh_.advertiseService("replot_map",&theseus::PathPlannerBase::displayMapService, this);
  wp_distance_service_    = nh_.advertiseService("display_wp_distance",&theseus::PathPlannerBase::displayD2WP, this);

  //******************** CLASS VARIABLES *******************//
  RandGen rg_in(input_file_.seed);
	rg_                     = rg_in;												    // Copy that random generator into the class.
  plt.increase_path_id_   = false;

  //***************** CALLBACKS AND TIMERS *****************//
  update_viz_timer_ = nh_.createWallTimer(ros::WallDuration(1.0/4.0), &PathPlannerBase::updateViz, this);

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
    ROS_WARN("reference latitude: %f", lat_ref);
    ROS_WARN("reference longitude: %f", lon_ref);
    ROS_WARN("reference height: %f", h_ref);
    gps_converter_.set_reference(lat_ref, lon_ref, h_ref);
    recieved_state_ =  true;
    odometry_.N     = N_init;
    odometry_.E     = E_init;
    odometry_.D     = D_init;
    chi0_           =  0.0f;
  }
  if (recieved_state_)
  {
    ending_point_ = odometry_;
    ending_chi_   = chi0_;
  }
}
PathPlannerBase::~PathPlannerBase()
{

}
bool PathPlannerBase::planMission(uav_msgs::GeneratePath::Request &req, uav_msgs::GeneratePath::Response &res)
{
  if (recieved_state_ == false)
  {
    ROS_ERROR("PATH PLANNER HAS NOT RECIEVED AN INITIAL STATE");
    return false;
  }
  bool check_wps = false;
  int num_waypoints  = req.mission.waypoints.size();
  int num_boundaries = req.mission.boundaries.size();
  int num_obstacles  = req.mission.stationary_obstacles.size();

  map_s mission_map;
  double lat, lon, alt, N, E, D;
  NED_s ned;
  cyl_s cyl;
  for (int i = 0; i < num_waypoints; i++)
  {
    lat = req.mission.waypoints[i].point.latitude;
    lon = req.mission.waypoints[i].point.longitude;
    alt = req.mission.waypoints[i].point.altitude;
    gps_converter_.gps2ned(lat, lon, alt, ned.N, ned.E, ned.D);
    mission_map.wps.push_back(ned);
    ROS_INFO("waypoints:: %f %f %f", ned.N, ned.E, ned.D);
  }
  for (int i = 0; i < num_boundaries; i++)
  {
    lat = req.mission.boundaries[i].point.latitude;
    lon = req.mission.boundaries[i].point.longitude;
    alt = req.mission.boundaries[i].point.altitude;
    gps_converter_.gps2ned(lat, lon, alt, ned.N, ned.E, ned.D);
    mission_map.boundary_pts.push_back(ned);
    ROS_WARN("boundary points:: %f %f %f", ned.N, ned.E, ned.D);
  }
  for (int i = 0; i < num_obstacles; i++)
  {
    lat   = req.mission.stationary_obstacles[i].point.latitude;
    lon   = req.mission.stationary_obstacles[i].point.longitude;
    alt   = req.mission.stationary_obstacles[i].point.altitude;
    cyl.R = req.mission.stationary_obstacles[i].cylinder_height;
    cyl.H = req.mission.stationary_obstacles[i].cylinder_radius;
    gps_converter_.gps2ned(lat, lon, alt, cyl.N, cyl.E, D);
    mission_map.cylinders.push_back(cyl);
    ROS_FATAL("cylinder:: %f %f", cyl.N, cyl.E);
  }
  bool direct_hit;
  if (req.mission.mission_type == req.mission.MISSION_TYPE_WAYPOINT)
  {
    last_primary_wps_ = mission_map.wps;
    direct_hit = true;
  }
  else
    direct_hit = false;
  double chi;
  bool landing = false;
  if (req.mission.mission_type == req.mission.MISSION_TYPE_LAND)
  {
    chi = req.mission.waypoints[0].point.chi;
    ROS_INFO("chi: %f", chi);
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
  if (req.mission.mission_type == req.mission.MISSION_TYPE_SEARCH)
  {
    check_wps = true;
    rrt_obj_.newMap(mission_map);
    mission_map.wps.clear();
    for (int i = 0; i < num_waypoints; i++)
    {
      lat = req.mission.waypoints[i].point.latitude;
      lon = req.mission.waypoints[i].point.longitude;
      alt = req.mission.waypoints[i].point.altitude;
      gps_converter_.gps2ned(lat, lon, alt, ned.N, ned.E, ned.D);
      if (rrt_obj_.checkPoint(ned, input_file_.clearance + 0.5f*input_file_.turn_radius))
        mission_map.wps.push_back(ned);
    }
  }

  myWorld_ = mission_map;
  rrt_obj_.newMap(mission_map);
  if (has_map_ == false)
  {
    cyl_distances_.clear();
    for (int i = 0; i < rrt_obj_.map_.cylinders.size(); i++)
      cyl_distances_.push_back(INFINITY);
    min_cyl_dis_ = INFINITY;
  }
  has_map_ = true;
  if (req.mission.mission_type == req.mission.MISSION_TYPE_WAYPOINT)
  {
    wp_distances_.clear();
    for (int i = 0; i < rrt_obj_.map_.wps.size(); i++)
      wp_distances_.push_back(INFINITY);
  }
  ROS_INFO("RECIEVED JUDGES' MAP");
  plt.displayMap(myWorld_);
  NED_s ref_zero(0.0f, 0.0f, 0.0f);
  if (rrt_obj_.col_det_.checkWithinBoundaries(ref_zero, 0.0f) == false)
  {
    ROS_WARN("GPS reference point is not within the boundary points");

    if ((myWorld_.boundary_pts[0] - ref_zero).norm() > 160934.0f)
    {
      ROS_FATAL("GPS REFERENCE POINT IS MORE THAN 100 MILES FROM FIRST BOUNDARY POINT");
      return true;
    }
  }
  
  bool now = req.mission.now; // Whether to immediately send the planned path to the plane.

  solveStatic(landing, direct_hit, now, check_wps);
  return true;
}
bool PathPlannerBase::newRandomMap(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  getInitialMap();
  ROS_INFO("RECIEVED NEW RANDOM MAP");
  plt.clearRViz(myWorld_);
  res.success = true;
  return true;
}
bool PathPlannerBase::solveStatic(bool landing, bool direct_hit, bool now, bool check_wps)
{
  if (recieved_state_ == false)
  {
    ROS_ERROR("PATH PLANNER HAS NOT RECIEVED AN INITIAL STATE");
    return false;
  }
  if (has_map_ == false)
    getInitialMap();
  if (check_wps == true)
  {
    std::vector<NED_s> all_wps = myWorld_.wps;
    myWorld_.wps.clear();
    for (int j = 0; j < all_wps.size(); j++)
    {
      if (rrt_obj_.checkPoint(all_wps[j], input_file_.clearance + 0.5f*input_file_.turn_radius))
        myWorld_.wps.push_back(all_wps[j]);
    }
    rrt_obj_.newMap(myWorld_);
  }

  float initial_chi;
  NED_s initial_pos;
  if (now)
  {
    initial_chi = chi0_;
    initial_pos = odometry_;
  }
  else
  {
    initial_pos = ending_point_;
    initial_chi = ending_chi_;
  }
  plt.displayMap(myWorld_);
  rrt_obj_.solveStatic(initial_pos, initial_chi, direct_hit, landing);
  if (now)
    sendWaypointsCore(now);
  plt.displayPath(initial_pos, rrt_obj_.all_wps_, clr.green, 5.0);
  if (rrt_obj_.landing_now_ == false)
    plt.drawCircle(rrt_obj_.all_wps_.back(), input_file_.loiter_radius);
  return true;
}
bool PathPlannerBase::wpsNow(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res)
{
  if (has_map_ == false)
    getInitialMap();
  else
  {
    myWorld_.wps.clear();
    myWorld_.wps = last_primary_wps_;
    rrt_obj_.newMap(myWorld_);
  }
  res.success = solveStatic(false, true, true, false);
  return true;
}
bool PathPlannerBase::addWps(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res)
{
  if (has_map_ == false)
    getInitialMap();
  else
  {
    myWorld_.wps.clear();
    myWorld_.wps = last_primary_wps_;
    rrt_obj_.newMap(myWorld_);
  }
  res.success = solveStatic(false, true, false, false);
  return true;
}
bool PathPlannerBase::landing(bool now)
{
  if (has_map_ == false)
    getInitialMap();
  std::fstream fin;
  fin.open("landing.txt", std::ifstream::in);
  if (!(fin.is_open()))
  {
    ROS_FATAL("WAYPOINTS FILE DID NOT OPEN."); // try putting it in the ~/.ros directory.
    return false;
  }
  float chi;
  NED_s descend_point;
  fin >> descend_point.N >> descend_point.E >> descend_point.D >> chi;
  fin.close();
  bool direct_hit = true;
  bool landing = true;
  rrt_obj_.map_.wps.clear();
  rrt_obj_.map_.wps.push_back(descend_point);
  // if (rrt_obj_.col_det_.checkPoint(descend_point, input_file_.clearance) == false)
  // {
  //   ROS_ERROR("Landing point violates boundary or obstacles. Choose another landing spot");
  //   return false;
  // }
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
  return solveStatic(true, false, now, false);
}
bool PathPlannerBase::addLanding(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res)
{
  res.success = landing(false);
  return true;
}
bool PathPlannerBase::landNow(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res)
{
  res.success = landing(true);
  return true;
}
bool PathPlannerBase::textfile(bool now)
{
  if (has_map_ == false)
    getInitialMap();
  std::fstream fin;
  fin.open("path.txt", std::ifstream::in);
  if (!(fin.is_open()))
  {
    ROS_FATAL("WAYPOINTS FILE DID NOT OPEN."); // try putting it in the ~/.ros directory.
    return false;
  }
  NED_s wp;
  myWorld_.wps.clear();
  while (fin.eof() == false)
  {
    fin >> wp.N >> wp.E >> wp.D;
    if (fin.eof())
      break;
    myWorld_.wps.push_back(wp);
  }
  fin.close();
  rrt_obj_.newMap(myWorld_);
  return solveStatic(false, false, now, true);
}
bool PathPlannerBase::addTextfile(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res)
{
  res.success = textfile(false);
  return true;
}
bool PathPlannerBase::textfileNow(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res)
{
  res.success = textfile(true);
  return true;
}
bool PathPlannerBase::displayMapService(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res)
{
  if (has_map_ == false)
    getInitialMap();
  plt.displayMap(myWorld_);
  res.success = true;
  return true;
}
bool PathPlannerBase::displayD2WP(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res)
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
bool PathPlannerBase::sendWaypointsCore(bool now)
{
  rosplane_msgs::NewWaypoints srv;
  rosplane_msgs::Waypoint new_waypoint;
  NED_s in_front;
  for (long unsigned int i = 0; i < rrt_obj_.all_wps_.size(); i++)
  {
    new_waypoint.landing = false;
    if (rrt_obj_.landing_now_ && i >= rrt_obj_.all_wps_.size() - 1 - 1) // landing = true on the last 2 waypoints
      new_waypoint.landing = true;
    new_waypoint.w[0] = rrt_obj_.all_wps_[i].N;
    new_waypoint.w[1] = rrt_obj_.all_wps_[i].E;
    new_waypoint.w[2] = rrt_obj_.all_wps_[i].D;
    nh_.param<float>("pp/Va", new_waypoint.Va_d, 20.0);
    if (rrt_obj_.landing_now_ == false && i == rrt_obj_.all_wps_.size() - 1)
      new_waypoint.loiter_point  = true;
    else
      new_waypoint.loiter_point  = false;
    new_waypoint.priority = rrt_obj_.all_priorities_[i];
    if (now == true && i == 0)
      new_waypoint.set_current = true;
    else
      new_waypoint.set_current = false;
    new_waypoint.clear_wp_list = false;
    srv.request.waypoints.push_back(new_waypoint);
  }
  if (srv.request.waypoints.back().loiter_point == true)
  {
    srv.request.waypoints.back().loiter_point = false;
    new_waypoint.set_current  = false;
    int last_idx = rrt_obj_.all_wps_.size() - 1;
    in_front = rrt_obj_.all_wps_[last_idx] + \
               ((rrt_obj_.all_wps_[last_idx] - rrt_obj_.all_wps_[last_idx - 1]).normalize())*0.1;
    ROS_DEBUG("in_front N: %f, E: %f, D: %f", in_front.N, in_front.E, in_front.D);
    new_waypoint.priority = 3;
    new_waypoint.w[0] = in_front.N;
    new_waypoint.w[1] = in_front.E;
    new_waypoint.w[2] = in_front.D;
    srv.request.waypoints.push_back(new_waypoint);
  }
  bool found_service = ros::service::waitForService("/waypoint_path", ros::Duration(1.0));
  while (found_service == false)
  {
    ROS_WARN("No waypoint server found. Checking again.");
    found_service = ros::service::waitForService("/waypoint_path", ros::Duration(1.0));
  }
  bool sent_correctly = waypoint_client_.call(srv);
  if (sent_correctly)
    ROS_INFO("Waypoints succesfully sent");
  else
    ROS_ERROR("Waypoint server unsuccessful");

  ending_point_ = rrt_obj_.ending_point_;
  ending_chi_ = rrt_obj_.ending_chi_;

  int priority_level = 0;
  for (int i = 0; i < srv.request.waypoints.size(); i++)
    if (srv.request.waypoints[i].priority > priority_level)
      priority_level = srv.request.waypoints[i].priority;
  for (int i = 0; i < all_sent_wps_.size(); i++)
    if (all_sent_priorities_[i] < priority_level)
    {
      all_sent_wps_.erase(all_sent_wps_.begin() + i);
      i--;
    }
  for (int i = 0; i < srv.request.waypoints.size(); i++)
  {
    if (all_sent_wps_.size() == 0)
    {
      all_sent_priorities_.push_back(5);
      all_sent_wps_.push_back(odometry_);
    }
    NED_s next_wp;
    next_wp.N = srv.request.waypoints[i].w[0];
    next_wp.E = srv.request.waypoints[i].w[1];
    next_wp.D = srv.request.waypoints[i].w[2];
    all_sent_priorities_.push_back(srv.request.waypoints[i].priority);
    all_sent_wps_.push_back(next_wp);
  }
  plt.clearRViz(myWorld_, all_sent_wps_, clr.purple, 5.0);
  if (srv.request.waypoints.back().loiter_point == true)
    plt.drawCircle(in_front, input_file_.loiter_radius);
  return sent_correctly;
}
bool PathPlannerBase::sendWaypoints(uav_msgs::UploadPath::Request &req, uav_msgs::UploadPath::Response &res)
{
  res.success = sendWaypointsCore(false);
  return true;
}
void PathPlannerBase::getInitialMap()
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
  last_primary_wps_ = myWorld_.wps;
}
void PathPlannerBase::stateCallback(const rosplane_msgs::State &msg)
{
  if (recieved_state_ == false)
  {
    gps_converter_.set_reference(msg.initial_lat, msg.initial_lon, msg.initial_alt);
    ROS_INFO("REFERENCE POINT SET");
    recieved_state_ = true;
  }
  odometry_.N = msg.position[0];
  odometry_.E = msg.position[1];
  odometry_.D = msg.position[2];
  chi0_       =  msg.chi;
  if (has_map_)
  {
    for (int i = 0; i < rrt_obj_.map_.wps.size(); i++)
    {
      float d = (odometry_ - rrt_obj_.map_.wps[i]).norm();
      if (d < wp_distances_[i])
        wp_distances_[i] = d;
    }
    for (int i = 0; i < rrt_obj_.map_.cylinders.size(); i++)
    {
      float d;
      NED_s c;
      c.N = rrt_obj_.map_.cylinders[i].N;
      c.E = rrt_obj_.map_.cylinders[i].E;
      c.D = odometry_.D;
      d = (odometry_ - c).norm() - rrt_obj_.map_.cylinders[i].R;
      if (-odometry_.D <= rrt_obj_.map_.cylinders[i].H)
      {
        if (d < 0.0f)
          d = 0.0f;
      }
      else if (d < 0.0f)
        d = -odometry_.D - rrt_obj_.map_.cylinders[i].H;
      else
      {
        float h = -odometry_.D - rrt_obj_.map_.cylinders[i].H;
        d = sqrtf(d*d + h*h);
      }
      if (d < cyl_distances_[i])
        cyl_distances_[i] = d;
      if (d < min_cyl_dis_)
        min_cyl_dis_ = d;
    }
  }
}
void PathPlannerBase::updateViz(const ros::WallTimerEvent&)
{
  geometry_msgs::Point p;
  p.x =  odometry_.E;
  p.y =  odometry_.N;
  p.z = -odometry_.D;
  plt.odomCallback(p);
}
} // end namespace rosplane

//********************************************************//
//************************ MAIN **************************//
//********************************************************//
int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_planner");
  theseus::PathPlannerBase path_planner_obj;

  ros::spin();
  return 0;
} // end main
