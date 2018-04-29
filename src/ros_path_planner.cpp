#include <theseus/ros_path_planner.h>

namespace theseus
{
RosPathPlanner::RosPathPlanner() :
  nh_(ros::NodeHandle())
{
  //********************** PARAMETERS **********************//

  //************** SUBSCRIBERS AND PUBLISHERS **************//
  recieved_state_        = false;
  has_map_               = false;
  state_subscriber_      = nh_.subscribe("/state",100,&theseus::RosPathPlanner::stateCallback, this);
  waypoint_publisher_    = nh_.advertise<rosplane_msgs::Waypoint>("/waypoint_path", 1);
  path_solver_service_   = nh_.advertiseService("solve_static",&theseus::RosPathPlanner::solveStatic, this);
  new_map_service_       = nh_.advertiseService("new_random_map",&theseus::RosPathPlanner::newRandomMap, this);
  plan_mission_service_  = nh_.advertiseService("plan_mission",&theseus::RosPathPlanner::planMission, this);
  send_wps_service_      = nh_.advertiseService("send_waypoints",&theseus::RosPathPlanner::sendWaypoints, this);
  replot_map_service_    = nh_.advertiseService("replot_map",&theseus::RosPathPlanner::displayMapService, this);
  wp_distance_service_   = nh_.advertiseService("display_wp_distance",&theseus::RosPathPlanner::displayD2WP, this);
  marker_pub_            = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  //******************** CLASS VARIABLES *******************//
  RandGen rg_in(input_file_.seed);
	rg_                          = rg_in;												    // Copy that random generator into the class.
  odom_mkr_.header.frame_id    = "/local_ENU";
  odom_mkr_.ns                 = "plane_odom";
  odom_mkr_.type               = visualization_msgs::Marker::POINTS;
  odom_mkr_.action             = visualization_msgs::Marker::ADD;
  odom_mkr_.pose.orientation.x = 0.0;
  odom_mkr_.pose.orientation.y = 0.0;
  odom_mkr_.pose.orientation.z = 0.0;
  odom_mkr_.pose.orientation.w = 1.0;
  odom_mkr_.color.r            = 0.0f;
  odom_mkr_.color.g            = 1.0f;
  odom_mkr_.color.b            = 0.0f;
  odom_mkr_.color.a            = 1.0;
  odom_mkr_.lifetime           = ros::Duration();
  odom_mkr_.scale.x            = 15.0; // point width
  odom_mkr_.scale.y            = 15.0; // point width

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
    nh_.param<double>("lat_ref", lat_ref, 38.14326388888889);
    nh_.param<double>("lon_ref", lon_ref, -76.43075);
    nh_.param<double>("h_ref", h_ref, 6.701);
    nh_.param<float>("N_init", N_init, 0.0);
    nh_.param<float>("E_init", E_init, 0.0);
    nh_.param<float>("D_init", D_init, 0.0);
    ROS_WARN("testing = true, initializing reference and initial position");
    gps_converter_.set_reference(lat_ref, lon_ref, h_ref);
    recieved_state_ = true;
    odometry_[0]    = N_init;
    odometry_[1]    = E_init;
    odometry_[2]    = D_init;
    chi0_           = 0.0;
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
  if (req.mission.mission_type == req.mission.MISSION_TYPE_WAYPOINT)
    direct_hit = true;
  else
    direct_hit = false;

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
  displayMap();
  rrt_obj_.solveStatic(pos, chi0_, true);
  displayPath();
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
  displayMap();
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
  displayMap();
  NED_s pos;
  pos.N =  odometry_[1];
  pos.E =  odometry_[0];
  pos.D = -odometry_[2];
  bool direct_hit = true;
  rrt_obj_.solveStatic(pos, chi0_, direct_hit);
  visualization_msgs::Marker clear_mkr;
  clear_mkr.action = visualization_msgs::Marker::DELETEALL;
  marker_pub_.publish(clear_mkr);
  displayMap();
  ROS_INFO("solved the path. displaying path");
  displayPath();
  ROS_INFO("returning");
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
  displayMap();
  res.success = true;
  return true;
}
bool RosPathPlanner::displayD2WP(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res)
{
  for (int i = 0; i < wp_distances_.size(); i++)
    ROS_INFO("min distance to wp %i: %f", i, wp_distances_[i]);
  for (int i = 0; i < cyl_distances_.size(); i++)
    ROS_INFO("min distance to cylinder number %i: %f", i, cyl_distances_[i]);
  ROS_INFO("minimum distance to cylinder (overall): %f", min_cyl_dis_);
  res.success = true;
  return true;
}
void RosPathPlanner::displayMap()
{
  ROS_INFO("Displaying Map");
  visualization_msgs::Marker obs_mkr, pWPS_mkr, bds_mkr, clear_mkr;
  clear_mkr.action = visualization_msgs::Marker::DELETEALL;
  marker_pub_.publish(clear_mkr);

  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  obs_mkr.header.frame_id = pWPS_mkr.header.frame_id = bds_mkr.header.frame_id = "/local_ENU";
  // Set the namespace and id for this obs_mkr.  This serves to create a unique ID
  // Any obs_mkr sent with the same namespace and id will overwrite the old one
  obs_mkr.ns            = "static_obstacle";
  pWPS_mkr.ns           = "primary_wps";
  bds_mkr.ns            = "boundaries";
  uint32_t cyl          = visualization_msgs::Marker::CYLINDER;
  uint32_t pts          = visualization_msgs::Marker::POINTS;
  uint32_t lis          = visualization_msgs::Marker::LINE_STRIP;
  obs_mkr.type          = cyl;
  pWPS_mkr.type         = pts;
  bds_mkr.type          = lis;
  // Set the obs_mkr action.  Options are ADD (Which is really create or modify), DELETE, and new in ROS Indigo: 3 (DELETEALL)
  obs_mkr.action = pWPS_mkr.action = bds_mkr.action  = visualization_msgs::Marker::ADD;
  obs_mkr.pose.orientation.x  = pWPS_mkr.pose.orientation.x = bds_mkr.pose.orientation.x = 0.0;
  obs_mkr.pose.orientation.y  = pWPS_mkr.pose.orientation.y = bds_mkr.pose.orientation.y = 0.0;
  obs_mkr.pose.orientation.z  = pWPS_mkr.pose.orientation.z = bds_mkr.pose.orientation.z = 0.0;
  obs_mkr.pose.orientation.w  = pWPS_mkr.pose.orientation.w = bds_mkr.pose.orientation.w = 1.0;
  // Set the color -- be sure to set alpha to something non-zero!
  obs_mkr.color.r             = 1.0f;
  obs_mkr.color.g             = 0.0f;
  obs_mkr.color.b             = 0.0f;
  obs_mkr.color.a             = 0.9;
  pWPS_mkr.color.r            = 1.0f;
  pWPS_mkr.color.g            = 1.0f;
  pWPS_mkr.color.b            = 0.0f;
  pWPS_mkr.color.a            = 1.0;
  bds_mkr.color.r             = 1.0f;
  bds_mkr.color.g             = 0.0f;
  bds_mkr.color.b             = 0.0f;
  bds_mkr.color.a             = 1.0;
  obs_mkr.lifetime = pWPS_mkr.lifetime = bds_mkr.lifetime  = ros::Duration();

  int id = 0;
  obs_mkr.header.stamp = ros::Time::now();
  ROS_INFO("Number of Cylinders: %lu", myWorld_.cylinders.size());
  for (long unsigned int i = 0; i <  myWorld_.cylinders.size(); i++)
  {
    obs_mkr.id = id++;
    obs_mkr.pose.position.x    = myWorld_.cylinders[i].E;     // Center X position
    obs_mkr.pose.position.y    = myWorld_.cylinders[i].N;     // Center Y position
    obs_mkr.pose.position.z    = myWorld_.cylinders[i].H/2.0; // Center Z position
    // Set the scale of the obs_mkr -- 1x1x1 here means 1m on a side
    obs_mkr.scale.x = myWorld_.cylinders[i].R*2.0; // Diameter in x direction
    obs_mkr.scale.y = myWorld_.cylinders[i].R*2.0; // Diameter in y direction
    obs_mkr.scale.z = myWorld_.cylinders[i].H;     // Height
    // Publish the obs_mkr
    while (marker_pub_.getNumSubscribers() < 1)
    {
      if (!ros::ok())
        return;
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub_.publish(obs_mkr);
    sleep(0.05); // apparently needs a small delay otherwise rviz can't keep up?
  }

  // primary waypoints
  pWPS_mkr.header.stamp = ros::Time::now();
  pWPS_mkr.id           =  0;
  pWPS_mkr.scale.x      =  25.0; // point width
  pWPS_mkr.scale.y      =  25.0; // point height
  ROS_INFO("Number of Waypoints: %lu", myWorld_.wps.size());
  for (long unsigned int i = 0; i < myWorld_.wps.size(); i++)
  {
    geometry_msgs::Point p;
    p.y =  myWorld_.wps[i].N;
    p.x =  myWorld_.wps[i].E;
    p.z = -myWorld_.wps[i].D;
    pWPS_mkr.points.push_back(p);
  }
  marker_pub_.publish(pWPS_mkr);
  sleep(0.05);

  // Boundaries
  bds_mkr.header.stamp = ros::Time::now();
  bds_mkr.id           =  0;
  bds_mkr.scale.x      =  15.0; // line width
  ROS_INFO("Number of Boundary Points: %lu",  myWorld_.boundary_pts.size());
  for (long unsigned int i = 0; i < myWorld_.boundary_pts.size(); i++)
  {
    geometry_msgs::Point p;
    p.y = myWorld_.boundary_pts[i].N;
    p.x = myWorld_.boundary_pts[i].E;
    p.z = 0.0;
    bds_mkr.points.push_back(p);
  }
  geometry_msgs::Point p0;
  p0.y = myWorld_.boundary_pts[0].N;
  p0.x = myWorld_.boundary_pts[0].E;
  p0.z = 0.0;
  bds_mkr.points.push_back(p0);
  p0.y = myWorld_.boundary_pts[1].N;
  p0.x = myWorld_.boundary_pts[1].E;
  p0.z = 0.0;
  bds_mkr.points.push_back(p0);
  marker_pub_.publish(bds_mkr);
  sleep(0.05);
}
void RosPathPlanner::displayPath()
{
  visualization_msgs::Marker planned_path_mkr, aWPS_mkr;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  planned_path_mkr.header.frame_id = aWPS_mkr.header.frame_id = "/local_ENU";
  // Set the namespace and id for this obs_mkr.  This serves to create a unique ID
  // Any obs_mkr sent with the same namespace and id will overwrite the old one
  planned_path_mkr.ns   = "planned_path";
  aWPS_mkr.ns           = "all_wps";
  uint32_t cyl          = visualization_msgs::Marker::CYLINDER;
  uint32_t pts          = visualization_msgs::Marker::POINTS;
  uint32_t lis          = visualization_msgs::Marker::LINE_STRIP;
  planned_path_mkr.type = lis;
  aWPS_mkr.type         = pts;
  // Set the obs_mkr action.  Options are ADD (Which is really create or modify), DELETE, and new in ROS Indigo: 3 (DELETEALL)
  planned_path_mkr.action = aWPS_mkr.action = visualization_msgs::Marker::ADD;
  planned_path_mkr.pose.orientation.x = aWPS_mkr.pose.orientation.x = 0.0;
  planned_path_mkr.pose.orientation.y = aWPS_mkr.pose.orientation.y = 0.0;
  planned_path_mkr.pose.orientation.z = aWPS_mkr.pose.orientation.z = 0.0;
  planned_path_mkr.pose.orientation.w = aWPS_mkr.pose.orientation.w = 1.0;
  // Set the color -- be sure to set alpha to something non-zero!
  planned_path_mkr.color.r    = 0.0f;
  planned_path_mkr.color.g    = 0.0f;
  planned_path_mkr.color.b    = 1.0f;
  planned_path_mkr.color.a    = 1.0;
  aWPS_mkr.color.r            = 0.0f;
  aWPS_mkr.color.g            = 0.0f;
  aWPS_mkr.color.b            = 1.0f;
  aWPS_mkr.color.a            = 1.0;
  planned_path_mkr.lifetime = aWPS_mkr.lifetime = ros::Duration();

  while (marker_pub_.getNumSubscribers() < 1)
  {
    if (!ros::ok())
      return;
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }
  // all waypoints
  if (false)
  {
    aWPS_mkr.header.stamp = ros::Time::now();
    aWPS_mkr.id           =  0;
    aWPS_mkr.scale.x      =  10.0; // point width
    aWPS_mkr.scale.y      =  10.0; // point height
    for (long unsigned int i = 0; i < rrt_obj_.all_wps_.size(); i++)
  	{
      geometry_msgs::Point p;
      p.y =  rrt_obj_.all_wps_[i].N;
      p.x =  rrt_obj_.all_wps_[i].E;
      p.z = -rrt_obj_.all_wps_[i].D;
      aWPS_mkr.points.push_back(p);
    }
    marker_pub_.publish(aWPS_mkr);
    sleep(0.05);
  }

  // Plot desired path
  ROS_INFO("finding desired positions");
  planned_path_mkr.header.stamp = ros::Time::now();
  planned_path_mkr.id           =  0;
  planned_path_mkr.scale.x      =  5.0; // line width
  ROS_INFO("number of waypoints: %lu", rrt_obj_.all_wps_.size());
  std::vector<NED_s> all_wps;
  NED_s pos;
  pos.N =  odometry_[1];
  pos.E =  odometry_[0];
  pos.D = -odometry_[2];
  all_wps.push_back(pos);
  for (long unsigned int i = 0; i < rrt_obj_.all_wps_.size(); i++)
	{
    ROS_INFO("n: %f e: %f d: %f", rrt_obj_.all_wps_[i].N, rrt_obj_.all_wps_[i].E, rrt_obj_.all_wps_[i].D);
    pos.N = rrt_obj_.all_wps_[i].N;
    pos.E = rrt_obj_.all_wps_[i].E;
    pos.D = rrt_obj_.all_wps_[i].D;
    all_wps.push_back(pos);
  }
  std::vector<NED_s> vis_path;
  vis_path.push_back(all_wps[0]);
  for (int i = 1; i < all_wps.size() - 1; i++)
  {
    fillet_s fil;
    fil.calculate(all_wps[i - 1], all_wps[i], all_wps[i + 1], input_file_.turn_radius);
    vis_path.push_back(fil.z1);

    std::vector<std::vector<float> > NcEc;
    if (fil.lambda == -1)
    {
      NcEc = arc(fil.c.N, fil.c.E, input_file_.turn_radius, (fil.z2 - fil.c).getChi(), (fil.z1 - fil.c).getChi());
      // need to flip the vectors
      std::reverse(NcEc[0].begin(),NcEc[0].end());
      std::reverse(NcEc[1].begin(),NcEc[1].end());
    }
    if (fil.lambda ==  1)
    {
      NcEc = arc(fil.c.N, fil.c.E, input_file_.turn_radius, (fil.z1 - fil.c).getChi(), (fil.z2 - fil.c).getChi());
    }
    std::vector<float> Nc = NcEc[0];
    std::vector<float> Ec = NcEc[1];
    for (int j = 0; j < Nc.size(); j++)
    {
      pos.N = Nc[j];
      pos.E = Ec[j];
      pos.D = fil.c.D;
      vis_path.push_back(pos);
    }
    vis_path.push_back(fil.z2);
  }
  vis_path.push_back(all_wps[all_wps.size() - 1]);

  for (int i = 0; i < vis_path.size(); i++)
  {
    geometry_msgs::Point p;
    p.x =  vis_path[i].E;
    p.y =  vis_path[i].N;
    p.z = -vis_path[i].D;
    planned_path_mkr.points.push_back(p);
  }
  marker_pub_.publish(planned_path_mkr);
  sleep(0.05);
}
bool RosPathPlanner::sendWaypoints(uav_msgs::UploadPath::Request &req, uav_msgs::UploadPath::Response &res)
{
  for (long unsigned int i = 0; i < rrt_obj_.all_wps_.size(); i++)
  {
    ros::Duration(0.5).sleep();
    rosplane_msgs::Waypoint new_waypoint;

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
std::vector<std::vector<float > > RosPathPlanner::arc(float N, float E, float r, float aS, float aE)
{
  std::vector<float> Nc, Ec;
  while (aE < aS)
    aE += 2.0f*M_PI;
  if (aE - aS == 0.0)
  {
    Ec.push_back(r*sin(aS)+ E);
    Nc.push_back(r*cos(aS)+ N);
  }
  for (float th = aS; th <= aE; th += M_PI/35.0)
  {
    Ec.push_back(r*sin(th)+ E);
    Nc.push_back(r*cos(th)+ N);
  }
  std::vector<std::vector<float> > NcEc;
  NcEc.push_back(Nc);
  NcEc.push_back(Ec);
  return NcEc;
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
  odom_mkr_.header.stamp = ros::Time::now();
  geometry_msgs::Point p;
  p.x = odometry_[0];
  p.y = odometry_[1];
  p.z = odometry_[2];
  odom_mkr_.points.push_back(p);
  marker_pub_.publish(odom_mkr_);
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
