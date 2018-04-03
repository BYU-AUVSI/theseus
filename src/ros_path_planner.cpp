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
  state_subscriber_      = nh_.subscribe("/state",1,&theseus::RosPathPlanner::stateCallback, this);
  waypoint_publisher_    = nh_.advertise<rosplane_msgs::Waypoint>("/waypoint_path", 1);
  path_solver_service_   = nh_.advertiseService("solve_static",&theseus::RosPathPlanner::solveStatic, this);
  new_map_service_       = nh_.advertiseService("new_random_map",&theseus::RosPathPlanner::newRandomMap, this);
  plan_mission_service_  = nh_.advertiseService("plan_mission",&theseus::RosPathPlanner::planMission, this);
  send_wps_service_      = nh_.advertiseService("send_waypoints",&theseus::RosPathPlanner::sendWaypoints, this);
  marker_pub_            = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);

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
  update_viz_timer_ = nh_.createWallTimer(ros::WallDuration(1.0/5.0), &RosPathPlanner::updateViz, this);

  //********************** FUNCTIONS ***********************//
  RRT rrt_obj(myWorld_.map, input_file_.seed, &input_file_, rrt_i_);
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
  rrt_obj_.newMap(mission_map);
  has_map_ = true;
  ROS_INFO("RECIEVED JUDGES MAP");
  NED_s pos;
  pos.N =  odometry_[1];
  pos.E =  odometry_[0];
  pos.D = -odometry_[2];
  displayMap();
  rrt_obj_.solveStatic(pos);
  displayPath();
  return true;
}
bool RosPathPlanner::newRandomMap(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  mapper myWorld(rg_.UINT(), &input_file_);
  myWorld_ = myWorld;
  rrt_obj_.newMap(myWorld_.map);
  has_map_ = true;
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
    myWorld_ = myWorld;
    rrt_obj_.newMap(myWorld_.map);
    has_map_ = true;
  }
  displayMap();
  NED_s pos;
  pos.N =  odometry_[1];
  pos.E =  odometry_[0];
  pos.D = -odometry_[2];
  rrt_obj_.solveStatic(pos);
  displayPath();
  res.success = true;
  return true;
}
void RosPathPlanner::displayMap()
{

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
  for (long unsigned int i = 0; i <  myWorld_.map.cylinders.size(); i++)
  {
    obs_mkr.id = id++;
    obs_mkr.pose.position.x    = myWorld_.map.cylinders[i].E;     // Center X position
    obs_mkr.pose.position.y    = myWorld_.map.cylinders[i].N;     // Center Y position
    obs_mkr.pose.position.z    = myWorld_.map.cylinders[i].H/2.0; // Center Z position
    // Set the scale of the obs_mkr -- 1x1x1 here means 1m on a side
    obs_mkr.scale.x = myWorld_.map.cylinders[i].R*2.0; // Diameter in x direction
    obs_mkr.scale.y = myWorld_.map.cylinders[i].R*2.0; // Diameter in y direction
    obs_mkr.scale.z = myWorld_.map.cylinders[i].H;     // Height
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
  for (long unsigned int i = 0; i < myWorld_.map.wps.size(); i++)
  {
    geometry_msgs::Point p;
    p.y =  myWorld_.map.wps[i].N;
    p.x =  myWorld_.map.wps[i].E;
    p.z = -myWorld_.map.wps[i].D;
    pWPS_mkr.points.push_back(p);
  }
  marker_pub_.publish(pWPS_mkr);
  sleep(0.05);

  // Boundaries
  bds_mkr.header.stamp = ros::Time::now();
  bds_mkr.id           =  0;
  bds_mkr.scale.x      =  15.0; // line width
  for (long unsigned int i = 0; i < myWorld_.map.boundary_pts.size(); i++)
  {
    geometry_msgs::Point p;
    p.y = myWorld_.map.boundary_pts[i].N;
    p.x = myWorld_.map.boundary_pts[i].E;
    p.z = 0.0;
    bds_mkr.points.push_back(p);
  }
  geometry_msgs::Point p0;
  p0.y = myWorld_.map.boundary_pts[0].N;
  p0.x = myWorld_.map.boundary_pts[0].E;
  p0.z = 0.0;
  bds_mkr.points.push_back(p0);
  p0.y = myWorld_.map.boundary_pts[1].N;
  p0.x = myWorld_.map.boundary_pts[1].E;
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
      for (long unsigned int j = 0; j < rrt_obj_.all_wps_[i].size(); j++)
  		{
        geometry_msgs::Point p;
        p.y =  rrt_obj_.all_wps_[i][j].N;
        p.x =  rrt_obj_.all_wps_[i][j].E;
        p.z = -rrt_obj_.all_wps_[i][j].D;
        aWPS_mkr.points.push_back(p);
      }
    }
    marker_pub_.publish(aWPS_mkr);
    sleep(0.05);
  }

  // Plot desired path
  planned_path_mkr.header.stamp = ros::Time::now();
  planned_path_mkr.id           =  0;
  planned_path_mkr.scale.x      =  15.0; // line width
  std::vector<double> xwpsAll, ywpsAll, dwpsAll, xwps, ywps, dwps;
  for (long unsigned int i = 0; i < rrt_obj_.all_wps_.size(); i++)
	{
    for (long unsigned int j = 0; j < rrt_obj_.all_wps_[i].size(); j++)
		{
      xwpsAll.push_back(rrt_obj_.all_wps_[i][j].E);
      ywpsAll.push_back(rrt_obj_.all_wps_[i][j].N);
      dwpsAll.push_back(rrt_obj_.all_wps_[i][j].D);
    }
  }
  for (long unsigned int i = 0; i < myWorld_.map.wps.size(); i++)
  {
    xwps.push_back(myWorld_.map.wps[i].E);
    ywps.push_back(myWorld_.map.wps[i].N);
    dwps.push_back(myWorld_.map.wps[i].D);
  }
  std::vector<std::vector<double> > mav_path = filletMavPath(xwps, ywps, dwps, xwpsAll, ywpsAll, dwpsAll);
  for (long unsigned int i = 0; i < mav_path.size(); i++)
  {
    geometry_msgs::Point p;
    p.x =  mav_path[i][0];
    p.y =  mav_path[i][1];
    p.z = -mav_path[i][2];
    planned_path_mkr.points.push_back(p);
  }
  marker_pub_.publish(planned_path_mkr);
  sleep(0.05);
  for (long unsigned int i = 0; i < rrt_obj_.all_wps_.size(); i++)
  {
    for (long unsigned int j = 0; j < rrt_obj_.all_wps_[i].size(); j++)
    {
      geometry_msgs::Point p;
      p.y =  rrt_obj_.all_wps_[i][j].N;
      p.x =  rrt_obj_.all_wps_[i][j].E;
      p.z = -rrt_obj_.all_wps_[i][j].D;
      aWPS_mkr.points.push_back(p);
      ros::Duration(0.05).sleep();
    }
  }
}
bool RosPathPlanner::sendWaypoints(uav_msgs::UploadPath::Request &req, uav_msgs::UploadPath::Response &res)
{
  for (long unsigned int i = 0; i < rrt_obj_.all_wps_.size(); i++)
  {
    for (long unsigned int j = 0; j < rrt_obj_.all_wps_[i].size(); j++)
    {
      ros::Duration(0.5).sleep();
      rosplane_msgs::Waypoint new_waypoint;

      new_waypoint.w[0] = rrt_obj_.all_wps_[i][j].N;
      new_waypoint.w[1] = rrt_obj_.all_wps_[i][j].E;
      new_waypoint.w[2] = rrt_obj_.all_wps_[i][j].D;

      new_waypoint.Va_d = 20.0; // TODO find a good initial spot for this Va
      if (i == 0 && j == 0)
        new_waypoint.set_current = true;
      else
        new_waypoint.set_current = false;
      new_waypoint.clear_wp_list = false;
      waypoint_publisher_.publish(new_waypoint);
    }
  }
  res.success = true;
  return true;
}
std::vector<std::vector<double> > RosPathPlanner::filletMavPath(std::vector<double> xwps,   std::vector<double> ywps,\
                                                                  std::vector<double> dwps,std::vector<double> xwpsAll,\
                                                               std::vector<double> ywpsAll, std::vector<double> dwpsAll)
{
  long unsigned int wp_index = 1;
  std::vector<std::vector<double> > allWPS_plus_arc;
  for (int i = 0; i < xwps.size(); i++)
  {
    long unsigned int j = wp_index;
    while (xwps[i] != xwpsAll[j] || ywps[i] != ywpsAll[j] || dwps[i] != dwpsAll[j])
        j = j + 1;
    // Figure out the points to define a fillet path
    std::vector<double> x_path_data, y_path_data, d_path_data;
    for (int k = wp_index-1; k < j+1; k++)
    {
      x_path_data.push_back(xwpsAll[k]);
      y_path_data.push_back(ywpsAll[k]);
      d_path_data.push_back(dwpsAll[k]);
    }
    std::vector<std::vector<double> > path_data = filletPath(x_path_data, y_path_data, d_path_data);
    for (int k = 0; k < path_data.size(); k++)
    {
      allWPS_plus_arc.push_back(path_data[k]);
    }
    wp_index = j + 1;
  }
  return allWPS_plus_arc;
}
std::vector<std::vector<double> > RosPathPlanner::filletPath(std::vector<double> x_path_data,\
                                                              std::vector<double> y_path_data,\
                                                              std::vector<double> d_path_data)
{
  std::vector<std::vector<double> > path_data_new;
  std::vector<double> p;
  p.push_back(x_path_data[0]);
  p.push_back(y_path_data[0]);
  p.push_back(d_path_data[0]);
  path_data_new.push_back(p);
  p.clear();
  double par[3] = { };
  double mid[3] = { };
  double nex[3] = { };
  if (x_path_data.size() > 2)
  {
    for (int i  = 1; i < x_path_data.size() - 1; i++)
    {
      par[0] = x_path_data[i-1];
      par[1] = y_path_data[i-1];
      par[2] = d_path_data[i-1];
      mid[0] = x_path_data[i];
      mid[1] = y_path_data[i];
      mid[2] = d_path_data[i];
      nex[0] = x_path_data[i+1];
      nex[1] = y_path_data[i+1];
      nex[2] = d_path_data[i+1];
      double pe[3] = { };
      double ps[3] = { };
      double a_dot_b = (par[1] - mid[1])*(nex[1] - mid[1]) + (par[0] - mid[0])*(nex[0] - mid[0]) + (par[2] - mid[2])*(nex[2] - mid[2]);
      double A = sqrt(pow(par[1] - mid[1], 2) + pow(par[0] - mid[0], 2) + pow(par[2] - mid[2], 2));
      double B = sqrt(pow(nex[0] - mid[0], 2) + pow(nex[1] - mid[1], 2) + pow(nex[2] - mid[2], 2));
      double a_dot_b_D_AB = (a_dot_b) / (A*B);
      if (a_dot_b_D_AB <= -1 && a_dot_b_D_AB > -1-0.001)
          a_dot_b_D_AB = -1 + 0.0000001;
      double Fangle = acos(a_dot_b_D_AB);
      double distance_in = input_file_.turn_radius / tan(Fangle / 2.0);
       // Notice this equation was written incorrectly in the UAV book
       //sqrt(turn_radius*turn_radius / sin(Fangle / 2.0) / sin(Fangle / 2.0) - turn_radius*turn_radius);
      double theta = atan2(nex[0] - mid[0], nex[1] - mid[1]);
      pe[0] = (mid[0]) + sin(theta)*distance_in;
      pe[1] = (mid[1]) + cos(theta)*distance_in;
      pe[2] = mid[2];
      double gamma = atan2(par[0] - mid[0], par[1] - mid[1]);
      ps[0] = (mid[0]) + sin(gamma)*distance_in;
      ps[1] = (mid[1]) + cos(gamma)*distance_in;
      ps[2] = mid[2];
      // Find out whether it is going to the right (cw) or going to the left (ccw)
      // Use the cross product to see if it is cw or ccw
      double cross_product = ((mid[1] - ps[1])*(pe[0] - mid[0]) - (mid[0] - ps[0])*(pe[1] - mid[1]));
      bool ccw = false;
      if (cross_product < 0)
        ccw = false;
      else
        ccw = true;
      double cp[3] = { };
      std::vector<double> Nc, Ec;
      if (ccw)
      {
        cp[0] = (mid[0]) + sin(gamma - Fangle / 2.0)*input_file_.turn_radius / sin(Fangle / 2.0);
        cp[1] = (mid[1]) + cos(gamma - Fangle / 2.0)*input_file_.turn_radius / sin(Fangle / 2.0);
        cp[2] = mid[2];
        std::vector<std::vector<double > > NcEc =arc(cp[0],cp[1],input_file_.turn_radius,\
                                                     gamma+M_PI/2.0,theta-M_PI/2.0);
        Nc = NcEc[0];
        Ec = NcEc[1];
      }
      else
      {
        cp[0] = (mid[0]) + sin(gamma + Fangle / 2.0)*input_file_.turn_radius / sin(Fangle / 2.0);
        cp[1] = (mid[1]) + cos(gamma + Fangle / 2.0)*input_file_.turn_radius / sin(Fangle / 2.0);
        cp[2] = mid[2];
        if (gamma + 3.0/2.0*M_PI > M_PI/2.0)
            gamma = gamma-2.0*M_PI;
        // double sA = theta + M_PI/2.0;
        // double eA = gamma + 3.0/2.0*M_PI;
        std::vector<std::vector<double > > NcEc = arc(cp[0],cp[1],input_file_.turn_radius,theta+M_PI/2.0,\
                                                      gamma + 3.0/2.0*M_PI);
        Nc = NcEc[0];
        Ec = NcEc[1];
      }
      std::vector<double> Dc (Nc.size(),cp[2]);
      if (ccw == false)
      {
        std::reverse(Nc.begin(),Nc.end());
        std::reverse(Ec.begin(),Ec.end());
        std::reverse(Dc.begin(),Dc.end());
      }
      for (int uu = 0; uu < Nc.size(); uu++)
      {
        p.push_back(Nc[uu]);
        p.push_back(Ec[uu]);
        p.push_back(Dc[uu]);
        path_data_new.push_back(p);
        p.clear();
      }
    }
  }
  else
  {
    for (int uu = 1; uu < x_path_data.size(); uu++)
    {
      p.push_back(x_path_data[uu]);
      p.push_back(y_path_data[uu]);
      p.push_back(d_path_data[uu]);
      path_data_new.push_back(p);
      p.clear();
    }
  }
  p.push_back(x_path_data.back());
  p.push_back(y_path_data.back());
  p.push_back(d_path_data.back());
  path_data_new.push_back(p);
  p.clear();
  return path_data_new;
}
std::vector<std::vector<double > > RosPathPlanner::arc(double N, double E, double r, double aS, double aE)
{
  std::vector<double> Nc, Ec;
  while (aE < aS)
    aE += 2.0*M_PI;
  if (aE - aS == 0.0)
  {
    Ec.push_back(r*cos(aS)+ E);
    Nc.push_back(r*sin(aS)+ N);
  }
  for (float th = aS; th <= aE; th += M_PI/35.0)
  {
    Ec.push_back(r*cos(th)+ E);
    Nc.push_back(r*sin(th)+ N);
  }
  std::vector<std::vector<double> > NcEc;
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
