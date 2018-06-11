#include <theseus/rrt_plotter.h>

namespace theseus
{
rrtPlotter::rrtPlotter() :
  nh_(ros::NodeHandle())
{
  marker_pub_                  = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  ground_pub_                  = nh_.advertise<visualization_msgs::Marker>("groundstation/visualization_marker", 10);
  path_id_                     = 0;
  pWPS_id_                     = 0;
  odom_mkr_.header.frame_id    = "/local_ENU";
  odom_mkr_.ns                 = "plane_odom";
  odom_mkr_.type               = visualization_msgs::Marker::POINTS;
  odom_mkr_.action             = visualization_msgs::Marker::ADD;
  odom_mkr_.pose.orientation.x = 0.0;
  odom_mkr_.pose.orientation.y = 0.0;
  odom_mkr_.pose.orientation.z = 0.0;
  odom_mkr_.pose.orientation.w = 1.0;
  odom_mkr_.color.r            = 1.0f;
  odom_mkr_.color.g            = 1.0f;
  odom_mkr_.color.b            = 1.0f;
  odom_mkr_.color.a            = 1.0;
  odom_mkr_.lifetime           = ros::Duration();
  odom_mkr_.scale.x            = 5.0; // point width
  odom_mkr_.scale.y            = 5.0; // point width
  increase_path_id_            = true;

  planned_path_mkr_.header.frame_id = "/local_ENU";
  planned_path_mkr_.ns   = "planned_path";
  uint32_t lis           = visualization_msgs::Marker::LINE_STRIP;
  planned_path_mkr_.type = lis;
  planned_path_mkr_.action = visualization_msgs::Marker::ADD;
  planned_path_mkr_.pose.orientation.x = 0.0;
  planned_path_mkr_.pose.orientation.y = 0.0;
  planned_path_mkr_.pose.orientation.z = 0.0;
  planned_path_mkr_.pose.orientation.w = 1.0;
  planned_path_mkr_.color.r    = clr.green.N;
  planned_path_mkr_.color.g    = clr.green.E;
  planned_path_mkr_.color.b    = clr.green.D;
  planned_path_mkr_.color.a    = 1.0;
  planned_path_mkr_.scale.x    = 8.0; // line width
  planned_path_mkr_.lifetime   = ros::Duration();
  planned_path_mkr_.id         = 0;

  mobs_mkr_.header.frame_id    = "/local_ENU";
  mobs_mkr_.ns                 = "moving_obstacle";
  mobs_mkr_.type               = visualization_msgs::Marker::SPHERE;
  mobs_mkr_.action             = visualization_msgs::Marker::ADD;
  mobs_mkr_.pose.orientation.x = 0.0;
  mobs_mkr_.pose.orientation.y = 0.0;
  mobs_mkr_.pose.orientation.z = 0.0;
  mobs_mkr_.pose.orientation.w = 1.0;
  mobs_mkr_.color.r            = clr.red.N;
  mobs_mkr_.color.g            = clr.red.E;
  mobs_mkr_.color.b            = clr.red.D;
  mobs_mkr_.color.a            = 0.7;
  mobs_mkr_.lifetime           = ros::Duration();
  mobs_mkr_.id                 = 0;

  display_on_judges_map_ = false;
}
rrtPlotter::~rrtPlotter()
{

}
void rrtPlotter::displayMap(map_s map)
{
  ROS_INFO("Displaying Map");
  visualization_msgs::Marker obs_mkr, bds_mkr, run_mkr, run_mkr2;

  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  obs_mkr.header.frame_id = bds_mkr.header.frame_id = run_mkr.header.frame_id = run_mkr2.header.frame_id = "/local_ENU";
  // Set the namespace and id for this obs_mkr.  This serves to create a unique ID
  // Any obs_mkr sent with the same namespace and id will overwrite the old one
  obs_mkr.ns            = "static_obstacle";
  bds_mkr.ns            = "boundaries";
  run_mkr.ns            = "runway";
  run_mkr2.ns           = "runway2";
  uint32_t cyl          = visualization_msgs::Marker::CYLINDER;
  uint32_t lis          = visualization_msgs::Marker::LINE_STRIP;
  obs_mkr.type          = cyl;
  bds_mkr.type          = lis;
  run_mkr.type          = lis;
  run_mkr2.type         = lis;
  // Set the obs_mkr action.  Options are ADD (Which is really create or modify), DELETE, and new in ROS Indigo: 3 (DELETEALL)
  obs_mkr.action = bds_mkr.action  = run_mkr.action = run_mkr2.action = visualization_msgs::Marker::ADD;
  obs_mkr.pose.orientation.x  = bds_mkr.pose.orientation.x = run_mkr.pose.orientation.x = run_mkr2.pose.orientation.x = 0.0;
  obs_mkr.pose.orientation.y  = bds_mkr.pose.orientation.y = run_mkr.pose.orientation.y = run_mkr2.pose.orientation.y = 0.0;
  obs_mkr.pose.orientation.z  = bds_mkr.pose.orientation.z = run_mkr.pose.orientation.z = run_mkr2.pose.orientation.z = 0.0;
  obs_mkr.pose.orientation.w  = bds_mkr.pose.orientation.w = run_mkr.pose.orientation.w = run_mkr2.pose.orientation.w = 1.0;
  // Set the color -- be sure to set alpha to something non-zero!
  obs_mkr.color.r             = 1.0f;
  obs_mkr.color.g             = 0.0f;
  obs_mkr.color.b             = 0.0f;
  obs_mkr.color.a             = 0.9;
  bds_mkr.color.r             = 1.0f;
  bds_mkr.color.g             = 0.0f;
  bds_mkr.color.b             = 0.0f;
  bds_mkr.color.a             = 1.0;
  run_mkr.color.r             = 0.0f;
  run_mkr.color.g             = 1.0f;
  run_mkr.color.b             = 1.0f;
  run_mkr.color.a             = 1.0;
  run_mkr2.color.r            = 0.0f;
  run_mkr2.color.g            = 1.0f;
  run_mkr2.color.b            = 1.0f;
  run_mkr2.color.a            = 1.0;
  obs_mkr.lifetime = bds_mkr.lifetime  = run_mkr.lifetime = run_mkr2.lifetime = ros::Duration();

  int id = 0;
  obs_mkr.header.stamp = ros::Time::now();
  ROS_INFO("Number of Cylinders: %lu", map.cylinders.size());
  for (long unsigned int i = 0; i <  map.cylinders.size(); i++)
  {
    obs_mkr.id = id++;
    obs_mkr.pose.position.x    = map.cylinders[i].E;     // Center X position
    obs_mkr.pose.position.y    = map.cylinders[i].N;     // Center Y position
    obs_mkr.pose.position.z    = map.cylinders[i].H/2.0; // Center Z position
    // Set the scale of the obs_mkr -- 1x1x1 here means 1m on a side
    obs_mkr.scale.x = map.cylinders[i].R*2.0; // Diameter in x direction
    obs_mkr.scale.y = map.cylinders[i].R*2.0; // Diameter in y direction
    obs_mkr.scale.z = map.cylinders[i].H;     // Height
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
    ground_pub_.publish(obs_mkr);
    sleep(0.05); // apparently needs a small delay otherwise rviz can't keep up?
  }

  // primary waypoints
  displayPrimaryWaypoints(map.wps);

  // Boundaries
  bds_mkr.header.stamp = ros::Time::now();
  bds_mkr.id           =  0;
  bds_mkr.scale.x      =  15.0; // line width
  ROS_INFO("Number of Boundary Points: %lu",  map.boundary_pts.size());
  for (long unsigned int i = 0; i < map.boundary_pts.size(); i++)
  {
    geometry_msgs::Point p;
    p.y = map.boundary_pts[i].N;
    p.x = map.boundary_pts[i].E;
    p.z = 0.0;
    bds_mkr.points.push_back(p);
  }
  geometry_msgs::Point p0;
  p0.y = map.boundary_pts[0].N;
  p0.x = map.boundary_pts[0].E;
  p0.z = 0.0;
  bds_mkr.points.push_back(p0);
  p0.y = map.boundary_pts[1].N;
  p0.x = map.boundary_pts[1].E;
  p0.z = 0.0;
  bds_mkr.points.push_back(p0);
  marker_pub_.publish(bds_mkr);
  sleep(0.05);
  ground_pub_.publish(bds_mkr);

  // Runway
  run_mkr.header.stamp = run_mkr2.header.stamp = ros::Time::now();
  run_mkr.id           = run_mkr2.id           = 0;
  run_mkr.scale.x      = run_mkr2.scale.x      = 12.0; // line width
  geometry_msgs::Point p;

  // Webster field
  p.y = 851.282091;
  p.x = -671.665872;
  p.z = 5.907706;
  NED_s r1(p.y, p.x, -p.z);
  run_mkr.points.push_back(p);
  p.y = -323.669996;
  p.x =  253.258923;
  p.z = 5.986742;
  NED_s r2(p.y, p.x, -p.z);
  run_mkr.points.push_back(p);
  marker_pub_.publish(run_mkr);
  ground_pub_.publish(run_mkr);
  run_mkr.id++;
  sleep(0.05);
  // run_mkr.points.clear(); // this is bad. don't clear this
  p.y = -246.621673;
  p.x = -552.535779;
  p.z = 5.971316;
  NED_s r3(p.y, p.x, -p.z);
  run_mkr2.points.push_back(p);
  p.y = 361.343377;
  p.x = 811.429686;
  p.z = 5.938186;
  NED_s r4(p.y, p.x, -p.z);
  run_mkr2.points.push_back(p);
  marker_pub_.publish(run_mkr2);
  ground_pub_.publish(run_mkr2);
  run_mkr2.id++;
  sleep(0.05);
  // run_mkr.points.clear();

  // elberta
  // p.y = -79.073785;
  // p.x = 41.016782;
  // p.z = 5.999377;
  // NED_s r5(p.y, p.x, -p.z);
  // run_mkr.points.push_back(p);
  // p.y = 568.993663;
  // p.x = -770.280935;
  // p.z = 5.928122;
  // NED_s r6(p.y, p.x, -p.z);
  // run_mkr.points.push_back(p);
  // marker_pub_.publish(run_mkr);
  // ground_pub_.publish(run_mkr);
  // run_mkr.id++;
  // sleep(0.05);
  // run_mkr.points.clear();
  ROS_DEBUG("finished displayMap");


  // ROS_DEBUG("runway 1 chi %f %f", (r1 - r2).getChi(), (r2 - r1).getChi());
  // ROS_DEBUG("runway 2 chi %f %f", (r3 - r4).getChi(), (r4 - r3).getChi());
  // ROS_DEBUG("runway 3 chi %f %f", (r5 - r6).getChi(), (r6 - r5).getChi());
}
void rrtPlotter::displayPrimaryWaypoints(std::vector<NED_s> wps)
{
  ROS_INFO("Displaying Waypoints");
  visualization_msgs::Marker pWPS_mkr;

  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  pWPS_mkr.header.frame_id = "/local_ENU";
  // Set the namespace and id for this obs_mkr.  This serves to create a unique ID
  // Any obs_mkr sent with the same namespace and id will overwrite the old one
  pWPS_mkr.ns           = "primary_wps";
  uint32_t pts          = visualization_msgs::Marker::POINTS;
  pWPS_mkr.type         = pts;
  // Set the obs_mkr action.  Options are ADD (Which is really create or modify), DELETE, and new in ROS Indigo: 3 (DELETEALL)
  pWPS_mkr.action = visualization_msgs::Marker::ADD;
  pWPS_mkr.pose.orientation.x = 0.0;
  pWPS_mkr.pose.orientation.y = 0.0;
  pWPS_mkr.pose.orientation.z = 0.0;
  pWPS_mkr.pose.orientation.w = 1.0;
  // Set the color -- be sure to set alpha to something non-zero!
  pWPS_mkr.color.r            = 1.0f;
  pWPS_mkr.color.g            = 1.0f;
  pWPS_mkr.color.b            = 0.0f;
  pWPS_mkr.color.a            = 1.0;
  pWPS_mkr.lifetime = ros::Duration();

  // primary waypoints
  pWPS_mkr.header.stamp = ros::Time::now();
  pWPS_mkr.id           =  20;
  pWPS_mkr.scale.x      =  25.0; // point width
  pWPS_mkr.scale.y      =  25.0; // point height
  ROS_INFO("Number of Waypoints: %lu", wps.size());
  for (long unsigned int i = 0; i < wps.size(); i++)
  {
    geometry_msgs::Point p;
    p.y =  wps[i].N;
    p.x =  wps[i].E;
    p.z = -wps[i].D;
    pWPS_mkr.points.push_back(p);
  }
  marker_pub_.publish(pWPS_mkr);
  ground_pub_.publish(pWPS_mkr);
  sleep(0.05);
  ROS_DEBUG("finished displaying waypoints");
}
void rrtPlotter::odomCallback(geometry_msgs::Point p)
{
  odom_mkr_.header.stamp = ros::Time::now();
  odom_mkr_.points.push_back(p);
  marker_pub_.publish(odom_mkr_);
  // ground_pub_.publish(odom_mkr_);
}
void rrtPlotter::mobsCallback(std::vector<NED_s> mobs_in, std::vector<float> radius)
{
  mobs_mkr_.header.stamp = ros::Time::now();

  for (int i = 0; i < mobs_in.size(); i++)
  {
    mobs_mkr_.scale.x         = radius[i]*2.0f;
    mobs_mkr_.scale.y         = radius[i]*2.0f;
    mobs_mkr_.scale.z         = radius[i]*2.0f;
    mobs_mkr_.pose.position.x = mobs_in[i].E;
    mobs_mkr_.pose.position.y = mobs_in[i].N;
    mobs_mkr_.pose.position.z = -mobs_in[i].D;
    mobs_mkr_.id = i;
    marker_pub_.publish(mobs_mkr_);
    // ground_pub_.publish(mobs_mkr_);

    ros::Duration(0.01).sleep();
  }
}
void rrtPlotter::pingBoundaries()
{
  marker_pub_.publish(bds_mkr_);
  ground_pub_.publish(bds_mkr_);
}
void rrtPlotter::addFinalPath(NED_s ps, std::vector<NED_s> stuff_in)
{
  planned_path_mkr_.points.clear();
  std::vector<NED_s> path;
  path.push_back(ps);
  ROS_DEBUG("PATH N: %f E: %f D: %f", ps.N, ps.E, ps.D);
  for (int it = 0; it < stuff_in.size(); it++)
  {
    path.push_back(stuff_in[it]);
    ROS_DEBUG("PATH N: %f E: %f D: %f", stuff_in[it].N, stuff_in[it].E, stuff_in[it].D);
  }
  std::vector<NED_s> vis_path;
  vis_path.push_back(path[0]);
  for (int i = 1; i < path.size() - 1; i++)
  {
    fillet_s fil;
    fil.calculate(path[i - 1], path[i], path[i + 1], input_file_.turn_radius);
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
    NED_s pos;
    for (int j = 0; j < Nc.size(); j++)
    {
      pos.N = Nc[j];
      pos.E = Ec[j];
      pos.D = fil.c.D;
      vis_path.push_back(pos);
    }
    vis_path.push_back(fil.z2);
  }
  vis_path.push_back(path[path.size() - 1]);

  for (int i = 0; i < vis_path.size(); i++)
  {
    geometry_msgs::Point p;
    p.x =  vis_path[i].E;
    p.y =  vis_path[i].N;
    p.z = -vis_path[i].D;
    planned_path_mkr_.points.push_back(p);
  }
  marker_pub_.publish(planned_path_mkr_);
  ground_pub_.publish(planned_path_mkr_);
}
void rrtPlotter::pingPath()
{
  marker_pub_.publish(planned_path_mkr_);
}
void rrtPlotter::displayPath(NED_s ps, std::vector<NED_s> path, NED_s color, float width)
{
  std::vector<NED_s> temp_neds;
  temp_neds.push_back(ps);
  ROS_DEBUG("PATH N: %f E: %f D: %f", ps.N, ps.E, ps.D);
  for (int it = 0; it < path.size(); it++)
  {
    temp_neds.push_back(path[it]);
    ROS_DEBUG("PATH N: %f E: %f D: %f", path[it].N, path[it].E, path[it].D);
  }
  displayPath(temp_neds, color, width);
}
void rrtPlotter::displayPath(NED_s ps, std::vector<node*> path, NED_s color, float width)
{
  std::vector<NED_s> temp_neds;
  temp_neds.push_back(ps);
  ROS_DEBUG("PATH N: %f E: %f D: %f", ps.N, ps.E, ps.D);
  for (int it = 0; it < path.size(); it++)
  {
    temp_neds.push_back(path[it]->p);
    ROS_DEBUG("PATH N: %f E: %f D: %f", path[it]->p.N, path[it]->p.E, path[it]->p.D);
  }
  displayPath(temp_neds, color, width);
}
void rrtPlotter::displayPath(std::vector<node*> path, NED_s color, float width)
{
  std::vector<NED_s> temp_neds;
  for (int it = 0; it < path.size(); it++)
  {
    temp_neds.push_back(path[it]->p);
    ROS_DEBUG("PATH N: %f E: %f D: %f", path[it]->p.N, path[it]->p.E, path[it]->p.D);
  }
  displayPath(temp_neds, color, width);
}
void rrtPlotter::displayBoundaries(map_s map)
{
  bds_mkr_.header.frame_id = "/local_ENU";
  bds_mkr_.ns           = "boundaries";
  uint32_t lis          = visualization_msgs::Marker::LINE_STRIP;
  bds_mkr_.type         = lis;
  bds_mkr_.action  = visualization_msgs::Marker::ADD;
  bds_mkr_.pose.orientation.x = 0.0;
  bds_mkr_.pose.orientation.y = 0.0;
  bds_mkr_.pose.orientation.z = 0.0;
  bds_mkr_.pose.orientation.w = 1.0;
  bds_mkr_.color.r            = 1.0f;
  bds_mkr_.color.g            = 0.0f;
  bds_mkr_.color.b            = 0.0f;
  bds_mkr_.color.a            = 1.0;
  bds_mkr_.lifetime  = ros::Duration();

  int id = 0;
  // Boundaries
  bds_mkr_.header.stamp = ros::Time::now();
  bds_mkr_.id           =  0;
  bds_mkr_.scale.x      =  15.0; // line width
  for (long unsigned int i = 0; i < map.boundary_pts.size(); i++)
  {
    geometry_msgs::Point p;
    p.y = map.boundary_pts[i].N;
    p.x = map.boundary_pts[i].E;
    p.z = 0.0;
    bds_mkr_.points.push_back(p);
  }
  geometry_msgs::Point p0;
  p0.y = map.boundary_pts[0].N;
  p0.x = map.boundary_pts[0].E;
  p0.z = 0.0;
  bds_mkr_.points.push_back(p0);
  p0.y = map.boundary_pts[1].N;
  p0.x = map.boundary_pts[1].E;
  p0.z = 0.0;
  bds_mkr_.points.push_back(p0);
  marker_pub_.publish(bds_mkr_);
  ground_pub_.publish(bds_mkr_);
  sleep(0.05);
}
void rrtPlotter::displayPath(std::vector<NED_s> path, NED_s color, float width)
{
  visualization_msgs::Marker aWPS_mkr, planned_path_mkr;
  // if (path_id_ == 1)
  //   clearRVizPaths();
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  planned_path_mkr.header.frame_id = aWPS_mkr.header.frame_id = "/local_ENU";
  // Set the namespace and id for this obs_mkr.  This serves to create a unique ID
  // Any obs_mkr sent with the same namespace and id will overwrite the old one
  planned_path_mkr.ns   = "planned_path";
  aWPS_mkr.ns            = "all_wps";
  uint32_t cyl           = visualization_msgs::Marker::CYLINDER;
  uint32_t pts           = visualization_msgs::Marker::POINTS;
  uint32_t lis           = visualization_msgs::Marker::LINE_STRIP;
  planned_path_mkr.type = lis;
  aWPS_mkr.type          = pts;
  // Set the obs_mkr action.  Options are ADD (Which is really create or modify), DELETE, and new in ROS Indigo: 3 (DELETEALL)
  planned_path_mkr.action = aWPS_mkr.action = visualization_msgs::Marker::ADD;
  planned_path_mkr.pose.orientation.x = aWPS_mkr.pose.orientation.x = 0.0;
  planned_path_mkr.pose.orientation.y = aWPS_mkr.pose.orientation.y = 0.0;
  planned_path_mkr.pose.orientation.z = aWPS_mkr.pose.orientation.z = 0.0;
  planned_path_mkr.pose.orientation.w = aWPS_mkr.pose.orientation.w = 1.0;
  // Set the color -- be sure to set alpha to something non-zero!
  planned_path_mkr.color.r    = color.N;
  planned_path_mkr.color.g    = color.E;
  planned_path_mkr.color.b    = color.D;
  planned_path_mkr.color.a    = 1.0;
  aWPS_mkr.scale.x             = 10.0; // point width
  aWPS_mkr.scale.y             = 10.0; // point height
  planned_path_mkr.scale.x    = width; // line width

  aWPS_mkr.color.r             = 0.0f;
  aWPS_mkr.color.g             = 0.0f;
  aWPS_mkr.color.b             = 1.0f;
  aWPS_mkr.color.a             = 1.0;
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
    for (long unsigned int i = 0; i < path.size(); i++)
  	{
      geometry_msgs::Point p;
      p.y =  path[i].N;
      p.x =  path[i].E;
      p.z = -path[i].D;
      aWPS_mkr.points.push_back(p);
    }
    marker_pub_.publish(aWPS_mkr);
    sleep(1.0);
  }

  // Plot desired path
  planned_path_mkr.header.stamp = ros::Time::now();
  // ROS_DEBUG("path_id_ %i", path_id_);
  if (increase_path_id_)
    planned_path_mkr.id         = path_id_++;
  else
    planned_path_mkr.id         = path_id_;
  std::vector<NED_s> vis_path;
  vis_path.push_back(path[0]);
  for (int i = 1; i < path.size() - 1; i++)
  {
    fillet_s fil;
    fil.calculate(path[i - 1], path[i], path[i + 1], input_file_.turn_radius);
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
    NED_s pos;
    for (int j = 0; j < Nc.size(); j++)
    {
      pos.N = Nc[j];
      pos.E = Ec[j];
      pos.D = fil.c.D;
      vis_path.push_back(pos);
    }
    vis_path.push_back(fil.z2);
  }
  vis_path.push_back(path[path.size() - 1]);

  for (int i = 0; i < vis_path.size(); i++)
  {
    geometry_msgs::Point p;
    p.x =  vis_path[i].E;
    p.y =  vis_path[i].N;
    p.z = -vis_path[i].D;
    planned_path_mkr.points.push_back(p);
  }
  marker_pub_.publish(planned_path_mkr);

  if (display_on_judges_map_)
  {
    ground_pub_.publish(planned_path_mkr);
    display_on_judges_map_ = false;
  }
  sleep(0.05);
}
void rrtPlotter::drawCircle(NED_s cp, float r)
{
  std::vector<std::vector<float> > points = arc(cp.N, cp.E, r, 0.0f, 2.0f*M_PI + 0.1f);
  ROS_INFO("Displaying Circle");
  visualization_msgs::Marker pWPS_mkr, cir_mkr;

  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  pWPS_mkr.header.frame_id = cir_mkr.header.frame_id = "/local_ENU";
  // Set the namespace and id for this obs_mkr.  This serves to create a unique ID
  // Any obs_mkr sent with the same namespace and id will overwrite the old one
  pWPS_mkr.ns           = "cp";
  cir_mkr.ns            = "path";
  uint32_t pts          = visualization_msgs::Marker::POINTS;
  uint32_t lis          = visualization_msgs::Marker::LINE_STRIP;
  pWPS_mkr.type         = pts;
  cir_mkr.type          = lis;
  // Set the obs_mkr action.  Options are ADD (Which is really create or modify), DELETE, and new in ROS Indigo: 3 (DELETEALL)
  pWPS_mkr.action = cir_mkr.action  = visualization_msgs::Marker::ADD;
  pWPS_mkr.pose.orientation.x = cir_mkr.pose.orientation.x = 0.0;
  pWPS_mkr.pose.orientation.y = cir_mkr.pose.orientation.y = 0.0;
  pWPS_mkr.pose.orientation.z = cir_mkr.pose.orientation.z = 0.0;
  pWPS_mkr.pose.orientation.w = cir_mkr.pose.orientation.w = 1.0;
  // Set the color -- be sure to set alpha to something non-zero!
  pWPS_mkr.color.r            = 0.5f;
  pWPS_mkr.color.g            = 0.0f;
  pWPS_mkr.color.b            = 0.5f;
  pWPS_mkr.color.a            = 1.0;
  cir_mkr.color.r             = 0.5f;
  cir_mkr.color.g             = 0.5f;
  cir_mkr.color.b             = 0.0f;
  cir_mkr.color.a             = 1.0;
  pWPS_mkr.lifetime = cir_mkr.lifetime  = ros::Duration();

  ROS_INFO("checking for subscribers");
  int id = 0;
  while (marker_pub_.getNumSubscribers() < 1)
  {
    if (!ros::ok())
      return;
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }
  ROS_INFO("found subscribers");
  // primary waypoints
  pWPS_mkr.header.stamp = ros::Time::now();
  pWPS_mkr.id           =  0;
  pWPS_mkr.scale.x      =  10.0; // point width
  pWPS_mkr.scale.y      =  10.0; // point height
  geometry_msgs::Point p;
  p.y =  cp.N;
  p.x =  cp.E;
  p.z = -cp.D;
  pWPS_mkr.points.push_back(p);
  marker_pub_.publish(pWPS_mkr);
  sleep(0.05);

  // Circle
  cir_mkr.header.stamp = ros::Time::now();
  cir_mkr.id           =  0;
  cir_mkr.scale.x      =  10.0; // line width
  std::vector<float> Nc = points[0];
  std::vector<float> Ec = points[1];
  NED_s pos;
  for (int j = 0; j < Nc.size(); j++)
  {
    geometry_msgs::Point p;
    p.y = Nc[j];
    p.x = Ec[j];
    p.z = -cp.D;
    cir_mkr.points.push_back(p);
  }
  marker_pub_.publish(cir_mkr);
  ground_pub_.publish(cir_mkr);
  sleep(0.05);
  ROS_INFO("End of drawCircle");
}
std::vector<std::vector<float > > rrtPlotter::arc(float N, float E, float r, float aS, float aE)
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
void rrtPlotter::clearRViz(map_s map)
{
  visualization_msgs::Marker clear_mkr;
  clear_mkr.action = visualization_msgs::Marker::DELETEALL;
  marker_pub_.publish(clear_mkr);
  ground_pub_.publish(clear_mkr);
  displayMap(map);
  path_id_ = 0;
}
void rrtPlotter::clearRViz(map_s map, std::vector<NED_s> path, NED_s color, float width)
{
  clearRViz(map);
  path_id_ = 1;
  if (path.size() > 0)
    displayPath(path, color, width);
  path_id_ = 0;
}
void rrtPlotter::displayTree(node* root)
{
  fringe_.clear();
  addFringe(root);
  for (int j = 0; j < fringe_.size(); j++)
  {
    tree_path_.clear();
    addTreePath(root, fringe_[j]);
    tree_path_.push_back(root->p);
    std::reverse(tree_path_.begin(), tree_path_.end());
    displayPath(tree_path_, clr.gray, 2.5f);
    sleep(0.005);
  }
}
void rrtPlotter::addFringe(node* nin)
{
  if (nin->children.size() == 0)
    fringe_.push_back(nin);
  else
  {
    tree_path_.clear();
    for (unsigned int i = 0; i < nin->children.size(); i++)
      addFringe(nin->children[i]);
  }
}
void rrtPlotter::addTreePath(node* root, node* nin)
{
  if (nin->p != root->p)
  {
    tree_path_.push_back(nin->p);
    addTreePath(root, nin->parent);
  }
}
} // end namespace theseus
