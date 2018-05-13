#include <theseus/rrt_plotter.h>

namespace theseus
{
rrtPlotter::rrtPlotter() :
  nh_(ros::NodeHandle())
{
  marker_pub_                  = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  path_id_                     = 0;
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
  odom_mkr_.scale.x            = 15.0; // point width
  odom_mkr_.scale.y            = 15.0; // point width
  increase_path_id_            = true;

}
rrtPlotter::~rrtPlotter()
{

}
void rrtPlotter::displayMap(map_s map)
{
  ROS_INFO("Displaying Map");
  visualization_msgs::Marker obs_mkr, pWPS_mkr, bds_mkr;

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
  }

  // primary waypoints
  pWPS_mkr.header.stamp = ros::Time::now();
  pWPS_mkr.id           =  0;
  pWPS_mkr.scale.x      =  10.0; // point width
  pWPS_mkr.scale.y      =  10.0; // point height
  ROS_INFO("Number of Waypoints: %lu", map.wps.size());
  for (long unsigned int i = 0; i < map.wps.size(); i++)
  {
    geometry_msgs::Point p;
    p.y =  map.wps[i].N;
    p.x =  map.wps[i].E;
    p.z = -map.wps[i].D;
    pWPS_mkr.points.push_back(p);
  }
  marker_pub_.publish(pWPS_mkr);
  sleep(0.05);

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
  ROS_DEBUG("finished displayMap");
}
void rrtPlotter::odomCallback(geometry_msgs::Point p)
{
  odom_mkr_.header.stamp = ros::Time::now();
  odom_mkr_.points.push_back(p);
  marker_pub_.publish(odom_mkr_);
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
void rrtPlotter::displayPath(std::vector<NED_s> path, NED_s color, float width)
{
  visualization_msgs::Marker planned_path_mkr, aWPS_mkr;
  // if (path_id_ == 1)
  //   clearRVizPaths();
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
  planned_path_mkr.color.r    = color.N;
  planned_path_mkr.color.g    = color.E;
  planned_path_mkr.color.b    = color.D;
  planned_path_mkr.color.a    = 1.0;
  aWPS_mkr.scale.x            = 10.0; // point width
  aWPS_mkr.scale.y            = 10.0; // point height
  planned_path_mkr.scale.x    = width; // line width

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

  int id = 0;
  while (marker_pub_.getNumSubscribers() < 1)
  {
    if (!ros::ok())
      return;
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }
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
  sleep(0.05);
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
