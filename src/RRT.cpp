#include <theseus/RRT.h>

namespace theseus
{
RRT::RRT(map_s map_in, unsigned int seed) :
  nh_(ros::NodeHandle())// Setup the object
{
  if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
   ros::console::notifyLoggerLevelsChanged();
  }
  marker_pub_     = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  nh_.param<float>("pp/segment_length", segment_length_, 100.0);
  num_paths_      = 1;            // number of paths to solve between each waypoint
	RandGen rg_in(seed);            // Make a random generator object that is seeded
	rg_             = rg_in;        // Copy that random generator into the class.
  map_            = map_in;
  col_det_.newMap(map_in);
  last_path_id_   = 1;              // just used for debugging
  path_id_        = 1;
  ending_chi_     = 0.0f;
}
RRT::RRT()
{

}
RRT::~RRT()
{
	deleteTree();                          // Delete all of those tree pointer nodes
	// std::vector<node*>().swap(root_ptrs_); // Free the memory of the vector.
}
void RRT::solveStatic(NED_s pos, float chi0, bool direct_hit, bool landing)         // This function solves for a path in between the waypoinnts (2 Dimensional)
{
  std::vector<NED_s> all_rough_paths;

  direct_hit_      = direct_hit;
  path_clearance_  = input_file_.clearance;
  ROS_DEBUG("Starting RRT solver");
  clearForNewPath();
	initializeTree(pos, chi0);
  all_rough_paths.push_back(root_ptrs_[0]->p);
  taking_off_ = (-pos.D < input_file_.minFlyHeight);
  col_det_.taking_off_ = taking_off_;
  if (taking_off_) {ROS_DEBUG("taking_off_ on initial set is true");}
  else {ROS_DEBUG("taking_off_ on initial set is false");}
  printRRTSetup(pos, chi0);
  if (taking_off_ == false && direct_hit_ == true)
    createFan(root_ptrs_[0],root_ptrs_[0]->p, chi0, path_clearance_);
  long unsigned int iters_left = input_file_.iters_limit;

  for (unsigned int i = 0; i < map_.wps.size(); i++)
  {
    landing_now_ = landing;
    col_det_.landing_now_ = landing_now_;
    if (i > 0 &&taking_off_ == false && direct_hit_ == true)
      createFan(root_ptrs_[i],root_ptrs_[i]->p, (root_ptrs_[i]->p - root_ptrs_[i]->parent->p).getChi(), path_clearance_);
    ROS_DEBUG("Finding route to waypoint %lu", i + (long unsigned int) 1);
    path_clearance_        = input_file_.clearance;
    bool direct_connection = tryDirectConnect(root_ptrs_[i], root_ptrs_[i + 1], i);
    ROS_INFO("trying to connect to N %f, E %f, D %f", root_ptrs_[i + 1]->p.N, root_ptrs_[i + 1]->p.E, root_ptrs_[i + 1]->p.D);
    if (direct_connection == false)
    {
      int num_found_paths = 0;
      long unsigned int added_nodes = 0;
      ROS_DEBUG("Developing the tree");
      while (num_found_paths < num_paths_)
      {
        num_found_paths += developTree(i);
        added_nodes++;
        ROS_DEBUG("number of nodes %lu, %lu", added_nodes, iters_left);
        if ((float) added_nodes > iters_left/2.0f)
        {
          ROS_WARN("decreasing the clearance level");
          path_clearance_  = path_clearance_/2.0f;
          iters_left  = iters_left/2.0f;
          added_nodes = 0;
        }
      }
    }
    std::vector<node*> rough_path  = findMinimumPath(i);
    std::vector<node*> smooth_path = smoothPath(rough_path, i);
    addPath(smooth_path, i);
    if (-all_wps_[all_wps_.size() - 1].D > input_file_.minFlyHeight)
    {
      taking_off_ = false;
      col_det_.taking_off_ = false;
      ROS_DEBUG("taking off is false");
    }
    clearRVizPaths();
    clearRVizPaths();
    last_path_id_ = path_id_;
    path_id_ = 1; // just for debugging, displayPath();
    ROS_DEBUG("path_id_ reset, last_path_id_  = %i", last_path_id_);

    for (int it = 1; it < rough_path.size(); it++)
      all_rough_paths.push_back(rough_path[it]->p);
    if (landing_now_)
      break;
  }
  if (landing_now_)
  {

    for (int j = 1; j < map_.wps.size(); j++)
    {
      ROS_DEBUG("pushing back another waypoint");
      all_wps_.push_back(map_.wps[j]);
      ROS_DEBUG("N: %f, E: %f, D: %f", all_wps_.back().N, all_wps_.back().E, all_wps_.back().D);
      all_rough_paths.push_back(map_.wps[j]);
    }
  }
  // find a place to safely loiter?

  // Set up an extra straight line.
  // NED_s first_after, second_after;
  // float chi_last = (all_wps_[all_wps_.size() - 1] - all_wps_[all_wps_.size() - 2]).getChi();
  // first_after = (all_wps_[all_wps_.size() - 1] - all_wps_[all_wps_.size() - 2]).normalize()*300.0f + all_wps_[all_wps_.size() - 1];
  // second_after = (all_wps_[all_wps_.size() - 1] - all_wps_[all_wps_.size() - 2]).normalize()*500.0f + all_wps_[all_wps_.size() - 1];
  // all_wps_.push_back(first_after);
  // all_wps_.push_back(second_after);

  // for (int i = 0; i < map_.wps.size(); i++)
  //   ROS_DEBUG("WP %i: %f, %f, %f", i, map_.wps[i].N, map_.wps[i].E, map_.wps[i].D);

  // clearRVizPaths();
  path_id_ = 2;
  // displayPath(all_rough_paths, false);
  ending_point_ = all_wps_.back();
  ending_chi_   = (all_wps_.back() - all_wps_[all_wps_.size() - 2]).getChi();
  // ROS_FATAL("FINISHED THE RRT ALGORITHM");
  // sleep(10.0);
}


bool RRT::tryDirectConnect(node* ps, node* pe_node, unsigned int i)
{
  ROS_DEBUG("Attempting direct connect");
  float clearance = path_clearance_;
  node* start_of_line;
  if (ps->dontConnect) // then try one of the grand children
  {
    ROS_DEBUG("finding the grand child that is closes");
    start_of_line = findClosestNodeGChild(ps, pe_node->p);
  }
  else
  {
    ROS_DEBUG("using ps");
    start_of_line = ps;
  }
  ROS_DEBUG("checking the line");
  if (col_det_.checkLine(start_of_line->p, pe_node->p, clearance))
  {
    ROS_DEBUG("line passed");
    if (start_of_line->parent == NULL) // then this is the start
    {
      ROS_DEBUG("parent is null");
      float chi = (pe_node->p - start_of_line->p).getChi();
      ROS_DEBUG("checking after the waypoint");
      if (col_det_.checkAfterWP(pe_node->p, chi, clearance))
      {
        if (landing_now_)
        {
          // check to see if the change in chi is okay
          float chi1 = chi;
          float chi2 = (map_.wps.back() - map_.wps[map_.wps.size() - 2]).getChi() - M_PI;
          chi2 = chi2 - chi1;
          while (chi2 < -2.0f*M_PI)
            chi2 += 2.0f*M_PI;
          while (chi2 > 2.0f*M_PI)
            chi2 -= 2.0f*M_PI;
          if (chi2 < 15.0f*M_PI/180.0 && chi2 > -15.0f*M_PI/180.0)
          {
            ROS_DEBUG("too close of chi1 and chi2");
            return false;
          }
        }
        ROS_DEBUG("direct connection success 1");
        start_of_line->cost        = start_of_line->cost + (pe_node->p - start_of_line->p).norm();
        start_of_line->connects2wp = true;
        start_of_line->children.push_back(pe_node);
        most_recent_node_          = pe_node;
        return true;
      }
    }
    else
    {
      fillet_s fil;
      ROS_DEBUG("calculating fillet");
      bool fil_possible = fil.calculate(start_of_line->parent->p, start_of_line->p, pe_node->p, input_file_.turn_radius);
      fillet_s temp_fil = fil;
      float slope = atan2f(-1.0f*(fil.z1.D - start_of_line->fil.z2.D), sqrtf(powf(start_of_line->fil.z2.N - \
                           fil.z1.N, 2.0f) + powf(start_of_line->fil.z2.E - fil.z1.E, 2.0f)));
      if (slope < -1.0f*input_file_.max_descend_angle || slope > input_file_.max_climb_angle)
        return false;
      slope = atan2f(-1.0f*(pe_node->p.D - fil.z2.D), sqrtf(powf(fil.z2.N - pe_node->p.N, 2.0f) \
                     + powf(fil.z2.E - pe_node->p.E, 2.0f)));
      if (slope < -1.0f*input_file_.max_descend_angle || slope > input_file_.max_climb_angle)
        return false;
      temp_fil.w_im1 = fil.z1;
      ROS_DEBUG("cheking fillet");
      if (fil_possible && col_det_.checkFillet(temp_fil, clearance))
      {
        ROS_DEBUG("fillet checked out, now trying neighboring fillets");
        if (start_of_line->parent != NULL && start_of_line->fil.roomFor(fil) == false)
        {
          //printNode(start_of_line);
          ROS_DEBUG("failed direct connection because of neighboring fillets");
          return false;
        }
        float chi = (pe_node->p - start_of_line->p).getChi();
        ROS_DEBUG("checking after the waypoint");
        if (col_det_.checkAfterWP(pe_node->p, chi, clearance))
        {
          // check to see if the change in chi is okay
          float chi1 = chi;
          float chi2 = (map_.wps.back() - map_.wps[map_.wps.size() - 2]).getChi() - M_PI;
          chi2 = chi2 - chi1;
          while (chi2 < -2.0f*M_PI)
            chi2 += 2.0f*M_PI;
          while (chi2 > 2.0f*M_PI)
            chi2 -= 2.0f*M_PI;
          if (chi2 < 15.0f*M_PI/180.0 && chi2 > -15.0f*M_PI/180.0)
          {
            ROS_DEBUG("too close of chi1 and chi2, number 2");
            return false;
          }
          ROS_DEBUG("direct connection success 2");
          start_of_line->cost        = start_of_line->cost + (pe_node->p - start_of_line->p).norm() - fil.adj;
          start_of_line->connects2wp = true;
          start_of_line->children.push_back(pe_node);
          most_recent_node_          = pe_node;
          return true;
        }
      }
    }
  }
  ROS_DEBUG("direct connection failed");
  return false;
}
int RRT::developTree(unsigned int i)
{
  ROS_DEBUG("looking for next node");
  bool added_new_node = false;
  float clearance = path_clearance_;
  while (added_new_node == false)
  {
    // generate a good point to test
    NED_s random_point = randomPoint(i);
    float min_d        = INFINITY;
    node* closest_node = findClosestNode(root_ptrs_[i], random_point, root_ptrs_[i], &min_d);
    if (taking_off_ == false && landing_now_ == false)
      random_point.D     = redoRandomDownPoint(i,  closest_node->p.D); // this is so that more often a node passes the climb angle check
    NED_s test_point   = (random_point - closest_node->p).normalize()*segment_length_ + closest_node->p;
    added_new_node     = checkForCollision(closest_node, test_point, i, clearance, false);

    // std::vector<NED_s> temp_path;
    // if (closest_node->parent != NULL)
    //   temp_path.push_back(closest_node->fil.z2);
    // temp_path.push_back(closest_node->p);
    // temp_path.push_back(test_point);
    // displayPath(temp_path,true);

  }
  ROS_DEBUG("found a new node");
  // std::vector<NED_s> temp_path;
  // if (most_recent_node_->parent->parent != NULL)
  //   temp_path.push_back(most_recent_node_->parent->fil.z2);
  // temp_path.push_back(most_recent_node_->parent->p);
  // temp_path.push_back(most_recent_node_->p);
  // displayPath(temp_path,false);


  ROS_DEBUG("trying direct connect for the new node");
  bool connect_to_end = tryDirectConnect(most_recent_node_, root_ptrs_[i + 1], i);
  if (connect_to_end == true)
    return 1;
  else
    return 0;
}
std::vector<node*> RRT::findMinimumPath(unsigned int i)
{
  ROS_DEBUG("finding a minimum path");
  // recursively go through the tree to find the connector
  std::vector<node*> rough_path;
  float minimum_cost = INFINITY;
  node* almost_last  = findMinConnector(root_ptrs_[i], root_ptrs_[i], &minimum_cost);
  ROS_DEBUG("found a minimum path");
  root_ptrs_[i + 1]->parent  = almost_last;
  smooth_rts_[i + 1]->parent = almost_last;
  if (almost_last->parent != NULL)
  {
    fillet_s fil;
    bool fil_b = fil.calculate(almost_last->parent->p, almost_last->p, root_ptrs_[i + 1]->p, input_file_.turn_radius);
    root_ptrs_[i + 1]->fil    = fil;
    smooth_rts_[i + 1]->fil    = fil;
    // ROS_DEBUG("calculated fillet");
  }

	std::stack<node*> wpstack;
	node *current_node = root_ptrs_[i + 1];
  // //ROS_DEBUG("printing the root");
  // printNode(root_ptrs_[i]);
	while (current_node != root_ptrs_[i])
  {
    ROS_DEBUG("pushing parent");
		// printNode(current_node);
    wpstack.push(current_node);
		current_node = current_node->parent;
	}
  rough_path.push_back(root_ptrs_[i]);
  ROS_DEBUG("about to empty the stack");
	while (!wpstack.empty())
	{
    //ROS_DEBUG("pushing to rough_path");
		rough_path.push_back(wpstack.top());
		wpstack.pop();
	}
  //ROS_DEBUG("created rough path");
  return rough_path;
}
std::vector<node*> RRT::smoothPath(std::vector<node*> rough_path, int i)
{
  std::vector<NED_s> temp_neds;
  for (int it = 0; it < rough_path.size(); it++)
  {
    temp_neds.push_back(rough_path[it]->p);
    ROS_DEBUG("ROUGH PATH N: %f E: %f D: %f", rough_path[it]->p.N, rough_path[it]->p.E, rough_path[it]->p.D);
  }
  // displayPath(temp_neds, false);
  temp_neds.clear();
  std::vector<node*> new_path;

  rough_path.erase(rough_path.begin());
  return rough_path;
  ROS_ERROR("STARTING THE SMOOTHER");
  new_path.push_back(smooth_rts_[i]);
  // ROS_DEBUG("N: %f, E: %f, D: %f", new_path.back()->p.N, new_path.back()->p.E,new_path.back()->p.D);
  // printNode(new_path.back());
  std::vector<NED_s> temp_path; // used for plotting
  if (direct_hit_)
  {
    int ptr;
    if (i > 0)
    {
      ptr = 0;
      fillet_s fil1, fil2;
      bool passed1 = fil1.calculate(new_path.back()->parent->p, new_path.back()->p, rough_path[ptr + 1]->p, input_file_.turn_radius);
      bool passed2 = fil2.calculate(new_path.back()->p, rough_path[ptr + 1]->p, rough_path[ptr + 2]->p, input_file_.turn_radius);
      node *fake_child           = new node;
      node *normal_gchild        = new node;
      fake_child->p              = rough_path[ptr + 1]->p;
      fake_child->fil            = fil1;
      fake_child->parent         = new_path.back();
      fake_child->cost           = (rough_path[ptr + 1]->p - new_path.back()->p).norm();
      fake_child->dontConnect    = false;
      fake_child->connects2wp    = false;
      new_path.back()->children.push_back(fake_child);
      normal_gchild->p           = rough_path[ptr + 2]->p;
      normal_gchild->fil         = fil2;
      normal_gchild->parent      = fake_child;
      normal_gchild->cost        = normal_gchild->parent->cost + (rough_path[ptr + 2]->p - rough_path[ptr + 1]->p).norm() - fil2.adj;
      normal_gchild->dontConnect = false;
      normal_gchild->connects2wp = false;
      fake_child->children.push_back(normal_gchild);

      ptr = 2;
      new_path.push_back(fake_child);
      new_path.push_back(normal_gchild);
    }
    else
      ptr = 0; // TODO there will be a problem if the fan is created for the first node...
    while (ptr < rough_path.size() - 1)
    {
      ROS_DEBUG("ptr = %i of %lu",ptr, rough_path.size() - 1);
      if (new_path.back()->parent != NULL)
        temp_path.push_back(new_path.back()->fil.z2);
      temp_path.push_back(new_path.back()->p);
      temp_path.push_back(rough_path[ptr + 1]->p);
      // displayPath(temp_path,false);
      temp_path.clear();

      ROS_DEBUG("ptr = %i of %lu",ptr, rough_path.size() - 1);
      if (checkForCollision(new_path.back(), rough_path[ptr + 1]->p, i, path_clearance_, false) == false) // if there is a collision
      {
        ROS_DEBUG("Collision, adding the most recent node");
        ROS_DEBUG("N: %f, E: %f, D: %f", new_path.back()->p.N, new_path.back()->p.E,new_path.back()->p.D);
        ROS_DEBUG("N: %f, E: %f, D: %f", most_recent_node_->p.N, most_recent_node_->p.E,most_recent_node_->p.D);
        if (new_path.back()->parent != NULL)
          temp_path.push_back(new_path.back()->fil.z2);
        temp_path.push_back(new_path.back()->p);
        temp_path.push_back(most_recent_node_->p);
        // displayPath(temp_path,true);
        temp_path.clear();
        new_path.push_back(most_recent_node_);
        ROS_DEBUG("Adding most recent node");
      }
      else
      {
        ptr++;
      }
    }
    new_path.push_back(smooth_rts_[i + 1]);
    ROS_DEBUG("N: %f, E: %f, D: %f", new_path.back()->p.N, new_path.back()->p.E,new_path.back()->p.D);
    for (int it = 0; it < new_path.size(); it++)
    {
      // temp_neds.push_back(new_path[it]->p);
      ROS_DEBUG("SMOOTH PATH N: %f E: %f D: %f", new_path[it]->p.N, new_path[it]->p.E, new_path[it]->p.D);
    }
    // displayPath(temp_neds, true);
    temp_neds.clear();
    // TODO see if it is possible to smooth the fan
  }
  else
  {
    int ptr = 0;
    while (ptr < rough_path.size() - 1)
    {
      ROS_DEBUG("ptr = %i of %lu",ptr, rough_path.size() - 1);
      if (new_path.back()->parent != NULL)
        temp_path.push_back(new_path.back()->fil.z2);
      temp_path.push_back(new_path.back()->p);
      temp_path.push_back(rough_path[ptr + 1]->p);
      // displayPath(temp_path,false);
      temp_path.clear();
      if (checkForCollision(new_path.back(), rough_path[ptr + 1]->p, i, path_clearance_, false) == false) // if there is a collision
      {
        ROS_DEBUG("Collision, adding the most recent node");
        ROS_DEBUG("N: %f, E: %f, D: %f", new_path.back()->p.N, new_path.back()->p.E,new_path.back()->p.D);
        ROS_DEBUG("N: %f, E: %f, D: %f", most_recent_node_->p.N, most_recent_node_->p.E,most_recent_node_->p.D);
        if (new_path.back()->parent != NULL)
          temp_path.push_back(new_path.back()->fil.z2);
        temp_path.push_back(new_path.back()->p);
        temp_path.push_back(most_recent_node_->p);
        // displayPath(temp_path,true);
        temp_path.clear();

        new_path.push_back(most_recent_node_);
      }
      else
      {
        ptr++;
        ROS_DEBUG("no collision");
      }
    }
    new_path.push_back(smooth_rts_[i + 1]);
    for (int it = 0; it < new_path.size(); it++)
    {
      temp_neds.push_back(new_path[it]->p);
      ROS_DEBUG("SMOOTH PATH N: %f E: %f D: %f", new_path[it]->p.N, new_path[it]->p.E, new_path[it]->p.D);
    }
    // displayPath(temp_neds, true);
  }
  new_path.erase(new_path.begin());
  return new_path;
}
void RRT::addPath(std::vector<node*> smooth_path, unsigned int i)
{
  ROS_DEBUG("Adding the path");
  ROS_DEBUG("smooth_path.size() %lu",smooth_path.size());
  for (unsigned int j = 0; j < smooth_path.size(); j++)
  {
    if (direct_hit_ == true && j == smooth_path.size() - 1 && i != map_.wps.size() - 1 && j != 0)
    {

    }
    // else if (direct_hit_ == true && i == 2 && j == 0)
    // {
    //
    // }
    else
      all_wps_.push_back(smooth_path[j]->p);
  }
}


// Secondary functions
node* RRT::findClosestNodeGChild(node* root, NED_s p)
{
  float distance = INFINITY;
  node* closest_gchild;
  node* closest_node;
  for (unsigned int j = 0; j < root->children.size(); j++)
    for (unsigned int k = 0; k < root->children[j]->children.size(); k++)
    {
      float d_gchild = (p - root->children[j]->children[k]->p).norm();
      closest_gchild = findClosestNode(root->children[j]->children[k], p, root->children[j]->children[k], &d_gchild);
      if (d_gchild < distance)
      {
        closest_node = closest_gchild;
        distance = d_gchild;
      }
    }
    return closest_node;
}
bool RRT::checkForCollision(node* ps, NED_s pe, unsigned int i, float clearance, bool connecting_to_end)
{
  // returns true if there was no collision detected.
  // returns false if there was a collision collected.
  node* start_of_line;
  if (ps->dontConnect) // then try one of the grand children
  {
    // //ROS_DEBUG("finding one of the grand children");
    start_of_line = findClosestNodeGChild(ps, pe);
  }
  else
  {
    // //ROS_DEBUG("using the starting point");
    start_of_line = ps;
  }
  // //ROS_DEBUG("checking the line");
  if (col_det_.checkLine(start_of_line->p, pe, clearance))
  {
    // ROS_FATAL("chekcLine in RRT passed");
    // //ROS_DEBUG("line worked");
    if (start_of_line->parent == NULL) // then this is the start
    {
      // //ROS_DEBUG("parent was null");
      float chi = (pe - start_of_line->p).getChi();
      // //ROS_DEBUG("checking after waypoint");
      // //ROS_DEBUG("found a good connection");
      node* ending_node        = new node;
      ending_node->p           = pe;
      // don't do the fillet
      ending_node->parent      = start_of_line;
      ending_node->cost        = start_of_line->cost + (pe - start_of_line->p).norm();
      ending_node->dontConnect = false;
      ending_node->connects2wp = (pe == map_.wps[i]);
      start_of_line->children.push_back(ending_node);
      most_recent_node_        = ending_node;
      // //ROS_DEBUG("printing ending node");
      // printNode(ending_node);
      return true;
    }
    else
    {
      // //ROS_DEBUG("parent not null, caclulating fillet");
      fillet_s fil;
      bool fil_possible = fil.calculate(start_of_line->parent->p, start_of_line->p, pe, input_file_.turn_radius);
      // ROS_WARN("n_beg: %f, e_beg: %f, d_beg: %f, n_end: %f, e_end: %f, d_end: %f, ",\
      // fil.w_im1.N, fil.w_im1.E, fil.w_im1.D, fil.z1.N, fil.z1.E, fil.z1.D);
      // printFillet(fil);

      // if (fil_possible) { ROS_INFO("fillet possible");}
      // else {ROS_WARN("fillet not possible");}
      fillet_s temp_fil = fil;
      float slope = atan2f(-1.0f*(fil.z1.D - start_of_line->fil.z2.D), sqrtf(powf(start_of_line->fil.z2.N - \
                           fil.z1.N, 2.0f) + powf(start_of_line->fil.z2.E - fil.z1.E, 2.0f)));
      if (slope < -1.0f*input_file_.max_descend_angle || slope > input_file_.max_climb_angle)
        return false;
      slope = atan2f(-1.0f*(pe.D - fil.z2.D), sqrtf(powf(fil.z2.N - pe.N, 2.0f) + powf(fil.z2.E - pe.E, 2.0f)));
      if (slope < -1.0f*input_file_.max_descend_angle || slope > input_file_.max_climb_angle)
        return false;
      temp_fil.w_im1 = fil.z1;
      if (fil_possible && col_det_.checkFillet(temp_fil, clearance))
      {
        //ROS_DEBUG("passed fillet check, checking for neighboring fillets");
        if (start_of_line->parent->parent != NULL && start_of_line->fil.roomFor(fil) == false)
        {
          // printNode(start_of_line);
          //ROS_DEBUG("testing spot N: %f, E %f, D %f", pe.N, pe.E, pe.D);
          // ROS_FATAL("failed neighboring fillets");
          return false;
        }
        //ROS_DEBUG("passed neighboring fillets");
        float chi = (pe - start_of_line->p).getChi();
        // //ROS_DEBUG("checking after wp");
        //ROS_DEBUG("everything worked, adding another connection");
        node* ending_node        = new node;
        ending_node->p           = pe;
        ending_node->fil         = fil;
        ending_node->parent      = start_of_line;
        ending_node->cost        = start_of_line->cost + (pe - start_of_line->p).norm() - fil.adj;
        ending_node->dontConnect = false;
        ending_node->connects2wp = (pe == map_.wps[i]);
        start_of_line->children.push_back(ending_node);
        most_recent_node_        = ending_node;
        // //ROS_DEBUG("printing ending node");
        // printNode(ending_node);
        return true;
      }
      // else
      //   ROS_ERROR("failed fillet");
    }
  }
  else
    //ROS_DEBUG("failed line check");
  // //ROS_DEBUG("Adding node Failed");
  return false;
}
NED_s RRT::randomPoint(unsigned int i)
{
  NED_s P;
  P.N = rg_.randLin()*(col_det_.maxNorth_ - col_det_.minNorth_) + col_det_.minNorth_;
	P.E = rg_.randLin()*(col_det_.maxEast_  - col_det_.minEast_)  + col_det_.minEast_;
  if (taking_off_)
    P.D = root_ptrs_[i]->p.D - sqrtf(P.N*P.N + P.E*P.E)*0.6f*input_file_.max_climb_angle;
  else if (landing_now_)
    P.D = root_ptrs_[i]->p.D + sqrtf(P.N*P.N + P.E*P.E)*0.6f*input_file_.max_descend_angle;
  else
    P.D = root_ptrs_[i]->p.D;
  return P;
}
float RRT::redoRandomDownPoint(unsigned int i, float closest_D)
{
  if (-root_ptrs_[i + 1]->p.D > -closest_D)
    return -(segment_length_*sinf(input_file_.max_climb_angle)*(rg_.randLin()*1.5f - 0.5f)*0.5f - closest_D);
  if (-root_ptrs_[i + 1]->p.D < -closest_D)
    return -(segment_length_*sinf(input_file_.max_descend_angle)*(rg_.randLin()*-1.5f + 0.5f)*0.5f - closest_D);
  else
    return -(segment_length_*sinf(input_file_.max_climb_angle)*(rg_.randLin()*1.5f - 0.75f)*0.5f - closest_D);
}
node* RRT::findClosestNode(node* nin, NED_s P, node* minNode, float* minD) // This recursive function return the closes node to the input point P, for some reason it wouldn't go in the cpp...
{// nin is the node to measure, P is the point, minNode is the closes found node so far, minD is where to store the minimum distance
  // Recursion
  float distance;                                         // distance to the point P
  for (unsigned int i = 0; i < nin->children.size(); i++) // For all of the children figure out their distances
  {
    distance = (P - nin->children[i]->p).norm();
    if (distance < *minD)          // If we found a better distance, update it
    {
      minNode = nin->children[i];  // reset the minNode
      *minD = distance;            // reset the minimum distance
    }
    minNode = findClosestNode(nin->children[i], P, minNode, minD); // Recursion for each child
  }
  return minNode;                  // Return the closest node
}
node* RRT::findMinConnector(node* nin, node* minNode, float* minCost) // This recursive function return the closes node to the input point P, for some reason it wouldn't go in the cpp...
{// nin is the node to measure, minNode is the closes final node in the path so far, minD is where to store the minimum distance
  // Recursion
  float cost;                     // total cost of the function
  if (nin->connects2wp == true)
  {
    //ROS_DEBUG("found a connector");
    cost = nin->cost;
    if (cost <= *minCost)          // If we found a better cost, update it
    {
      //ROS_DEBUG("found lower costing connector");
      minNode = nin;              // reset the minNode
      *minCost = cost;               // reset the minimum cost
    }
  }
  else
  {
    // //ROS_DEBUG("checking all children for connectors");
    for (unsigned int i = 0; i < nin->children.size(); i++) // For all of the children figure out their distances
      minNode = findMinConnector(nin->children[i], minNode, minCost); // Recursion for each child
    // //ROS_DEBUG("finished checking all children for connect  ors");
  }
  return minNode;                  // Return the closest node
}
void RRT::createFan(node* root, NED_s p, float chi, float clearance)
{
  ROS_DEBUG("Creating Fan");
  bool found_at_least_1_good_path = false;
  // Make sure that it is possible to go to the next waypoint

  float alpha  = M_PI / 4.0;			// Start with 45.0 degrees // TODO this might cause errors depending on the turning radius
  float R      = 3.0*input_file_.turn_radius;
  int num_circle_trials = 10;				// Will try num_circle_trials on one side and num_circle_trials on the other side.
  float dalpha = (M_PI - alpha) / num_circle_trials;

  float approach_angle = -(chi + 1.0f*M_PI)  + M_PI/2.0f; //atan2(p.N - coming_from.N, p.E - coming_from.E) + M_PI;
  float beta, lambda, Q, phi, theta, zeta, gamma, d;
  NED_s cpa, cea, lea, fake_wp;
  for (int j = 0; j < num_circle_trials; j++)
  {
    alpha  = alpha + dalpha;
    beta   = M_PI / 2 - alpha;
    lambda = M_PI - 2 * beta;
    Q      = sqrtf(R*(R - input_file_.turn_radius*sinf(lambda) / sinf(beta)) + input_file_.turn_radius*input_file_.turn_radius);
    phi    = M_PI - asinf(R*sinf(beta) / Q);
    theta  = acosf(input_file_.turn_radius / Q);
    zeta   = (2 * M_PI - phi - theta) / 2.0;
    gamma  = M_PI - 2 * zeta;
    d      = input_file_.turn_radius / tanf(gamma / 2.0) + 1.0f;

    // Check the positive side
    fake_wp.N = p.N - d*sinf(approach_angle);
    fake_wp.E = p.E - d*cosf(approach_angle);
    fake_wp.D = p.D;

    cpa.N = p.N + input_file_.turn_radius*cosf(approach_angle);
    cpa.E = p.E - input_file_.turn_radius*sinf(approach_angle);
    cpa.D = p.D;

    cea.N = fake_wp.N + d*sinf(gamma + approach_angle);
    cea.E = fake_wp.E + d*cosf(gamma + approach_angle);
    cea.D = p.D;

    lea.N = p.N + R*sinf(approach_angle + alpha);
    lea.E = p.E + R*cosf(approach_angle + alpha);
    lea.D = p.D;

    if (col_det_.checkArc(p, cea, input_file_.turn_radius, cpa, -1, clearance))
      if (col_det_.checkLine(cea, lea, clearance))
      {
        fillet_s fil1, fil2;
        bool passed1, passed2;
        if (root->parent == NULL)
        {
          NED_s fake_parent;
          fake_parent.N = root->p.N + 100.0f*cosf(chi + M_PI);
          fake_parent.E = root->p.E + 100.0f*sinf(chi + M_PI);
          fake_parent.D = root->p.D;
          passed1 = fil1.calculate(fake_parent, p, fake_wp, input_file_.turn_radius);
        }
        else
          passed1 = fil1.calculate(root->parent->p, p, fake_wp, input_file_.turn_radius);
        passed2 = fil2.calculate(p, fake_wp, lea, input_file_.turn_radius);
        node *fake_child        = new node;
        node *normal_gchild     = new node;
        fake_child->p           = fake_wp;
        fake_child->fil         = fil1;
        fake_child->parent      = root;
        fake_child->cost        = (fake_wp - p).norm();
        fake_child->dontConnect = false;
        fake_child->connects2wp = false;
        root->children.push_back(fake_child);
        normal_gchild->p           = lea;
        normal_gchild->fil         = fil2;
        normal_gchild->parent      = fake_child;
        normal_gchild->cost        = normal_gchild->parent->cost + (lea - fake_wp).norm() - fil2.adj;
        normal_gchild->dontConnect = false;
        normal_gchild->connects2wp = false;
        fake_child->children.push_back(normal_gchild);

        // std::vector<NED_s> temp_path;
        // if (normal_gchild->parent->parent != NULL)
        //   temp_path.push_back(normal_gchild->parent->parent->p);
        // temp_path.push_back(normal_gchild->parent->p);
        // temp_path.push_back(normal_gchild->p);
        // displayPath(temp_path,false);
      }
    // Check the negative side
    cpa.N = p.N - input_file_.turn_radius*cosf(approach_angle);
    cpa.E = p.E + input_file_.turn_radius*sinf(approach_angle);
    cpa.D = p.D;

    cea.N = fake_wp.N + d*sinf(-gamma + approach_angle);
    cea.E = fake_wp.E + d*cosf(-gamma + approach_angle);
    cea.D = p.D;

    lea.N = p.N + R*sinf(approach_angle - alpha);
    lea.E = p.E + R*cosf(approach_angle - alpha);
    lea.D = p.D;

    if (col_det_.checkArc(p, cea, input_file_.turn_radius, cpa, 1, clearance))
      if (col_det_.checkLine(cea, lea, clearance))
      {
        fillet_s fil1, fil2;
        bool passed1, passed2;
        if (root->parent == NULL)
        {
          NED_s fake_parent;
          fake_parent.N = root->p.N + 100.0f*cosf(chi + M_PI);
          fake_parent.E = root->p.E + 100.0f*sinf(chi + M_PI);
          fake_parent.D = root->p.D;
          passed1 = fil1.calculate(fake_parent, p, fake_wp, input_file_.turn_radius);
        }
        else
          passed1 = fil1.calculate(root->parent->p, p, fake_wp, input_file_.turn_radius);
        passed2 = fil2.calculate(p, fake_wp, lea, input_file_.turn_radius);
        node *fake_child        = new node;
        node *normal_gchild     = new node;
        fake_child->p           = fake_wp;
        fake_child->fil         = fil1;
        fake_child->parent      = root;
        fake_child->cost        = (fake_wp - p).norm();
        fake_child->dontConnect = false;
        fake_child->connects2wp = false;
        root->children.push_back(fake_child);
        normal_gchild->p           = lea;
        normal_gchild->fil         = fil2;
        normal_gchild->parent      = fake_child;
        normal_gchild->cost        = normal_gchild->parent->cost + (lea - fake_wp).norm() - fil2.adj;
        normal_gchild->dontConnect = false;
        normal_gchild->connects2wp = false;
        fake_child->children.push_back(normal_gchild);

        // std::vector<NED_s> temp_path;
        // if (normal_gchild->parent->parent != NULL)
        //   temp_path.push_back(normal_gchild->parent->parent->p);
        // temp_path.push_back(normal_gchild->parent->p);
        // temp_path.push_back(normal_gchild->p);
        // displayPath(temp_path,false);
      }
  }
  ROS_DEBUG("Created the fan");
}
// Initializing and Clearing Data
void RRT::initializeTree(NED_s pos, float chi0)
{
  bool fan_first_node = direct_hit_;
  if (-pos.D < input_file_.minFlyHeight)
    fan_first_node = false;
	// Set up all of the roots
	node *root_in0        = new node;        // Starting position of the tree (and the waypoint beginning)
  node *root_in0_smooth = new node;
  fillet_s emp_f;
	root_in0->p           = pos;
  root_in0->fil         = emp_f;
  root_in0->fil.z2      = root_in0->p;
	root_in0->parent      = NULL;            // No parent
	root_in0->cost        = 0.0f;            // 0 distance.
  root_in0->dontConnect = fan_first_node;
  root_in0->connects2wp = false;
  ROS_DEBUG("about to set smoother");
  root_in0_smooth->equal(root_in0);
  ROS_DEBUG("set smooth_rts");
	root_ptrs_.push_back(root_in0);
  smooth_rts_.push_back(root_in0_smooth);
  int num_root = 0;
  num_root++;
  for (unsigned int i = 0; i < map_.wps.size(); i++)
	{
		node *root_in        = new node;       // Starting position of the tree (and the waypoint beginning)
    node *root_in_smooth = new node;
    root_in->p           = map_.wps[i];
    root_in->fil         = emp_f;
    root_in0->fil.z2     = root_in0->p;
  	root_in->parent      = NULL;           // No parent
  	root_in->cost        = 0.0f;           // 0 distance.
    root_in->dontConnect = direct_hit_;
    root_in->connects2wp = false;
    root_in_smooth->equal(root_in);
    root_ptrs_.push_back(root_in);
    smooth_rts_.push_back(root_in_smooth);
    num_root++;
	}
  printRoots();
}
void RRT::clearForNewPath()
{
  all_wps_.clear();
  clearTree();                    // Clear all of those tree pointer nodes
  // std::vector<node*>().swap(root_ptrs_);
}
void RRT::newMap(map_s map_in)
{
  map_ = map_in;
  col_det_.newMap(map_in);
}
void RRT::newSeed(unsigned int seed)
{
  RandGen rg_in(seed);          // Make a random generator object that is seeded
	rg_         = rg_in;           // Copy that random generator into the class.
}
void RRT::deleteTree()
{
  if (root_ptrs_.size() > 0)
    deleteNode(root_ptrs_[0]);
  root_ptrs_.clear();
  if (smooth_rts_.size() > 0)
    deleteNode(smooth_rts_[0]);
  smooth_rts_.clear();
}
void RRT::deleteNode(node* pn)                         // Recursively delete every node
{
	for (unsigned int i = 0; i < pn->children.size();i++)
		deleteNode(pn->children[i]);
	pn->children.clear();
	delete pn;
}
void RRT::clearTree()
{
  //ROS_DEBUG("clearing tree");
  if (root_ptrs_.size() > 0)
    clearNode(root_ptrs_[0]);
  root_ptrs_.clear();
  if (smooth_rts_.size() > 0)
    clearNode(smooth_rts_[0]);
  smooth_rts_.clear();
}
void RRT::clearNode(node* pn)                         // Recursively delete every node
{
	for (unsigned int i = 0; i < pn->children.size();i++)
		clearNode(pn->children[i]);
	pn->children.clear();
  delete pn;
}

// Printing Functions
void RRT::printRRTSetup(NED_s pos, float chi0)
{
  // Print initial position
  ROS_DEBUG("Initial North: %f, Initial East: %f, Initial Down: %f", pos.N, pos.E, pos.D);

  ROS_DEBUG("Number of Boundary Points: %lu",  map_.boundary_pts.size());
  for (long unsigned int i = 0; i < map_.boundary_pts.size(); i++)
  {
    ROS_DEBUG("Boundary: %lu, North: %f, East: %f, Down: %f", \
    i, map_.boundary_pts[i].N, map_.boundary_pts[i].E, map_.boundary_pts[i].D);
  }
  ROS_DEBUG("Number of Waypoints: %lu", map_.wps.size());
  for (long unsigned int i = 0; i < map_.wps.size(); i++)
  {
    ROS_DEBUG("WP: %lu, North: %f, East: %f, Down: %f", i + (unsigned long int) 1, map_.wps[i].N, map_.wps[i].E, map_.wps[i].D);
  }
  ROS_DEBUG("Number of Cylinders: %lu", map_.cylinders.size());
  for (long unsigned int i = 0; i <  map_.cylinders.size(); i++)
  {
    ROS_DEBUG("Cylinder: %lu, North: %f, East: %f, Radius: %f, Height: %f", \
    i, map_.cylinders[i].N, map_.cylinders[i].E, map_.cylinders[i].R,  map_.cylinders[i].H);
  }
}
void RRT::printRoots()
{
  for (unsigned int i = 0; i < root_ptrs_.size(); i++)
    ROS_DEBUG("Waypoint %i, North: %f, East %f Down: %f", \
    i, root_ptrs_[i]->p.N, root_ptrs_[i]->p.E, root_ptrs_[i]->p.D);
}
void RRT::printNode(node* nin)
{
  ROS_DEBUG("NODE ADDRESS: %p", (void *)nin);
  ROS_DEBUG("p.N %f, p.E %f, p.D %f", nin->p.N, nin->p.E, nin->p.D);
  printFillet(nin->fil);
  ROS_DEBUG("fil.w_im1.N %f, fil.w_im1.E %f, fil.w_im1.D %f", nin->fil.w_im1.N, nin->fil.w_im1.E, nin->fil.w_im1.D);
  ROS_DEBUG("fil.w_i.N %f, fil.w_i.E %f, fil.w_i.D %f", nin->fil.w_i.N, nin->fil.w_i.E, nin->fil.w_i.D);
  ROS_DEBUG("fil.w_ip1.N %f, fil.w_ip1.E %f, fil.w_ip1.D %f", nin->fil.w_ip1.N, nin->fil.w_ip1.E, nin->fil.w_ip1.D);
  ROS_DEBUG("fil.z1.N %f, fil.z1.E %f, fil.z1.D %f", nin->fil.z1.N, nin->fil.z1.E, nin->fil.z1.D);
  ROS_DEBUG("fil.z2.N %f, fil.z2.E %f, fil.z2.D %f", nin->fil.z2.N, nin->fil.z2.E, nin->fil.z2.D);
  ROS_DEBUG("fil.c.N %f, fil.c.E %f, fil.c.D %f", nin->fil.c.N, nin->fil.c.E, nin->fil.c.D);
  ROS_DEBUG("fil.q_im1.N %f, fil.q_im1.E %f, fil.q_im1.D %f", nin->fil.q_im1.N, nin->fil.q_im1.E, nin->fil.q_im1.D);
  ROS_DEBUG("fil.q_i.N %f, fil.q_i.E %f, fil.q_i.D %f", nin->fil.q_i.N, nin->fil.q_i.E, nin->fil.q_i.D);
  ROS_DEBUG("fil.R %f", nin->fil.R);
  ROS_DEBUG("parent %p", (void *)nin->parent);
  ROS_DEBUG("number of children %lu", nin->children.size());
  ROS_DEBUG("cost %f", nin->cost);
  if (nin->dontConnect) {ROS_DEBUG("dontConnect == true");}
  else {ROS_DEBUG("dontConnect == false");}
  if (nin->connects2wp) {ROS_DEBUG("connects2wp == true");}
  else {ROS_DEBUG("connects2wp == false");}
}
void RRT::printFillet(fillet_s fil)
{
  ROS_DEBUG("fil.w_im1.N %f, fil.w_im1.E %f, fil.w_im1.D %f",  fil.w_im1.N,  fil.w_im1.E,  fil.w_im1.D);
  ROS_DEBUG("fil.w_i.N %f, fil.w_i.E %f, fil.w_i.D %f",  fil.w_i.N,  fil.w_i.E,  fil.w_i.D);
  ROS_DEBUG("fil.w_ip1.N %f, fil.w_ip1.E %f, fil.w_ip1.D %f",  fil.w_ip1.N,  fil.w_ip1.E,  fil.w_ip1.D);
  ROS_DEBUG("fil.z1.N %f, fil.z1.E %f, fil.z1.D %f",  fil.z1.N,  fil.z1.E,  fil.z1.D);
  ROS_DEBUG("fil.z2.N %f, fil.z2.E %f, fil.z2.D %f",  fil.z2.N,  fil.z2.E,  fil.z2.D);
  ROS_DEBUG("fil.c.N %f, fil.c.E %f, fil.c.D %f",  fil.c.N,  fil.c.E,  fil.c.D);
  ROS_DEBUG("fil.q_im1.N %f, fil.q_im1.E %f, fil.q_im1.D %f",  fil.q_im1.N,  fil.q_im1.E,  fil.q_im1.D);
  ROS_DEBUG("fil.q_i.N %f, fil.q_i.E %f, fil.q_i.D %f",  fil.q_i.N,  fil.q_i.E,  fil.q_i.D);
  ROS_DEBUG("fil.R %f",  fil.R);
}
void RRT::clearRVizPaths()
{
  visualization_msgs::Marker clear_mkr;
  ROS_DEBUG("clearing all paths");
  clear_mkr.ns   = "planned_path";
  ROS_DEBUG("1 to %i", last_path_id_);
  for (int i = 1; i < last_path_id_ + 1; i++)
  {
    clear_mkr.id = i;
    clear_mkr.action = visualization_msgs::Marker::DELETE;
    marker_pub_.publish(clear_mkr);
  }
}
void RRT::displayPath(std::vector<NED_s> path, bool testing)
{
  visualization_msgs::Marker planned_path_mkr, aWPS_mkr;
  if (path_id_ == 1)
    clearRVizPaths();
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
  if (testing)
  {
    planned_path_mkr.color.r    = 0.0f;
    planned_path_mkr.color.g    = 1.0f;
    planned_path_mkr.color.b    = 0.0f;
    planned_path_mkr.color.a    = 1.0;
    aWPS_mkr.scale.x      =  10.0; // point width
    aWPS_mkr.scale.y      =  10.0; // point height
    planned_path_mkr.scale.x      = 8.0; // line width
  }
  else
  {
    planned_path_mkr.color.r    = 0.0f;
    planned_path_mkr.color.g    = 0.0f;
    planned_path_mkr.color.b    = 1.0f;
    planned_path_mkr.color.a    = 1.0;
    aWPS_mkr.scale.x      =  10.0; // point width
    aWPS_mkr.scale.y      =  10.0; // point height
    planned_path_mkr.scale.x      = 5.0; // line width
  }
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
  if (testing)
    planned_path_mkr.id         = 0;
  else
    planned_path_mkr.id         = path_id_++;
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
  sleep(4.0);
}
std::vector<std::vector<float > > RRT::arc(float N, float E, float r, float aS, float aE)
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
} // end namespace theseus
