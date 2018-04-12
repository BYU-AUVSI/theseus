#include <theseus/RRT.h>

namespace theseus
{
RRT::RRT(map_s map_in, unsigned int seed) // Setup the object
{
  segment_length_ = 100.0;        // pull in segment_length_;
  num_paths_      = 1;            // number of paths to solve between each waypoint
	RandGen rg_in(seed);            // Make a random generator object that is seeded
	rg_             = rg_in;        // Copy that random generator into the class.
  map_            = map_in;
  col_det_.newMap(map_in);
  col_det_.taking_off_ = true;    // TODO implement this correctly
}
RRT::RRT()
{

}
RRT::~RRT()
{
	deleteTree();                          // Delete all of those tree pointer nodes
	std::vector<node*>().swap(root_ptrs_); // Free the memory of the vector.
}
void RRT::solveStatic(NED_s pos, float chi0, bool direct_hit)         // This function solves for a path in between the waypoinnts (2 Dimensional)
{
  direct_hit_ = direct_hit;
  ROS_INFO("Starting RRT solver");
  clearForNewPath();
	initializeTree(pos, chi0);
  taking_off_ = (-pos.D < input_file_.minFlyHeight);
  printRRTSetup(pos, chi0);
  for (unsigned int i = 0; i < map_.wps.size(); i++)
  {
    ROS_INFO("Finding route to waypoint %lu", i + (long unsigned int) 1);
    path_clearance_ = input_file_.clearance;
    bool direct_connection = tryDirectConnect(root_ptrs_[i], root_ptrs_[i + 1], i);
    if (direct_connection == false)
    {
      int num_found_paths = 0;
      while (num_found_paths < num_paths_)
      {
        num_found_paths += developTree(i);
      }
    }
    std::vector<node*> rough_path  = findMinimumPath(i);
    std::vector<node*> smooth_path = smoothPath(rough_path);
    addPath(smooth_path);
  }
  // find a place to safely loiter?
}


bool RRT::tryDirectConnect(node* ps, node* pe_node, unsigned int i)
{
  ROS_INFO("Attempting direct connect");
  float clearance = path_clearance_;
  node* start_of_line;
  if (ps->dontConnect) // then try one of the grand children
    start_of_line = findClosestNodeGChild(ps, pe_node->p);
  else
    start_of_line = ps;
  if (col_det_.checkLine(start_of_line->p, pe_node->p, clearance))
  {
    if (start_of_line->parent == NULL) // then this is the start
    {
      float chi = (pe_node->p - start_of_line->p).getChi();
      if (col_det_.checkAfterWP(pe_node->p, chi, clearance))
      {
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
      bool fil_possible = fil.calculate(start_of_line->parent->p, start_of_line->p, pe_node->p, input_file_.turn_radius);
      if (fil_possible && col_det_.checkFillet(fil, clearance))
      {
        if (start_of_line->parent != NULL && start_of_line->fil.roomFor(fil) == false)
          return false;
        float chi = (pe_node->p - start_of_line->p).getChi();
        if (col_det_.checkAfterWP(pe_node->p, chi, clearance))
        {
          start_of_line->cost        = start_of_line->cost + (pe_node->p - start_of_line->p).norm() - fil.adj;
          start_of_line->connects2wp = true;
          start_of_line->children.push_back(pe_node);
          most_recent_node_          = pe_node;
          return true;
        }
      }
    }
  }
  return false;
}
int RRT::developTree(unsigned int i)
{
  bool added_new_node = false;
  float clearance = path_clearance_;
  while (added_new_node == false)
  {
    // generate a good point to test
    NED_s random_point = randomPoint(i);
    float min_d        = INFINITY;
    node* closest_node = findClosestNode(root_ptrs_[i], random_point, root_ptrs_[i], &min_d);
    NED_s test_point   = (random_point - closest_node->p).normalize()*segment_length_ + closest_node->p;
    added_new_node     = checkForCollision(closest_node, test_point, i, clearance);
  }
  bool connect_to_end = tryDirectConnect(most_recent_node_, root_ptrs_[i + 1], i);
  if (connect_to_end == true)
    return 1;
  else
    return 0;
}
std::vector<node*> RRT::findMinimumPath(unsigned int i)
{
  // recursively go through the tree to find the connector
  std::vector<node*> rough_path;
  float minimum_cost = INFINITY;
  node* almost_last  = findMinConnector(root_ptrs_[i], root_ptrs_[i], &minimum_cost);
  fillet_s fil;
  bool fil_b = fil.calculate(almost_last->parent->p, almost_last->p, root_ptrs_[i + 1]->p, input_file_.turn_radius);
  root_ptrs_[i + 1]->fil    = fil;
  root_ptrs_[i + 1]->parent = almost_last;

	std::stack<node*> wpstack;
	node *current_node = root_ptrs_[i + 1];
	while (current_node != root_ptrs_[i])
	{
		wpstack.push(current_node);
		current_node = current_node->parent;
	}
	wpstack.push(root_ptrs_[i]);
	std::vector<NED_s> wps_to_PrimaryWP;
	while (!wpstack.empty())
	{
		rough_path.push_back(wpstack.top());
		wpstack.pop();
	}
  return rough_path;
}
std::vector<node*> RRT::smoothPath(std::vector<node*> rough_path)
{
  return rough_path;
}
void RRT::addPath(std::vector<node*> smooth_path)
{
  std::vector<NED_s> wps_to_PrimaryWP;
  for (unsigned int i = 0; i < smooth_path.size(); i++)
    wps_to_PrimaryWP.push_back(smooth_path[i]->p);
  all_wps_.push_back(wps_to_PrimaryWP);
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
bool RRT::checkForCollision(node* ps, NED_s pe, unsigned int i, float clearance)
{
  node* start_of_line;
  if (ps->dontConnect) // then try one of the grand children
    start_of_line = findClosestNodeGChild(ps, pe);
  else
    start_of_line = ps;
  if (col_det_.checkLine(start_of_line->p, pe, clearance))
  {
    if (start_of_line->parent == NULL) // then this is the start
    {
      float chi = (pe - start_of_line->p).getChi();
      if (col_det_.checkAfterWP(pe, chi, clearance))
      {
        node* ending_node        = new node;
        ending_node->p           = pe;
        // don't do the fillet
        ending_node->parent      = start_of_line;
        ending_node->cost        = start_of_line->cost + (pe - start_of_line->p).norm();
        ending_node->dontConnect = false;
        ending_node->connects2wp = (pe == map_.wps[i]);
        start_of_line->children.push_back(ending_node);
        most_recent_node_        = ending_node;
        return true;
      }
    }
    else
    {
      fillet_s fil;
      bool fil_possible = fil.calculate(start_of_line->parent->p, start_of_line->p, pe, input_file_.turn_radius);
      if (fil_possible && col_det_.checkFillet(fil, clearance))
      {
        if (start_of_line->parent != NULL && start_of_line->fil.roomFor(fil) == false)
          return false;
        float chi = (pe - start_of_line->p).getChi();
        if (col_det_.checkAfterWP(pe, chi, clearance))
        {
          node* ending_node        = new node;
          ending_node->p           = pe;
          ending_node->fil         = fil;
          ending_node->parent      = start_of_line;
          ending_node->cost        = start_of_line->cost + (pe - start_of_line->p).norm() - fil.adj;
          ending_node->dontConnect = false;
          ending_node->connects2wp = (pe == map_.wps[i]);
          start_of_line->children.push_back(ending_node);
          most_recent_node_        = ending_node;
          return true;
        }
      }
    }
  }
  return false;
}
NED_s RRT::randomPoint(unsigned int i)
{
  NED_s P;
  P.N = rg_.randLin()*(col_det_.maxNorth_ - col_det_.minNorth_) + col_det_.minNorth_;
	P.E = rg_.randLin()*(col_det_.maxEast_  - col_det_.minEast_)  + col_det_.minEast_;
  P.D = map_.wps[i].D;
  return P;
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
    cost = nin->cost;
    if (cost < *minCost)          // If we found a better cost, update it
    {
      minNode = nin;              // reset the minNode
      *minCost = cost;               // reset the minimum cost
    }
  }
  else
    for (unsigned int i = 0; i < nin->children.size(); i++) // For all of the children figure out their distances
      minNode = findMinConnector(nin->children[i], minNode, minCost); // Recursion for each child
  return minNode;                  // Return the closest node
}
// Initializing and Clearing Data
void RRT::initializeTree(NED_s pos, float chi0)
{
  bool fan_first_node = false; // TODO change this so there can be a fan for the initial point
  if (-pos.D < input_file_.minFlyHeight)
    fan_first_node = false;
	// Set up all of the roots
	node *root_in0 = new node;              // Starting position of the tree (and the waypoint beginning)
  fillet_s emp_f;
	root_in0->p           = pos;
  root_in0->fil         = emp_f;
	root_in0->parent      = NULL;           // No parent
	root_in0->cost        = 0.0;            // 0 distance.
  root_in0->dontConnect = fan_first_node;
  root_in0->connects2wp = false;
  // if (fan_first_node) // TODO
  //   // create initial fan
	root_ptrs_.push_back(root_in0);
  // TODO create fan for the initial point
  int num_root = 0;
  num_root++;
  for (unsigned int i = 0; i < map_.wps.size(); i++)
	{
		node *root_in        = new node;           // Starting position of the tree (and the waypoint beginning)
    root_in->p           = map_.wps[i];
    root_in->fil         = emp_f;
  	root_in->parent      = NULL;               // No parent
  	root_in->cost        = 0.0;                // 0 distance.
    root_in->dontConnect = direct_hit_;
    root_in->connects2wp = false;
		root_ptrs_.push_back(root_in);
    num_root++;
	}
  printRoots();
}
void RRT::clearForNewPath()
{
  for (long unsigned int i = 0; i < all_wps_.size(); i++)
    all_wps_[i].clear();
  all_wps_.clear();
  clearTree();                    // Clear all of those tree pointer nodes
  std::vector<node*>().swap(root_ptrs_);
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
	for (unsigned int i = 0; i < root_ptrs_.size(); i++) // Delete every tree generated
		deleteNode(root_ptrs_[i]);
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
	for (unsigned int i = 0; i < root_ptrs_.size(); i++) // Delete every tree generated
		clearNode(root_ptrs_[i]);
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
  ROS_INFO("Initial North: %f, Initial East: %f, Initial Down: %f", pos.N, pos.E, pos.D);

  ROS_INFO("Number of Boundary Points: %lu",  map_.boundary_pts.size());
  for (long unsigned int i = 0; i < map_.boundary_pts.size(); i++)
  {
    ROS_INFO("Boundary: %lu, North: %f, East: %f, Down: %f", \
    i, map_.boundary_pts[i].N, map_.boundary_pts[i].E, map_.boundary_pts[i].D);
  }
  ROS_INFO("Number of Waypoints: %lu", map_.wps.size());
  for (long unsigned int i = 0; i < map_.wps.size(); i++)
  {
    ROS_INFO("WP: %lu, North: %f, East: %f, Down: %f", i, map_.wps[i].N, map_.wps[i].E, map_.wps[i].D);
  }
  ROS_INFO("Number of Cylinders: %lu", map_.cylinders.size());
  for (long unsigned int i = 0; i <  map_.cylinders.size(); i++)
  {
    ROS_INFO("Cylinder: %lu, North: %f, East: %f, Radius: %f, Height: %f", \
    i, map_.cylinders[i].N, map_.cylinders[i].E, map_.cylinders[i].R,  map_.cylinders[i].H);
  }
}
void RRT::printRoots()
{
  for (unsigned int i = 0; i < root_ptrs_.size(); i++)
    ROS_INFO("Waypoint %i, North: %f, East %f Down: %f", \
    i, root_ptrs_[i]->p.N, root_ptrs_[i]->p.E, root_ptrs_[i]->p.D);
}
} // end namespace theseus
