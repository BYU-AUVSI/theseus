#include <theseus/RRT.h>

namespace theseus
{
RRT::RRT(map_s map_in, unsigned int seed, ParamReader *input_file_in, RRT_input alg_input_in) // Setup the object
{
	input_file_ = input_file_in;
	alg_input_  = alg_input_in;
	D_          = alg_input_.D;	    // Distance between each RRT waypoint
	map_        = map_in;           // Get a copy of the terrain map
	RandGen rg_in(seed);            // Make a random generator object that is seeded
	rg_         = rg_in;            // Copy that random generator into the class.
	ppSetup();                      // default stuff for every algorithm that needs to be called after it recieves the map.
}
RRT::RRT()
{

}
RRT::~RRT()
{
  // These lines free the memory in the vectors... We were having problems with memory in the mapper class
  // These lines fixed it there so they were inserted here as well.
  for (unsigned int i = 0; i < lineMinMax_.size(); i++)
    std::vector<double>().swap(lineMinMax_[i]);
  std::vector<std::vector<double> >().swap(lineMinMax_);
  for (unsigned int i = 0; i < line_Mandb_.size(); i++)
    std::vector<double>().swap(line_Mandb_[i]);
  std::vector<std::vector<double> >().swap(line_Mandb_);
  std::vector<double>().swap(path_distances_);
	deleteTree();                          // Delete all of those tree pointer nodes
	std::vector<node*>().swap(root_ptrs_); // Free the memory of the vector.
}
void RRT::solveStatic(NED_s pos)        // This function solves for a path in between the waypoinnts (2 Dimensional)
{
  ROS_WARN("STARTING SOLVER");
  clearForNewPath();
	initializeTree(pos);
  taking_off_ = (-pos.D < input_file_->minFlyHeight);
	NED_s second2last_post_smoothed;
	second2last_post_smoothed.N = root_ptrs_[0]->NED.N - cos(input_file_->chi0);
	second2last_post_smoothed.E = root_ptrs_[0]->NED.E - sin(input_file_->chi0);
	second2last_post_smoothed.D = root_ptrs_[0]->NED.D;
	for (unsigned int i = 0; i < map_.wps.size(); i++)
	{
		ROS_INFO("MAIN LOOP: %i",i);
		path_clearance_ = input_file_->clearance;
		node *second2last = root_ptrs_[i];					// This will be set as the second to last waypoint
		double distance_in, fillet_angle;
		bool direct_shot = false;
		bool direct_connect = directConnection(i, &second2last_post_smoothed, &distance_in, &fillet_angle, &direct_shot);   //******* IMPORTANT
		if (direct_connect)
			second2last = closest_node_;
		developTree(i, direct_connect,second2last,&second2last_post_smoothed,&distance_in,&fillet_angle);                   //******* IMPORTANT
		smoother(direct_connect, i, &distance_in, &fillet_angle, &second2last_post_smoothed, direct_shot);                  //******* IMPORTANT
		calcPathDistance(i);
	}
	all_wps_[map_.wps.size() - 1].push_back(map_.wps[map_.wps.size() - 1]);	// Add the final waypoint to the waypoint list.
	computePerformance();
}
void RRT::clearForNewPath()
{
  line_starts_.clear();
  for (long unsigned int i = 0; i < all_wps_.size(); i++)
    all_wps_[i].clear();
  all_wps_.clear();
  path_distances_.clear();
  clearTree();											  // Clear all of those tree pointer nodes
  root_ptrs_.clear();
}
void RRT::clearForNewMap()
{
  for (unsigned int i = 0; i < lineMinMax_.size(); i++)
    std::vector<double>().swap(lineMinMax_[i]);
  std::vector<std::vector<double> >().swap(lineMinMax_);
  for (unsigned int i = 0; i < line_Mandb_.size(); i++)
    std::vector<double>().swap(line_Mandb_[i]);
  std::vector<std::vector<double> >().swap(line_Mandb_);
}
void RRT::newMap(map_s map_in)
{
  clearForNewMap();
  map_        = map_in;          // Get a copy of the terrain map
  ppSetup();                     // default stuff for every algorithm that needs to be called after it recieves the map.
}
void RRT::newSeed(unsigned int seed)
{
  RandGen rg_in(seed);          // Make a random generator object that is seeded
	rg_         = rg_in;           // Copy that random generator into the class.
}
bool RRT::checkDirectFan(NED_s second_wp, NED_s primary_wp, NED_s coming_from, node* next_root, NED_s* cea_out, double* din, double* anglin)
{
	bool found_at_least_1_good_path = false;
	double R = sqrt(pow(second_wp.N - primary_wp.N,2) + pow(second_wp.E - primary_wp.E,2) + pow(second_wp.D - primary_wp.D,2));

	double approach_angle = atan2(primary_wp.N - coming_from.N, primary_wp.E - coming_from.E) + M_PI;
	double approach_angleDEGREES = approach_angle*180.0 / M_PI;
	double beta, lambda, Q, phi, theta, zeta, gamma, d;
	NED_s cpa, cea, lea, fake_wp;

	// What should alpha be?
	double leave_angle = atan2(second_wp.N - primary_wp.N, second_wp.E - primary_wp.E);
	double leave_angleDEGREES = leave_angle*180.0 / M_PI;
	double alpha = atan2(second_wp.N - primary_wp.N, second_wp.E - primary_wp.E) - approach_angle;
	//double alphaDEGREES = alpha*180.0 / M_PI;
	while (alpha < -M_PI)
		alpha = alpha + 2 * M_PI;
	while (alpha > M_PI)
		alpha = alpha - 2 * M_PI;

	bool positive_angle = true;
	if (alpha < 0.0)
	{
		positive_angle = false;
		alpha = -1.0*alpha;
	}
	if (2.0*input_file_->turn_radius / R > 1.0 || 2.0*input_file_->turn_radius / R < -1.0)
		return false;
	double minAngle = asin(2.0*input_file_->turn_radius / R) + 8*M_PI/180.0;
	if (alpha < minAngle || (alpha > M_PI- minAngle && alpha < M_PI + minAngle))
		return false;

	beta = M_PI / 2 - alpha;
	lambda = M_PI - 2 * beta;
	Q = sqrt(R*(R - input_file_->turn_radius*sin(lambda) / sin(beta)) + input_file_->turn_radius*input_file_->turn_radius);
	phi = M_PI - asin(R*sin(beta) / Q);
	theta = acos(input_file_->turn_radius / Q);
	zeta = (2 * M_PI - phi - theta) / 2.0;
	gamma = M_PI - 2 * zeta;
	d = input_file_->turn_radius / tan(gamma / 2.0);

	fake_wp.N = primary_wp.N - d*sin(approach_angle);
	fake_wp.E = primary_wp.E - d*cos(approach_angle);
	fake_wp.D = primary_wp.D;

	// What should it be on the positive or on the negative side?
	if (positive_angle)
	{
		// Check the positive side
		cpa.N = primary_wp.N + input_file_->turn_radius*cos(approach_angle);
		cpa.E = primary_wp.E - input_file_->turn_radius*sin(approach_angle);
		cpa.D = primary_wp.D;

		cea.N = fake_wp.N + d*sin(gamma + approach_angle);
		cea.E = fake_wp.E + d*cos(gamma + approach_angle);
		cea.D = primary_wp.D;

		lea.N = primary_wp.N + R*sin(approach_angle + alpha);
		lea.E = primary_wp.E + R*cos(approach_angle + alpha);
		lea.D = primary_wp.D;

		if (flyZoneCheck(primary_wp, cea, input_file_->turn_radius, cpa, clearance_, false))
			if (flyZoneCheck(cea, lea, clearance_))
			{
				// Looks like things are going to work out for this maneuver!
				found_at_least_1_good_path = true;
				node *fake_child = new node;
				//node *normal_gchild = new node;
				fake_child->NED = fake_wp;
				fake_child->available_dist = 0;
				fake_child->parent = next_root;
				fake_child->distance = 2.0*zeta*input_file_->turn_radius;
				fake_child->path_type = 1;
				next_root->children.push_back(fake_child);
			}
	}
	else // Check the negative side
	{
		cpa.N = primary_wp.N - input_file_->turn_radius*cos(approach_angle);
		cpa.E = primary_wp.E + input_file_->turn_radius*sin(approach_angle);
		cpa.D = primary_wp.D;

		cea.N = fake_wp.N + d*sin(-gamma + approach_angle);
		cea.E = fake_wp.E + d*cos(-gamma + approach_angle);
		cea.D = primary_wp.D;

		lea.N = primary_wp.N + R*sin(approach_angle - alpha);
		lea.E = primary_wp.E + R*cos(approach_angle - alpha);
		lea.D = primary_wp.D;

		if (flyZoneCheck(primary_wp, cea, input_file_->turn_radius, cpa, clearance_, false))
			if (flyZoneCheck(cea, lea, clearance_))
			{
				// Looks like things are going to work out for this maneuver!
				found_at_least_1_good_path = true;
				node *fake_child = new node;
				node *normal_gchild = new node;
				fake_child->NED = fake_wp;
				fake_child->available_dist = 0;
				fake_child->parent = next_root;
				fake_child->distance = 2.0*zeta*input_file_->turn_radius;
				fake_child->path_type = 1;
				next_root->children.push_back(fake_child);
			}
	}
	*cea_out = cea;
	*din = sqrt(pow(fake_wp.N - cea.N, 2) + pow(fake_wp.E - cea.E, 2) + pow(fake_wp.D - cea.D, 2));
	*anglin = 2.0*zeta;
	return found_at_least_1_good_path;
}
bool RRT::checkCreateFan(NED_s primary_wp, NED_s coming_from, node* next_root)
{
	bool found_at_least_1_good_path = false;
	// Make sure that it is possible to go to the next waypoint

	double alpha = M_PI / 4.0;			// Start with 45.0 degrees
	double R = 3.0 * input_file_->turn_radius;
	int num_circle_trials = 10;				// Will try num_circle_trials on one side and num_circle_trials on the other side.
	double dalpha = (M_PI - alpha) / num_circle_trials;

	double approach_angle = atan2(primary_wp.N - coming_from.N, primary_wp.E - coming_from.E) + M_PI;
	double beta, lambda, Q, phi, theta, zeta, gamma, d;
	NED_s cpa, cea, lea, fake_wp;
	for (int j = 0; j < num_circle_trials; j++)
	{
		alpha = alpha + dalpha;
		beta = M_PI / 2 - alpha;
		lambda = M_PI - 2 * beta;
		Q = sqrt(R*(R - input_file_->turn_radius*sin(lambda) / sin(beta)) + input_file_->turn_radius*input_file_->turn_radius);
		phi = M_PI - asin(R*sin(beta) / Q);
		theta = acos(input_file_->turn_radius / Q);
		zeta = (2 * M_PI - phi - theta) / 2.0;
		gamma = M_PI - 2 * zeta;
		d = input_file_->turn_radius / tan(gamma / 2.0);

		// Check the positive side
		fake_wp.N = primary_wp.N - d*sin(approach_angle);
		fake_wp.E = primary_wp.E - d*cos(approach_angle);
		fake_wp.D = primary_wp.D;

		cpa.N = primary_wp.N + input_file_->turn_radius*cos(approach_angle);
		cpa.E = primary_wp.E - input_file_->turn_radius*sin(approach_angle);
		cpa.D = primary_wp.D;

		cea.N = fake_wp.N + d*sin(gamma + approach_angle);
		cea.E = fake_wp.E + d*cos(gamma + approach_angle);
		cea.D = primary_wp.D;

		lea.N = primary_wp.N + R*sin(approach_angle + alpha);
		lea.E = primary_wp.E + R*cos(approach_angle + alpha);
		lea.D = primary_wp.D;

		if (flyZoneCheck(primary_wp, cea, input_file_->turn_radius, cpa, clearance_, false))
			if (flyZoneCheck(cea, lea, clearance_))
			{
				// Looks like things are going to work out for this maneuver!
				found_at_least_1_good_path = true;
				node *fake_child = new node;
				node *normal_gchild = new node;
				fake_child->NED = fake_wp;
				fake_child->available_dist = 0;
				fake_child->parent = next_root;
				fake_child->distance = 2.0*zeta*input_file_->turn_radius;
				fake_child->path_type = 1;
				fake_child->line_start = next_root->NED;
				next_root->children.push_back(fake_child);

				normal_gchild->NED = lea;
				normal_gchild->available_dist = sqrt(R*(R - input_file_->turn_radius*sin(lambda) / sin(beta)));
				normal_gchild->parent = fake_child;
				normal_gchild->distance = normal_gchild->available_dist;
				normal_gchild->path_type = 1;
				normal_gchild->line_start = cea;
				fake_child->children.push_back(normal_gchild);
			}
		// Check the negative side
		cpa.N = primary_wp.N - input_file_->turn_radius*cos(approach_angle);
		cpa.E = primary_wp.E + input_file_->turn_radius*sin(approach_angle);
		cpa.D = primary_wp.D;

		cea.N = fake_wp.N + d*sin(-gamma + approach_angle);
		cea.E = fake_wp.E + d*cos(-gamma + approach_angle);
		cea.D = primary_wp.D;

		lea.N = primary_wp.N + R*sin(approach_angle - alpha);
		lea.E = primary_wp.E + R*cos(approach_angle - alpha);
		lea.D = primary_wp.D;

		if (flyZoneCheck(primary_wp, cea, input_file_->turn_radius, cpa, clearance_, false))
			if (flyZoneCheck(cea, lea, clearance_))
			{
				// Looks like things are going to work out for this maneuver!
				found_at_least_1_good_path = true;
				node *fake_child = new node;
				node *normal_gchild = new node;
				fake_child->NED = fake_wp;
				fake_child->available_dist = 0;
				fake_child->parent = next_root;
				fake_child->distance = 2.0*zeta*input_file_->turn_radius;
				fake_child->path_type = 1;
				fake_child->line_start = next_root->NED;
				next_root->children.push_back(fake_child);

				normal_gchild->NED = lea;
				normal_gchild->available_dist = sqrt(R*(R - input_file_->turn_radius*sin(lambda) / sin(beta)));
				normal_gchild->parent = fake_child;
				normal_gchild->distance = normal_gchild->available_dist;
				normal_gchild->path_type = 1;
				normal_gchild->line_start = cea;
				fake_child->children.push_back(normal_gchild);
			}
	}
	return found_at_least_1_good_path;
}
bool RRT::checkFillet(NED_s par, NED_s mid, NED_s nex, double avail_dis, double* din, double* cangle, NED_s* line_start)
{
	bool found_feasible_link = true;
	// Calculate the fillet and check to see if it is a good fit.
	// a dot b = ||A|| * ||B|| * cos(theta)
	double a_dot_b = (par.E - mid.E)*(nex.E - mid.E) + (par.N - mid.N)*(nex.N - mid.N) + (par.D - mid.D)*(nex.D - mid.D);
	double A = sqrt(pow(par.E - mid.E, 2) + pow(par.N - mid.N, 2) + pow(par.D - mid.D, 2));
	double B = sqrt(pow(nex.N - mid.N, 2) + pow(nex.E - mid.E, 2) + pow(nex.D - mid.D, 2));
	double Fangle = acos((a_dot_b) / (A*B));
	double turn_radius = input_file_->turn_radius;
	double distance_in = turn_radius / tan(Fangle / 2.0);// Notice this equation was written incorrectly in the UAV book //sqrt(turn_radius*turn_radius / sin(Fangle / 2.0) / sin(Fangle / 2.0) - turn_radius*turn_radius);
	if (distance_in > avail_dis || distance_in > sqrt(pow(mid.N - nex.N, 2) + pow(mid.E - nex.E, 2) + pow(mid.D - nex.D, 2)))
		found_feasible_link = false;
	else
	{
		NED_s ps, pe, cp;
		double theta = atan2(nex.N - mid.N, nex.E - mid.E);
		pe.N = (mid.N) + sin(theta)*distance_in;
		pe.E = (mid.E) + cos(theta)*distance_in;
		pe.D = mid.D;
		double gamma = atan2(par.N - mid.N, par.E - mid.E);
		ps.N = (mid.N) + sin(gamma)*distance_in;
		ps.E = (mid.E) + cos(gamma)*distance_in;
		ps.D = mid.D;
		// Find out whether it is going to the right (cw) or going to the left (ccw)
		// Use the cross product to see if it is cw or ccw
		bool ccw;
		double cross_product = ((mid.E - ps.E)*(pe.N - mid.N) - (mid.N - ps.N)*(pe.E - mid.E));
		if (cross_product < 0)
			ccw = false;
		else
			ccw = true;
		if (ccw)
		{
			cp.N = (mid.N) + sin(gamma - Fangle / 2.0)*turn_radius / sin(Fangle / 2.0);
			cp.E = (mid.E) + cos(gamma - Fangle / 2.0)*turn_radius / sin(Fangle / 2.0);
		}
		else
		{
			cp.N = (mid.N) + sin(gamma + Fangle / 2.0)*turn_radius / sin(Fangle / 2.0);
			cp.E = (mid.E) + cos(gamma + Fangle / 2.0)*turn_radius / sin(Fangle / 2.0);
		}
		cp.D = mid.D;
		if (flyZoneCheck(ps, pe, turn_radius, cp, clearance_, ccw) == false)
			found_feasible_link = false;
		*line_start = pe;
	}
	*din = distance_in;
	*cangle = 2.0*atan(distance_in / turn_radius);
	return found_feasible_link;
}
void RRT::initializeTree(NED_s pos)
{
	// Set up all of the roots
	node *root_in = new node;             // Starting position of the tree (and the waypoint beginning)
	NED_s starting_point;
	starting_point.N = pos.N;
	starting_point.E = pos.E;
	starting_point.D = pos.D;

	root_in->NED = starting_point;
	root_in->parent = NULL;               // No parent
	root_in->distance = 0.0;              // 0 distance.
	root_in->available_dist = 0.0;        // No available distance, (No parent assumption)
	root_in->path_type = 0;               // straight lines for now at the primary waypoints.
	root_in->line_start = root_in->NED;   // The line start is set to it's own location, for now.
	root_ptrs_.push_back(root_in);
  int num_root = 0;
  ROS_INFO("Root Number %i, North: %f, East %f Down: %f", num_root, root_ptrs_[num_root]->NED.N, root_ptrs_[num_root]->NED.E, root_ptrs_[num_root]->NED.D);
	num_root++;
  for (unsigned int i = 0; i < map_.wps.size() - 1; i++)
	{
		node *root_in = new node;           // Starting position of the tree (and the waypoint beginning)
		root_in->NED = map_.wps[i];
		root_in->parent = NULL;             // No parent
		root_in->distance = 0.0;            // 0 distance.
		root_in->available_dist = 0.0;      // No available distance, (No parent assumption)
		root_in->path_type = 0;             // straight lines for now at the primary waypoints.
		root_in->line_start = root_in->NED; // The line start is set to it's own location, for now.
		root_ptrs_.push_back(root_in);
    ROS_INFO("Root Number %i, North: %f, East %f Down: %f", num_root, root_ptrs_[num_root]->NED.N, root_ptrs_[num_root]->NED.E, root_ptrs_[num_root]->NED.D);
    num_root++;
	}
  unsigned int i = map_.wps.size() - 1;
  ROS_INFO("Waypoint %i, North: %f, East %f Down: %f", i, map_.wps[i].N, map_.wps[i].E, map_.wps[i].D);
}
bool RRT::directConnection(unsigned int i, NED_s* second2last_post_smoothed, double* distance_in, double* fillet_angle, bool* direct_shot)
{
	// Variables created for this chuck of code in a function
	bool reached_next_wp = false;
	NED_s P, line_start;
	node* root = root_ptrs_[i];
	double distance;

	// Make sure that the clearance level is right. Mostly this little bit checks to make sure that there is appropriate room to get waypoints close to the floor or ceiling.
	clearance_ = input_file_->clearance;
	clearance_ = (-map_.wps[i].D - input_file_->minFlyHeight < clearance_) ? -map_.wps[i].D - input_file_->minFlyHeight : clearance_;
	clearance_ = (map_.wps[i].D + input_file_->maxFlyHeight < clearance_) ? map_.wps[i].D + input_file_->maxFlyHeight : clearance_;
	if (i > 0)
	{
		clearance_ = (-map_.wps[i - 1].D - input_file_->minFlyHeight < clearance_) ? -map_.wps[i - 1].D - input_file_->minFlyHeight : clearance_;
		clearance_ = (map_.wps[i - 1].D + input_file_->maxFlyHeight < clearance_) ? map_.wps[i - 1].D + input_file_->maxFlyHeight : clearance_;
	}
	if (clearance_ <= 0)
		ROS_WARN("ERROR. CLEARANCE IS 0 OR LESS THAN 0.");

	// Check to see if it is possible to go straight to the next path
	ROS_INFO("Checking direct connect.");
	NED_s coming_from;
	if (i == 0)
	{
		coming_from = root->NED;
		reached_next_wp = flyZoneCheck(root->NED, map_.wps[i], clearance_);
		closest_node_ = root;
		if (reached_next_wp)
			reached_next_wp = checkSlope(root->line_start, map_.wps[i]);
		line_start = root->NED;
	}
	else
	{
		P = map_.wps[i];
		distance = INFINITY; // Some crazy big number to start out with, maybe look into HUGE_VAL or Inf - worried about embedded implementation.
		double distance_gchild;
		bool found_clean_path = false;
		// Check to see if you can make one turn radius and then go to the next waypoint.
		NED_s cea;
		if (i <= map_.wps.size() - 1 && checkDirectFan(map_.wps[i], root->NED, *second2last_post_smoothed, root, &cea, distance_in, fillet_angle))
		{
			line_start = cea;
			if (checkSlope(line_start, map_.wps[i]))
			{
				closest_node_ = root->children[root->children.size() - 1];
				found_clean_path = true;
				*direct_shot = true;
				coming_from = closest_node_->NED;
				reached_next_wp = true;
				closest_node_->available_dist = 0.0;
				distance = sqrt(pow(root->NED.N - closest_node_->NED.N, 2) + pow(root->NED.E - closest_node_->NED.E, 2) + pow(root->NED.D - closest_node_->NED.D, 2));
				closest_node_->distance = distance;
			}
		}
		else
		{
			for (unsigned int j = 0; j < root->children.size(); j++)
				for (unsigned int k = 0; k < root->children[j]->children.size(); k++)
				{
					distance_gchild = sqrt(pow(P.N - root->children[j]->children[k]->NED.N, 2) + pow(P.E - root->children[j]->children[k]->NED.E, 2) + pow(P.D - root->children[j]->children[k]->NED.D, 2));
					closest_node_gchild_ = findClosestNode(root->children[j]->children[k], P, root->children[j]->children[k], &distance_gchild);
					if (distance_gchild < distance)
					{
						closest_node_ = closest_node_gchild_;
						distance = distance_gchild;
					}
				}
			coming_from = closest_node_->NED;
			reached_next_wp = flyZoneCheck(coming_from, map_.wps[i], clearance_);
			if (reached_next_wp && checkFillet(closest_node_->parent->NED, closest_node_->NED, map_.wps[i], closest_node_->available_dist, distance_in, fillet_angle, &line_start))
				reached_next_wp = checkSlope(line_start, map_.wps[i]);
		}
	}
	if (reached_next_wp == true && i < map_.wps.size() - 1) // This if statement handles setting waypoints to fly out of the primary waypoints.
		if (checkCreateFan(map_.wps[i], coming_from, root_ptrs_[i + 1]) == false)
			reached_next_wp = false;
	if (reached_next_wp && i + 1 < root_ptrs_.size())
		root_ptrs_[i + 1]->line_start = line_start;
	return reached_next_wp;
}
void RRT::developTree(unsigned int i, bool reached_next_wp, node* second2last, NED_s* second2last_post_smoothed, double* distance_in, double* fillet_angle)
{
	// Variables needed to create for this function
	double distance;
	NED_s P, line_start;
	node* root = root_ptrs_[i];
	double clearanceP = clearance_;
	int clearance_drops = 0;
	double added_nodes = 0.0;								// Keep a record of how many nodes are added so that the algorithm doesn't get stuck.
	ROS_INFO("Developing branches");
	while (reached_next_wp == false)
	{
		node *vpos = new node;								// vpos is the next node to add to the tree
		bool found_feasible_link = false;
		// Once you found a node to add to the tree that doesn't intersect with an obstacle, add it to the tree
		unsigned int p_count(0);
		while (found_feasible_link == false)
		{
			// Generate random P until it is within boundaries and not on an obstacle.
			P.N = rg_.randLin()*(maxNorth_ - minNorth_) + minNorth_;
			P.E = rg_.randLin()*(maxEast_ - minEast_) + minEast_;
			P.D = map_.wps[i].D;
			while (flyZoneCheck(P, clearanceP) == false)
			{
				P.N = rg_.randLin()*(maxNorth_ - minNorth_) + minNorth_;
				P.E = rg_.randLin()*(maxEast_ - minEast_) + minEast_;
				P.D = map_.wps[i].D;
			}
			p_count++;
			if (p_count == floor(iters_limit_ / 2.0) || p_count == floor(iters_limit_*3.0 / 4.0) || p_count == floor(iters_limit_*7.0 / 8.0))
			{
				clearanceP = clearanceP / 6.0;
				ROS_WARN("DECREASING THE CLEARANCE LEVEL");
        ros::shutdown();
				clearance_ = clearance_ / 2.0;
				ROS_WARN("CLEARANCE: %f",clearance_);
				path_clearance_ = (clearance_ < path_clearance_) ? clearance_ : path_clearance_;
			}
			distance = sqrt(pow(P.N - root->NED.N, 2) + pow(P.E - root->NED.E, 2) + pow(P.D - root->NED.D, 2));

			// Find the closest node to the point P, if you are not trying to get to waypoint 1 (which is the second waypoint), then don't accept the root or the children of the root.
			if (i == 0)
				closest_node_ = findClosestNode(root, P, root, &distance);
			else
			{
				distance = INFINITY; // Some crazy big number to start out with - maybe look into HUGE_VAL or Inf - worried about embedded implementation.
				double distance_gchild;
				for (unsigned int j = 0; j < root->children.size(); j++)
					for (unsigned int k = 0; k < root->children[j]->children.size(); k++)
					{
						distance_gchild = sqrt(pow(P.N - root->children[j]->children[k]->NED.N, 2) + pow(P.E - root->children[j]->children[k]->NED.E, 2) + pow(P.D - root->children[j]->children[k]->NED.D, 2));
						closest_node_gchild_ = findClosestNode(root->children[j]->children[k], P, root->children[j]->children[k], &distance_gchild);
						if (distance_gchild < distance)
						{
							closest_node_ = closest_node_gchild_;
							distance = distance_gchild;
						}
					}
			}
			double theta = atan2(P.N - closest_node_->NED.N, P.E - closest_node_->NED.E);

			// Go a distance D_ along the line from closest node to P to find the next node position vpos
			if (alg_input_.uniform2P)
				D_ = rg_.randLin()*distance;
			else if (alg_input_.gaussianD)
				ROS_WARN("gaussianD is depreciated.");
			vpos->NED.N = (closest_node_->NED.N) + sin(theta)*D_;
			vpos->NED.E = (closest_node_->NED.E) + cos(theta)*D_;

			if (map_.wps[i].D > closest_node_->NED.D - tan(input_file_->climb_angle)*D_ && map_.wps[i].D < closest_node_->NED.D + tan(input_file_->descend_angle)*D_)
				vpos->NED.D = map_.wps[i].D;
			else if (map_.wps[i].D > closest_node_->NED.D)
				vpos->NED.D = closest_node_->NED.D + tan(input_file_->descend_angle)*D_;
			else
				vpos->NED.D = closest_node_->NED.D - tan(input_file_->descend_angle)*D_;

			// If this path is good move on.
			if (flyZoneCheck(closest_node_->NED, vpos->NED, clearance_))
			{
				found_feasible_link = true;
				vpos->parent = closest_node_;
				if (alg_input_.path_type == 1 && root != vpos->parent)								// If the path type is fillets, check to see if the fillet is possible.
          found_feasible_link = checkFillet(closest_node_->parent->NED, closest_node_->NED, vpos->NED, closest_node_->available_dist, distance_in, fillet_angle, &line_start);
      }
		}
		// add the new node to the tree
		vpos->line_start = line_start;
		closest_node_->children.push_back(vpos);
		vpos->distance = sqrt(pow(closest_node_->NED.N - vpos->NED.N, 2) + pow(closest_node_->NED.E - vpos->NED.E, 2) + pow(closest_node_->NED.D - vpos->NED.D, 2));
		if (alg_input_.path_type == 1 && vpos->parent != root)
		{
			// Adjust the vpos->distance to account for the turning radius ( a little bit smaller.)
			vpos->available_dist = vpos->distance - *distance_in;
			vpos->distance = vpos->distance - 2.0*(*distance_in) + (*fillet_angle)*input_file_->turn_radius;
		}
		else
			vpos->available_dist = vpos->distance;

		// Make provisions so that the algorithm doesn't hang
		added_nodes++;
		if (added_nodes == floor(iters_limit_ / 2.0) || added_nodes == floor(iters_limit_*3.0 / 4.0) || added_nodes == floor(iters_limit_*7.0 / 8.0))
		{
			ROS_WARN("DECREASING THE CLEARANCE LEVEL");
			clearance_ = clearance_ / (2.0*++clearance_drops);
			ROS_WARN("CLEARANCE: %f",clearance_);
			path_clearance_ = (clearance_ < path_clearance_) ? clearance_ : path_clearance_;
		}
		if (added_nodes >= iters_limit_)
		{
			ROS_ERROR("WARNING -- ALGORITHM FAILED TO CONVERGE. RESULTS WILL VIOLATE AN OBSTACLE");
			reached_next_wp = true;
			second2last = vpos;
			line_start = vpos->NED;
		}

		// Check to see if it is possible to go from this newly added node to the next primary waypoint
		if (alg_input_.connect_to_end && flyZoneCheck(vpos->NED, map_.wps[i], clearance_) && checkSlope(line_start, map_.wps[i]))
		{
			reached_next_wp = true;								// Set the flag
			second2last = vpos;
			if (alg_input_.path_type == 1 && root != vpos)								// If the path type is fillets, check to see if the fillet is possible.
				reached_next_wp = checkFillet(vpos->parent->NED, vpos->NED, map_.wps[i], vpos->available_dist, distance_in, fillet_angle,&line_start);
		}								// Set the flag
		if (!alg_input_.connect_to_end && sqrt(pow(map_.wps[i].N - vpos->NED.N, 2) + pow(map_.wps[i].E - vpos->NED.E, 2) + pow(map_.wps[i].D - vpos->NED.D, 2)) < D_ && flyZoneCheck(vpos->NED, map_.wps[i], clearance_) && checkSlope(line_start, map_.wps[i]))
		{
			reached_next_wp = true;								// Set the flag
			second2last = vpos;
			if (alg_input_.path_type == 1 && root != vpos)								// If the path type is fillets, check to see if the fillet is possible.
				reached_next_wp = checkFillet(vpos->parent->NED, vpos->NED, map_.wps[i], vpos->available_dist, distance_in, fillet_angle, &line_start);
		}
		if (reached_next_wp == true && i < map_.wps.size() - 1) // This if statement handles setting waypoints to get out of the primary waypoints.
		{
			if (checkCreateFan(map_.wps[i], vpos->NED, root_ptrs_[i + 1]) == false)
				reached_next_wp = false;
		}
	}
	// We can go to the next waypoint!
	// The following code wraps up the algorithm.
	clearance_ = input_file_->clearance;									// Bump up the clearance level again.
	node *final_node = new node;
	final_node->NED = map_.wps[i];
	final_node->line_start = line_start;
	second2last->children.push_back(final_node);
	*second2last_post_smoothed = second2last->NED;
	final_node->parent = second2last;
	final_node->distance = sqrt(pow(final_node->NED.N - second2last->NED.N, 2) + pow(final_node->NED.E - second2last->NED.E, 2) + pow(final_node->NED.D - second2last->NED.D, 2));
	if (alg_input_.path_type == 1 && final_node->parent != root)
		final_node->distance = final_node->distance - 2.0*(*distance_in) + (*fillet_angle)*input_file_->turn_radius;
	// populate all wps by working back up the tree
	std::stack<node*> wpstack;
	node *current_node = final_node;
	while (current_node != root)
	{
		wpstack.push(current_node);
		current_node = current_node->parent;
	}
	wpstack.push(root);

	// Now put all of the waypoints into the all_wps_ vector
	std::vector<NED_s> wps_to_PrimaryWP;
	std::vector<NED_s> line_starts_to_PrimaryWP;
	while (!wpstack.empty())
	{
		wps_to_PrimaryWP.push_back(wpstack.top()->NED);
		line_starts_to_PrimaryWP.push_back(wpstack.top()->line_start);
		wpstack.pop();
	}
	all_wps_.push_back(wps_to_PrimaryWP);
	line_starts_.push_back(line_starts_to_PrimaryWP);
	wps_to_PrimaryWP.clear();
	line_starts_to_PrimaryWP.clear();
}
void RRT::smoother(bool skip_smoother, unsigned int i, double* distance_in, double* fillet_angle, NED_s* second2last_post_smoothed, bool direct_shot)
{
	node* root = root_ptrs_[i];
	//skip_smoother = true; // Uncommmenting this line turns the smoother off. It can be helpful for debugging.

	// Smooth Out the path (Algorithm 11 in the UAV book)
	if (skip_smoother == false)
	{
		ROS_INFO("Smoothing Path");
		std::vector<NED_s> path_smoothed;
		std::vector<NED_s> line_starts_smoothed;
		std::vector<double> available_ds;
		unsigned int i_node = 0;	// i_node is which index of the SMOOTHED PATH you are on.
		unsigned int j_node = 1;	// j_node is which index of the ROUGH PATH you are on.
		double line_Distance;
		path_smoothed.push_back(all_wps_[i][0]);
		line_starts_smoothed.push_back(line_starts_[i][0]);
		NED_s line_start;
		if (i != 0)
		{
			path_smoothed.push_back(all_wps_[i][1]);
			line_starts_smoothed.push_back(line_starts_[i][1]);
			i_node++;
			j_node++;
			line_Distance = sqrt(pow(path_smoothed[i_node].N - path_smoothed[i_node - 1].N, 2.) + pow(path_smoothed[i_node].E - path_smoothed[i_node - 1].E, 2) + pow(path_smoothed[i_node].D - path_smoothed[i_node - 1].D, 2));
			available_ds.push_back(line_Distance - *distance_in);

			path_smoothed.push_back(all_wps_[i][2]);
			line_starts_smoothed.push_back(line_starts_[i][2]);
			i_node++;
			j_node++;
			line_Distance = sqrt(pow(path_smoothed[i_node].N - path_smoothed[i_node - 1].N, 2.) + pow(path_smoothed[i_node].E - path_smoothed[i_node - 1].E, 2) + pow(path_smoothed[i_node].D - path_smoothed[i_node - 1].D, 2));
			available_ds.push_back(line_Distance - *distance_in);
		}
		while (j_node < all_wps_[i].size())
		{
			bool bad_path_flag = false;
			if ((alg_input_.path_type == 0 || i_node == 0) && all_wps_[i].size() >= j_node + 2 && (flyZoneCheck(path_smoothed[i_node], all_wps_[i][j_node + 1], path_clearance_) == false || checkMaxSlope(line_starts_smoothed[i_node], all_wps_[i][j_node + 1]) == false))
				bad_path_flag = true;
			else if (alg_input_.path_type == 1 && all_wps_[i].size() >= j_node + 2 && (flyZoneCheck(path_smoothed[i_node], all_wps_[i][j_node + 1], path_clearance_) == false || checkMaxSlope(line_starts_smoothed[i_node], all_wps_[i][j_node + 1]) == false))
				bad_path_flag = true;
			if (alg_input_.path_type == 1 && all_wps_[i].size() >= j_node + 2 && i_node >= 1 && checkFillet(path_smoothed[i_node - 1], path_smoothed[i_node], all_wps_[i][j_node + 1], available_ds[i_node - 1], distance_in, fillet_angle, &line_start) == false)
				bad_path_flag = true;
			if (i != 0 && alg_input_.path_type == 1 && j_node == all_wps_[i].size() - 3 && bad_path_flag == false)
			{
				double temp_available_ds = sqrt(pow(path_smoothed[i_node].N - all_wps_[i][j_node + 2].N, 2) + pow(path_smoothed[i_node].E - all_wps_[i][j_node + 2].E, 2) + pow(path_smoothed[i_node].D - all_wps_[i][j_node + 2].D, 2)) - *distance_in;
				if (checkFillet(path_smoothed[i_node], all_wps_[i][j_node + 1], all_wps_[i][j_node + 2], temp_available_ds, distance_in, fillet_angle,&line_start) == false)
				{
					bad_path_flag = true;
					// If this is true then really you should check the next one back.
					// Now we are working backwards to find a good path.
					while (bad_path_flag)
					{
						j_node--;
						temp_available_ds = sqrt(pow(path_smoothed[i_node].N - all_wps_[i][j_node + 1].N, 2) + pow(path_smoothed[i_node].E - all_wps_[i][j_node + 1].E, 2) + pow(path_smoothed[i_node].D - all_wps_[i][j_node + 1].D, 2)) - *distance_in;
						if (flyZoneCheck(path_smoothed[i_node], all_wps_[i][j_node + 1], path_clearance_))
							if (checkFillet(path_smoothed[i_node - 1], path_smoothed[i_node], all_wps_[i][j_node + 1], temp_available_ds, distance_in, fillet_angle, &line_start))
								bad_path_flag = false;
					}
					bad_path_flag = true;
				}
			}
			if (j_node + 1 == all_wps_[i].size() - 1 && bad_path_flag == false && i != map_.wps.size()-1)
			{
				int previous_fan_nodes = root_ptrs_[i + 1]->children.size();
				if (checkCreateFan(map_.wps[i], path_smoothed[path_smoothed.size() - 1], root_ptrs_[i + 1]))
				{
					// Delete the other nodes
					for (unsigned int j = 0; j < root_ptrs_[i+1]->children.size(); j++)		// Delete every tree generated
						deleteNode(root_ptrs_[i+1]->children[j]);
					std::vector<node*>().swap(root_ptrs_[i+1]->children);
					root_ptrs_[i+1]->children.clear();
					checkCreateFan(map_.wps[i], path_smoothed[path_smoothed.size() - 1], root_ptrs_[i + 1]);
				}
				else
					bad_path_flag == true; // Make sure that jnode + 1 is added.
			}
			if (bad_path_flag) // Try adding the second to last node every time. Avoids problems with smoothing out the last fillet.
			{
				path_smoothed.push_back(all_wps_[i][j_node]);
				line_starts_smoothed.push_back(line_start);
				i_node++;
				line_Distance = sqrt(pow(path_smoothed[i_node].N - path_smoothed[i_node - 1].N, 2.) + pow(path_smoothed[i_node].E - path_smoothed[i_node - 1].E, 2) + pow(path_smoothed[i_node].D - path_smoothed[i_node - 1].D, 2));
				available_ds.push_back(line_Distance - *distance_in);
			}
			j_node++;
		}
		// Check to see if the fan can be created better
		// If so, change path_smoothed[1] and delete path_smoothed[2]
		NED_s cea;
		if (i > 0 && i <= map_.wps.size() && direct_shot == false && path_smoothed.size() >= 4 && checkDirectFan(path_smoothed[3], root->NED, all_wps_[i - 1][all_wps_[i - 1].size() - 1], root, &cea, distance_in, fillet_angle))
		{
			if (path_smoothed.size() >= 4 && flyZoneCheck(cea, path_smoothed[3], path_clearance_))
			{
				path_smoothed[1] = root->children[root->children.size() - 1]->NED;
				line_starts_smoothed[1] = cea;
				path_smoothed.erase(path_smoothed.begin() + 2);
				line_starts_smoothed.erase(line_starts_smoothed.begin() + 2);
				ROS_INFO("SMOOTHED THE FAN");
			}
		}

		all_wps_[i].swap(path_smoothed);
		line_starts_[i].swap(line_starts_smoothed);
		*second2last_post_smoothed = all_wps_[i][all_wps_[i].size() - 1];
	}
	else
		all_wps_[i].erase(all_wps_[i].begin() + all_wps_[i].size() - 1);
	if (taking_off_ == true)
		taking_off_ = false;
}
bool RRT::checkSlope(NED_s beg, NED_s en)
{
	double slope = atan2(-1.0*(en.D - beg.D), sqrt(pow(beg.N - en.N, 2) + pow(beg.E - en.E, 2)));
	if (slope < -1.0*input_file_->descend_angle || slope > input_file_->climb_angle)
		return false;
	return true;
}
bool RRT::checkMaxSlope(NED_s beg, NED_s en)
{
	double slope = atan2(-1.0*(en.D - beg.D), sqrt(pow(beg.N - en.N, 2) + pow(beg.E - en.E, 2)));
	if (slope < -1.0*input_file_->max_descend_angle || slope > input_file_->max_climb_angle)
		return false;
	return true;
}
void RRT::calcPathDistance(unsigned int i)
{
	// Determine the true path distance. (This kind of covers up some mistakes above...)
	double final_distance = 0;
	for (unsigned j = 0; j < all_wps_[i].size() - 1; j++)
	{
		final_distance += sqrt(pow(all_wps_[i][j].N - all_wps_[i][j + 1].N, 2) + pow(all_wps_[i][j].E - all_wps_[i][j + 1].E, 2) + pow(all_wps_[i][j].D - all_wps_[i][j + 1].D, 2));
	}
	final_distance += sqrt(pow(all_wps_[i][all_wps_[i].size() - 1].N - map_.wps[i].N, 2) + pow(all_wps_[i][all_wps_[i].size() - 1].E - map_.wps[i].E, 2) + pow(all_wps_[i][all_wps_[i].size() - 1].D - map_.wps[i].D, 2));
	path_distances_.push_back(final_distance);
	ROS_INFO("PATH DISTANCE: %f",path_distances_[i]);
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
}
void RRT::ppSetup()
{
	// This function is called by the child class at the end of their constructor. There are certain things
	// that need to be set first by that function and then the base class needs to do stuff - the base class constructor
	// would not do it in the right order.

	// This function pulls in the competition boundaries and finds the minimum and maxium North and East positions
	// Even though the mapper class does this, the mapper class won't be put into ROSplane, so it is done here
	// which should be put into ROSplane pathplanner
	is3D_ = input_file_->is3D;
	NED_s boundary_point;
	bool setFirstValues = true;
	for (unsigned int i = 0; i < map_.boundary_pts.size(); i++)
	{
		boundary_point = map_.boundary_pts[i];
		if (setFirstValues == false)
		{
			maxNorth_ = (boundary_point.N > maxNorth_) ? boundary_point.N : maxNorth_; // if new N is greater than maxN, set maxN = new N
			minNorth_ = (boundary_point.N < minNorth_) ? boundary_point.N : minNorth_;
			maxEast_  = (boundary_point.E > maxEast_)  ? boundary_point.E : maxEast_;
			minEast_  = (boundary_point.E < minEast_)  ? boundary_point.E : minEast_;
		}
		else
		{
			maxNorth_ = boundary_point.N;
			minNorth_ = boundary_point.N;
			maxEast_  = boundary_point.E;
			minEast_  = boundary_point.E;
			setFirstValues = false;
		}
	}
	clearance_    = input_file_->clearance;		 // Clearance for the path (m)
	minFlyHeight_ = input_file_->minFlyHeight; // 30.48 m = 100 ft. // This still needs to add in the take off altitude
	maxFlyHeight_ = input_file_->maxFlyHeight; // 228.6 m = 750 ft. // This still needs to add in the take off altitude
	iters_limit_  = input_file_->iters_limit;
	// Also do all of the calculations on the boundary lines.
	setupFlyZoneCheck();
}
void RRT::setupFlyZoneCheck()				// This function sets up alll of the stuff needed for the flyZoneCheck functions - mostly calculates the y = mx + b for each boundary line
{
	//vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv These lines are used to prep the flyZoneCheck() algorithm
	std::vector<double> NminNmaxEminEmax;       // Yeah, this is a riduculous name...
	std::vector<double> mb;                     // Vector of slope and intercepts
	nBPts_ = map_.boundary_pts.size();            // Number of Boundary Points
	double m, b, w, m_w;
	for (unsigned int i = 0; i < nBPts_; i++)    // Loop through all points
	{
		// Find the min and max of North and East coordinates on the line connecting two points.
		NminNmaxEminEmax.push_back(std::min(map_.boundary_pts[i].N, map_.boundary_pts[(i + 1) % nBPts_].N));
		NminNmaxEminEmax.push_back(std::max(map_.boundary_pts[i].N, map_.boundary_pts[(i + 1) % nBPts_].N));
		NminNmaxEminEmax.push_back(std::min(map_.boundary_pts[i].E, map_.boundary_pts[(i + 1) % nBPts_].E));
		NminNmaxEminEmax.push_back(std::max(map_.boundary_pts[i].E, map_.boundary_pts[(i + 1) % nBPts_].E));
		lineMinMax_.push_back(NminNmaxEminEmax);
		NminNmaxEminEmax.clear();
		// Find the slope and intercept
		m = (map_.boundary_pts[(i + 1) % nBPts_].N - map_.boundary_pts[i].N) / (map_.boundary_pts[(i + 1) % nBPts_].E - map_.boundary_pts[i].E);
		b = -m*map_.boundary_pts[i].E + map_.boundary_pts[i].N;
		w = (-1.0 / m);
		m_w = m - w;
		mb.push_back(m);
		mb.push_back(b);
		mb.push_back(w);
		mb.push_back(m_w);
		line_Mandb_.push_back(mb);
		mb.clear();
	}
	//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ These lines are used to set up the flyZoneCheck() algorithm.s
}

//****************************************************************LINE*************************************************************************************
//****************************************************************LINE*************************************************************************************
//****************************************************************LINE*************************************************************************************
bool RRT::flyZoneCheck(const NED_s ps, const NED_s pe, const double r) // Point start, point end, radius (clearance)
{
	// THIS FUNCTION IS REALLY IMPORTANT. IT DETERMINES IF A LINE CONNECTING ps AND pe INTERSECT ANY OBSTACLE OR GET WITHIN r OF ANY OBSTACLE.
	// This function should be good for 3 dimensions.
	// Preliminary Calculations about the line connecting ps and pe
  double pathMinMax[4];
	double path_Mandb[4];
	pathMinMax[0] = std::min(ps.N, pe.N);
	pathMinMax[1] = std::max(ps.N, pe.N);
	pathMinMax[2] = std::min(ps.E, pe.E);
	pathMinMax[3] = std::max(ps.E, pe.E);
	path_Mandb[0] = (pe.N - ps.N) / (pe.E - ps.E);
	path_Mandb[1] = pe.N - path_Mandb[0] * pe.E;
	path_Mandb[2] = -1.0 / path_Mandb[0];
	path_Mandb[3] = path_Mandb[0] - path_Mandb[2];
	double Ei, Ni;

	//vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
	//vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv Check for Boundary Lines vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
	bool withinBoundaries_ps, withinBoundaries_pe;
	int crossed_lines_ps(0), crossed_lines_pe(0);	// This is a counter of the number of lines that the point is NORTH of.
	for (unsigned int i = 0; i < nBPts_; i++)
	{
		// vvvvvvvvvvvvvvvv Ray Casting, count how many crosses south vvvvvvvvvvvvvvvv
		if (ps.E >= lineMinMax_[i][2] && ps.E < lineMinMax_[i][3])
		{
			if (ps.N > line_Mandb_[i][0] * ps.E + line_Mandb_[i][1])
				crossed_lines_ps++;
			else if (ps.N == line_Mandb_[i][0] * ps.E + line_Mandb_[i][1])
        return false;
		}
		if (pe.E >= lineMinMax_[i][2] && pe.E < lineMinMax_[i][3])
		{
			if (pe.N > line_Mandb_[i][0] * pe.E + line_Mandb_[i][1])
				crossed_lines_pe++;
			else if (pe.N == line_Mandb_[i][0] * pe.E + line_Mandb_[i][1])
        return false;
		}
		// ^^^^^^^^^^^^^^^^ Ray Casting, count how many crosses south ^^^^^^^^^^^^^^^^

		//vvvvvvvvvvvvvvvvvvvvvvvvvvvv Check if any point on the line gets too close to the boundary vvvvvvvvvvvvvvvvvvvvvvvvvvvv
		// Check distance between each endpoint
		if (sqrt(pow(ps.N - map_.boundary_pts[i].N, 2) + pow(ps.E - map_.boundary_pts[i].E, 2) < r))
      return false;
		if (sqrt(pow(pe.N - map_.boundary_pts[i].N, 2) + pow(pe.E - map_.boundary_pts[i].E, 2) < r))
      return false;
		// Check if they intersect
		if (line_Mandb_[i][0] != path_Mandb[0])
		{
			Ei = (path_Mandb[1] - line_Mandb_[i][1]) / (line_Mandb_[i][0] - path_Mandb[0]);
			Ni = line_Mandb_[i][0] * Ei + line_Mandb_[i][1];
			if (Ni > pathMinMax[0] && Ni < pathMinMax[1])
				if (Ni > lineMinMax_[i][0] && Ni < lineMinMax_[i][1])
          return false;
		}
		// Check distance from bl to each path end point
		bool lp_cleared;
		double lMinMax[4], l_Mandb[4];
		lMinMax[0] = lineMinMax_[i][0];
		lMinMax[1] = lineMinMax_[i][1];
		lMinMax[2] = lineMinMax_[i][2];
		lMinMax[3] = lineMinMax_[i][3];
		l_Mandb[0] = line_Mandb_[i][0];
		l_Mandb[1] = line_Mandb_[i][1];
		l_Mandb[2] = line_Mandb_[i][2];
		l_Mandb[3] = line_Mandb_[i][3];
		lp_cleared = lineAndPoint2d(map_.boundary_pts[i], map_.boundary_pts[(i + 1) % nBPts_], lMinMax, l_Mandb, ps, r);
		if (lp_cleared == false)
      return false;
		lp_cleared = lineAndPoint2d(map_.boundary_pts[i], map_.boundary_pts[(i + 1) % nBPts_], lMinMax, l_Mandb, pe, r);
		if (lp_cleared == false)
      return false;
		// Check distance from pl to each boundary end point
		lp_cleared = lineAndPoint2d(ps, pe, pathMinMax, path_Mandb, map_.boundary_pts[i], r);
		if (lp_cleared == false)
      return false;
		//^^^^^^^^^^^^^^^^^^^^^^^^^^^^ Check if any point on the line gets too close to the boundary ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	}

	// vvvvvvvvvvvvvvvv Finish up checking if the end points were both inside the boundary (ray casting) vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
	withinBoundaries_ps = crossed_lines_ps % 2; // If it crosses an even number of boundaries it is NOT inside, if it crosses an odd number it IS inside
	withinBoundaries_pe = crossed_lines_pe % 2;
	if (withinBoundaries_ps == false || withinBoundaries_pe == false)
    return false;
	// ^^^^^^^^^^^^^^^^ Finish up checking if the end points were both inside the boundary (ray casting) ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	// vvvvvvvvvvvvvvvvvvvvv Check to see if the point is within the right fly altitudes vvvvvvvvvvvvvvvvvvvvvv
	if (is3D_ && taking_off_ == false)
	{
		if (-ps.D < minFlyHeight_ + r || -ps.D > maxFlyHeight_ - r)
      return false;
		if (-pe.D < minFlyHeight_ + r || -pe.D > maxFlyHeight_ - r)
      return false;
	}
	// vvvvvvvvvvvvvvvvvvvvv Check to see if the point is within the right fly altitudes vvvvvvvvvvvvvvvvvvvvvv

	//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ Check for Boundary Lines ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	//vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
	//vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv Check for Cylinder Obstacles vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
	bool clearThisCylinder;
	NED_s cylinderPoint;
	for (unsigned int i = 0; i < map_.cylinders.size(); i++)
	{
		cylinderPoint.N = map_.cylinders[i].N;
		cylinderPoint.E = map_.cylinders[i].E;
		cylinderPoint.D = -map_.cylinders[i].H;
		clearThisCylinder = lineAndPoint2d(ps, pe, pathMinMax, path_Mandb, cylinderPoint, map_.cylinders[i].R + r);

		// Check in 3D. Note that if the above is true, this check does not need to be performed.
		if (is3D_ && clearThisCylinder == false)
		{
			double dD = (pe.D - ps.D) / sqrt(pow(ps.N - pe.N, 2) + pow(ps.E - pe.E, 2));
			double bt = cylinderPoint.N - path_Mandb[2] * cylinderPoint.E;
			Ei = (bt - path_Mandb[1]) / (path_Mandb[3]);
			Ni = path_Mandb[2] * Ei + bt;
			double bigLength = sqrt(pow(map_.cylinders[i].R + r, 2) - pow(Ni - cylinderPoint.N, 2) - pow(Ei - cylinderPoint.E, 2)); // What is bigLength????
			double d2cyl;
			// Check to see if the path is above the cylinder height or into the cylinder
			if (sqrt(pow(ps.N - cylinderPoint.N, 2) + pow(ps.E - cylinderPoint.E, 2)) < map_.cylinders[i].R + r && sqrt(pow(pe.N - cylinderPoint.N, 2) + pow(pe.E - cylinderPoint.E, 2)) < map_.cylinders[i].R + r)
			{// if BOTH of the endpoints is within the 2d cylinder
				if (-ps.D < map_.cylinders[i].H + r)
          return false;
				if (-ps.D < map_.cylinders[i].H + r)
          return false;
			}
			else
			{// if at least one waypoint is outside of the 2d cylinder
				if (sqrt(pow(ps.N - cylinderPoint.N, 2) + pow(ps.E - cylinderPoint.E, 2)) < map_.cylinders[i].R + r)
				{// if the starting point is within the 2d cylinder
					if (-ps.D < map_.cylinders[i].H + r)
            return false;
					// else (check to see if the line that intersects the cylinder is in or out)
					double smallLength = sqrt(pow(Ni - ps.N, 2) + pow(Ei - ps.E, 2));
					if (Ni > pathMinMax[0] && Ni < pathMinMax[1] && Ei > pathMinMax[2] && Ei < pathMinMax[3])
						d2cyl = bigLength + smallLength;
					else
						d2cyl = bigLength - smallLength;
					if (-(dD*d2cyl + ps.D) < map_.cylinders[i].H + r)
            return false;
				}
				else if (sqrt(pow(pe.N - cylinderPoint.N, 2) + pow(pe.E - cylinderPoint.E, 2)) < map_.cylinders[i].R + r)
				{// if the ending point is within the 2d cylinder
					if (-pe.D < map_.cylinders[i].H + r)
            return false;
					// else check to see if the line that intersects the cylinder is in or out
					double smallLength = sqrt(pow(Ni - pe.N, 2) + pow(Ei - pe.E, 2));
					if (Ni > pathMinMax[0] && Ni < pathMinMax[1] && Ei > pathMinMax[2] && Ei < pathMinMax[3])
						d2cyl = bigLength + smallLength;
					else
						d2cyl = bigLength - smallLength;
					if (-(-dD*d2cyl + pe.D) < map_.cylinders[i].H + r)
            return false;
				}
				// Now check the two intersection points
				else
				{
					// Calculate the intersection point of the line and the perpendicular line connecting the point
					double d_from_cyl2inter = sqrt(pow(cylinderPoint.N - Ni, 2) + pow(cylinderPoint.E - Ei, 2));
					double daway_from_int = sqrt(pow(r + map_.cylinders[i].R, 2) - pow(d_from_cyl2inter, 2)); // WHAT IS THIS?

					// Now test the height at int +- daway_from_int;
					double land_D_ps2i = sqrt(pow(Ni - ps.N, 2) + pow(Ei - ps.E, 2));
					double deltaD = dD*sqrt(pow(Ni - ps.N, 2) + pow(Ei - ps.E, 2));

					double Di = ps.D + dD*sqrt(pow(Ni - ps.N, 2) + pow(Ei - ps.E, 2));

					double height1 = -(Di + dD*daway_from_int);
					double height2 = -(Di - dD*daway_from_int);

					if (-(Di + dD*daway_from_int) < map_.cylinders[i].H + r)
            return false;
					if (-(Di - dD*daway_from_int) < map_.cylinders[i].H + r)
            return false;
					if (-Di < map_.cylinders[i].H + r)
            return false;
				}
			}
			clearThisCylinder = true;
		}
		if (clearThisCylinder == false)
      return false;
	}
	//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ Check for Cylinder Obstacles ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  return true; // The line is in the safe zone if it got to here!
}
//****************************************************************LINE*************************************************************************************
//****************************************************************LINE*************************************************************************************
//****************************************************************LINE*************************************************************************************

bool RRT::flyZoneCheck(const NED_s NED, const double radius) // Point start, radius
{
	// This is a more simple version of the flyZoneCheck() that just checks if the point NED is at least radius away from any obstacle.
	// First, Check Within the Boundaries
	bool withinBoundaries;
	// Look at the Point in Polygon Algorithm
	// Focus on rays South.
	int crossed_lines = 0;							// This is a counter of the number of lines that the point is NORTH of.
	double bt, Ei, Ni, de1, de2, shortest_distance;
	for (unsigned int i = 0; i < nBPts_; i++)
	{
		// Find out if the line is either North or South of the line
		if (NED.E >= lineMinMax_[i][2] && NED.E < lineMinMax_[i][3]) // Only one equal sign solves both the above/ below a vertice problem and the vertical line problem
		{
			if (NED.N > line_Mandb_[i][0] * NED.E + line_Mandb_[i][1])
				crossed_lines++;
			else if (NED.N == line_Mandb_[i][0] * NED.E + line_Mandb_[i][1])	// On the rare chance that the point is ON the line
				return false;
		}
		// Check to see if it is too close to the boundary lines
		if (NED.E >= lineMinMax_[i][2] - radius && NED.E < lineMinMax_[i][3] + radius && NED.N >= lineMinMax_[i][0] - radius && NED.N < lineMinMax_[i][1] + radius)
		{
			bt = NED.N - line_Mandb_[i][2] * NED.E;
			Ei = (bt - line_Mandb_[i][1]) / line_Mandb_[i][3];
			Ni = line_Mandb_[i][2] * Ei + bt;
			// 3 cases first point, second point, or on the line.
			// If the intersection is on the line, dl is the shortest distance
			// Otherwise it is one of the endpoints.
			if (Ni > lineMinMax_[i][0] && Ni < lineMinMax_[i][1] && Ei > lineMinMax_[i][2] && Ei < lineMinMax_[i][3])
				shortest_distance = sqrt(pow(Ni - NED.N, 2) + pow(Ei - NED.E, 2));
			else
			{
				de1 = sqrt(pow(map_.boundary_pts[i].N - NED.N, 2) + pow(map_.boundary_pts[i].E - NED.E, 2));
				de2 = sqrt(pow(map_.boundary_pts[(i + 1) % nBPts_].N - NED.N, 2) + pow(map_.boundary_pts[(i + 1) % nBPts_].E - NED.E, 2));
				shortest_distance = std::min(de1, de2);
			}
			if (shortest_distance < radius)
				return false;
		}
	}
	withinBoundaries = crossed_lines % 2; // If it crosses an even number of boundaries it is NOT inside, if it crosses an odd number it IS inside
	if (withinBoundaries == false)
		return false;
	// Check to see if the point is within the right fly altitudes
	if (is3D_ && taking_off_ == false)
		if (-NED.D < minFlyHeight_ + radius || -NED.D > maxFlyHeight_ - radius)
			return false;

	// Second, Check for Cylinders
	// Check if the point falls into the volume of the cylinder
	for (unsigned int i = 0; i < map_.cylinders.size(); i++)
		if (sqrt(pow(NED.N - map_.cylinders[i].N, 2) + pow(NED.E - map_.cylinders[i].E, 2)) < map_.cylinders[i].R + radius && -NED.D - radius < map_.cylinders[i].H)
			return false;
	return true; // The coordinate is in the safe zone if it got to here!
}

//****************************************************************ARC**************************************************************************************
//****************************************************************ARC**************************************************************************************
//****************************************************************ARC**************************************************************************************
bool RRT::flyZoneCheck(const NED_s ps, const NED_s pe, const double aradius, const NED_s cp, const double r,const bool ccw) // Point start, point end, arc radius, center point, clearance
{
	// THIS FUNCTION IS REALLY IMPORTANT. IT DETERMINES IF AN ARC CONNECTING ps AND pe INTERSECT ANY OBSTACLE OR GET WITHIN r OF ANY OBSTACLE.
	// Preliminary Calculations about the line connecting ps and pe
	double Ei, Ni;
	//vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
	//vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv Check for Boundary Lines vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
	bool withinBoundaries_ps, withinBoundaries_pe;
	int crossed_lines_ps(0), crossed_lines_pe(0);	// This is a counter of the number of lines that the point is NORTH of.
	for (unsigned int i = 0; i < nBPts_; i++)
	{
		// vvvvvvvvvvvvvvvv Ray Casting, count how many crosses south vvvvvvvvvvvvvvvv
		if (ps.E >= lineMinMax_[i][2] && ps.E < lineMinMax_[i][3])
		{
			if (ps.N > line_Mandb_[i][0] * ps.E + line_Mandb_[i][1])
				crossed_lines_ps++;
			else if (ps.N == line_Mandb_[i][0] * ps.E + line_Mandb_[i][1])
				return false;
		}
		if (pe.E >= lineMinMax_[i][2] && pe.E < lineMinMax_[i][3])
		{
			if (pe.N > line_Mandb_[i][0] * pe.E + line_Mandb_[i][1])
				crossed_lines_pe++;
			else if (pe.N == line_Mandb_[i][0] * pe.E + line_Mandb_[i][1])
				return false;
		}
		// ^^^^^^^^^^^^^^^^ Ray Casting, count how many crosses south ^^^^^^^^^^^^^^^^

		//vvvvvvvvvvvvvvvvvvvvvvvvvvvv Check if any point on the line gets too close to the boundary vvvvvvvvvvvvvvvvvvvvvvvvvvvv
		if (cp.E >= lineMinMax_[i][2] - r - aradius && cp.E <= lineMinMax_[i][3] + r + aradius && cp.N >= lineMinMax_[i][0] - r - aradius && cp.N <= lineMinMax_[i][1] + r + aradius)
		{
			double bt;
			// Calculate the intersection point of the line and the perpendicular line connecting the point
			bt = cp.N - line_Mandb_[i][2] * cp.E;
			Ei = (bt - line_Mandb_[i][1]) / (line_Mandb_[i][3]);
			Ni = line_Mandb_[i][2] * Ei + bt;
			if (Ni > lineMinMax_[i][0] && Ni < lineMinMax_[i][1] && Ei > lineMinMax_[i][2] && Ei < lineMinMax_[i][3])
			{
				// a dot b = A*B*cos(theta)
				if (lineIntersectsArc(Ni, Ei, cp, ps, pe, ccw))
				{
					if (sqrt(pow(Ni - cp.N, 2) + pow(Ei - cp.E, 2)) - aradius < r)
					{
						return false;
					}
				}
				else
				{
					bt = ps.N - line_Mandb_[i][2] * ps.E;
					Ei = (bt - line_Mandb_[i][1]) / (line_Mandb_[i][3]);
					Ni = line_Mandb_[i][2] * Ei + bt;
					if (Ni > lineMinMax_[i][0] && Ni < lineMinMax_[i][1] && Ei > lineMinMax_[i][2] && Ei < lineMinMax_[i][3])
					{
						if (sqrt(pow(Ni - ps.N, 2) + pow(Ei - ps.E, 2)) < r)
						{
							return false;
						}
					}
					else if (sqrt(pow(map_.boundary_pts[i].N - ps.N, 2) + pow(map_.boundary_pts[i].E - ps.E, 2)) < r)
					{
						return false;
					}
					else if (sqrt(pow(map_.boundary_pts[(i + 1) % nBPts_].N - ps.N, 2) + pow(map_.boundary_pts[(i + 1) % nBPts_].E - ps.E, 2)) < r) { return false; }
					bt = pe.N - line_Mandb_[i][2] * pe.E;
					Ei = (bt - line_Mandb_[i][1]) / (line_Mandb_[i][3]);
					Ni = line_Mandb_[i][2] * Ei + bt;
					if (Ni > lineMinMax_[i][0] && Ni < lineMinMax_[i][1] && Ei > lineMinMax_[i][2] && Ei < lineMinMax_[i][3])
					{
						if (sqrt(pow(Ni - pe.N, 2) + pow(Ei - pe.E, 2)) < r)
						{
							return false;
						}
					}
					else if (sqrt(pow(map_.boundary_pts[i].N - pe.N, 2) + pow(map_.boundary_pts[i].E - pe.E, 2)) < r)
					{
						return false;
					}
					//else if (sqrt(pow(map_.boundary_pts[(i + 1) % nBPts_].N - pe.N, 2) + pow(map_.boundary_pts[(i + 1) % nBPts_].E - pe.E, 2)) < r) { return false; }
				}
			}
			else
			{
				if (lineIntersectsArc(map_.boundary_pts[i].N, map_.boundary_pts[i].E, cp, ps, pe, ccw))
				{
					if (sqrt(pow(map_.boundary_pts[i].N - cp.N, 2) + pow(map_.boundary_pts[i].E - cp.E, 2)) - aradius < r)
					{
						return false;
					}
				}
				//if (lineIntersectsArc(map_.boundary_pts[(i + 1) % nBPts_].N, map_.boundary_pts[(i + 1) % nBPts_].E, cp, ps, pe, ccw))
				//{
				//	if (sqrt(pow(map_.boundary_pts[(i + 1) % nBPts_].N - cp.N, 2) + pow(map_.boundary_pts[(i + 1) % nBPts_].E - cp.E, 2)) - aradius < r) { return false; }
				//}
				if (sqrt(pow(map_.boundary_pts[i].N - ps.N, 2) + pow(map_.boundary_pts[i].E - ps.E, 2)) - aradius < r)
				{
					return false;
				}
				if (sqrt(pow(map_.boundary_pts[i].N - pe.N, 2) + pow(map_.boundary_pts[i].E - pe.E, 2)) - aradius < r)
				{
					return false;
				}
				//if (sqrt(pow(map_.boundary_pts[(i + 1) % nBPts_].N - ps.N, 2) + pow(map_.boundary_pts[(i + 1) % nBPts_].E - ps.E, 2)) - aradius < r) { return false; }
				//if (sqrt(pow(map_.boundary_pts[(i + 1) % nBPts_].N - pe.N, 2) + pow(map_.boundary_pts[(i + 1) % nBPts_].E - pe.E, 2)) - aradius < r) { return false; }
			}
		}
		//^^^^^^^^^^^^^^^^^^^^^^^^^^^^ Check if any point on the line gets too close to the boundary ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	}
	// vvvvvvvvvvvvvvvv Finish up checking if the end points were both inside the boundary (ray casting) vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
	withinBoundaries_ps = crossed_lines_ps % 2; // If it crosses an even number of boundaries it is NOT inside, if it crosses an odd number it IS inside
	withinBoundaries_pe = crossed_lines_pe % 2;
	if (withinBoundaries_ps == false || withinBoundaries_pe == false)
		return false;
	// ^^^^^^^^^^^^^^^^ Finish up checking if the end points were both inside the boundary (ray casting) ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	// vvvvvvvvvvvvvvvvvvvvv Check to see if the point is within the right fly altitudes vvvvvvvvvvvvvvvvvvvvvv
	if (is3D_ && taking_off_ == false)
	{
		if (-ps.D < minFlyHeight_ + r || -ps.D > maxFlyHeight_ - r)
			return false;
		if (-pe.D < minFlyHeight_ + r || -pe.D > maxFlyHeight_ - r)
			return false;
	}
	// vvvvvvvvvvvvvvvvvvvvv Check to see if the point is within the right fly altitudes vvvvvvvvvvvvvvvvvvvvvv

	//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ Check for Boundary Lines ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	//vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
	//vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv Check for Cylinder Obstacles vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
	bool clearThisCylinder;
	for (unsigned int i = 0; i < map_.cylinders.size(); i++)
	{
		if (sqrt(pow(map_.cylinders[i].N - cp.N, 2) + pow(map_.cylinders[i].E - cp.E, 2)) > r + aradius + map_.cylinders[i].R)
			clearThisCylinder = true;
		else if (lineIntersectsArc(map_.cylinders[i].N, map_.cylinders[i].E, cp, ps, pe, ccw))
		{
			if (sqrt(pow(map_.cylinders[i].N - cp.N, 2) + pow(map_.cylinders[i].E - cp.E, 2)) - aradius - map_.cylinders[i].R < r)
			{
				clearThisCylinder = false;
			}
		}
		else
		{
			if (sqrt(pow(map_.cylinders[i].N - ps.N, 2) + pow(map_.cylinders[i].E - ps.E, 2)) - map_.cylinders[i].R < r)
			{
				clearThisCylinder = false;
			}
			else if (sqrt(pow(map_.cylinders[i].N - pe.N, 2) + pow(map_.cylinders[i].E - pe.E, 2)) - map_.cylinders[i].R < r)
			{
				clearThisCylinder = false;
			}
			else { clearThisCylinder = true; }
		}
		if (is3D_ && clearThisCylinder == false)
		{
			if (ps.D < -map_.cylinders[i].H - r && pe.D < -map_.cylinders[i].H - r)
				clearThisCylinder = true;
		}
		if (clearThisCylinder == false)
			return false;
	}
	//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ Check for Cylinder Obstacles ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	return true; // The line is in the safe zone if it got to here!
}
//****************************************************************ARC**************************************************************************************
//****************************************************************ARC**************************************************************************************
//****************************************************************ARC**************************************************************************************
bool RRT::lineIntersectsArc(double Ni, double Ei, NED_s cp, NED_s ps, NED_s pe, bool ccw)
{
	// Find angle from cp to ps
	double aC2s = atan2(ps.N - cp.N, ps.E - cp.E);
	// Find angle from cp to pe
	double aC2e = atan2(pe.N - cp.N, pe.E - cp.E);
	// Find angle from cp to Ni, Ei
	double aC2i = atan2(Ni - cp.N, Ei - cp.E);
	// Do they overlap?
	if (ccw)
	{
		if (aC2i >= aC2s && aC2i <= aC2e)
			return true;
		else if (aC2s > aC2e)
		{
			if ((aC2i >= aC2s || aC2i <= aC2e))
				return true;
		}
	}
	else
	{
		if (aC2i <= aC2s && aC2i >= aC2e)
			return true;
		else if (aC2e > aC2s)
		{
			if (aC2i <= aC2s || aC2i >= aC2e)
				return true;
		}
	}
	return false;
}
bool RRT::lineAndPoint2d(NED_s ls, NED_s le, double MinMax[], double Mandb[], NED_s p, double r)
{
	// This function is used a lot by the LINE flyZoneCheck().
	// It checks to see if the point p is at least r away from the line segment connecting ls and le. 2 Dimensional Projection
	// This function is pretty important to have right.
	double bt, Ei, Ni, shortest_distance, de1, de2;
	// If the point is close enough to even consider calculating - if it is far away, it is clear.
	if (p.E >= MinMax[2] - r && p.E <= MinMax[3] + r && p.N >= MinMax[0] - r && p.N <= MinMax[1] + r)
	{
		// Calculate the intersection point of the line and the perpendicular line connecting the point
		bt = p.N - Mandb[2] * p.E;
		Ei = (bt - Mandb[1]) / (Mandb[3]);
		Ni = Mandb[2] * Ei + bt;

		// Find the distance between the point and the line segment
		// 3 cases. Closest point will be the first point(line beginning), second point (line ending), or on the line.
		// If the intersection is on the line, dl is the shortest distance
		// Otherwise it is one of the endpoints.
		if (Ni > MinMax[0] && Ni < MinMax[1] && Ei > MinMax[2] && Ei < MinMax[3])
			shortest_distance = sqrt(pow(Ni - p.N, 2) + pow(Ei - p.E, 2));
		else
		{
			de1 = sqrt(pow(ls.N - p.N, 2) + pow(ls.E - p.E, 2));
			de2 = sqrt(pow(le.N - p.N, 2) + pow(le.E - p.E, 2));
			shortest_distance = std::min(de1, de2);
		}
		if (shortest_distance < r)
			return false;
	}
	return true;	// It is at least r away from the line if it got to here.
}
void RRT::computePerformance()
{
	total_nWPS_ = 0;
	for (unsigned int i = 0; i < all_wps_.size(); i++)
		total_nWPS_ += all_wps_[i].size();
	total_path_length_ = 0;
	for (unsigned int i = 0; i < path_distances_.size(); i++)
		total_path_length_ += path_distances_[i];
}
} // end namespace theseus
