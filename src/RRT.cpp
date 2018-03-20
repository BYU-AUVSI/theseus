#include <theseus/RRT.h>

namespace theseus
{
RRT::RRT(map_s map_in, unsigned int seed, ParamReader *input_file_in, RRT_input alg_input_in) // Setup the object
{

  ROS_INFO("RRT Constructor");
	input_file = input_file_in;
	alg_input = alg_input_in;
	D = alg_input.D;										  // Distance between each RRT waypoint
	map = map_in;											    // Get a copy of the terrain map
	RandGen rg_in(seed);								  // Make a random generator object that is seeded
	rg = rg_in;												    // Copy that random generator into the class.
	ppSetup();												    // default stuff for every algorithm that needs to be called after it recieves the map.
}
RRT::~RRT()
{
  // These lines free the memory in the vectors... We were having problems with memory in the mapper class
  // These lines fixed it there so they were inserted here as well.
  for (unsigned int i = 0; i < lineMinMax.size(); i++)
    std::vector<double>().swap(lineMinMax[i]);
  std::vector<std::vector<double> >().swap(lineMinMax);
  for (unsigned int i = 0; i < line_Mandb.size(); i++)
    std::vector<double>().swap(line_Mandb[i]);
  std::vector<std::vector<double> >().swap(line_Mandb);
  std::vector<double>().swap(path_distances);
	delete_tree();											  // Delete all of those tree pointer nodes
	std::vector<node*>().swap(root_ptrs);	// Free the memory of the vector.
}
bool RRT::solve_static(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res)							// This function solves for a path in between the waypoinnts (2 Dimensional)
{
	initialize_tree();
	NED_s second2last_post_smoothed;
	second2last_post_smoothed.N = root_ptrs[0]->NED.N - cos(input_file->chi0);
	second2last_post_smoothed.E = root_ptrs[0]->NED.E - sin(input_file->chi0);
	second2last_post_smoothed.D = root_ptrs[0]->NED.D;
	for (unsigned int i = 0; i < map.wps.size(); i++)
	{
		ROS_INFO("MAIN LOOP: %i",i);
		path_clearance = input_file->clearance;
		node *second2last = root_ptrs[i];					// This will be set as the second to last waypoint
		double distance_in, fillet_angle;
		bool direct_shot = false;
		bool direct_connect = direct_connection(i, &second2last_post_smoothed, &distance_in, &fillet_angle, &direct_shot);		//******* IMPORTANT
		if (direct_connect)
			second2last = closest_node;
		develop_tree(i, direct_connect,second2last,&second2last_post_smoothed,&distance_in,&fillet_angle);						//******* IMPORTANT
		smoother(direct_connect, i, &distance_in, &fillet_angle, &second2last_post_smoothed, direct_shot);						//******* IMPORTANT
		calc_path_distance(i);
	}
	all_wps[map.wps.size() - 1].push_back(map.wps[map.wps.size() - 1]);	// Add the final waypoint to the waypoint list.
	compute_performance();
  sendROSmessage();
  res.success = true;
  return true;
}
void RRT::sendROSmessage()
{
  // Package up a message to send
  AuvsiMap map_msg;
  AuvsiBoundaries bounds_msg;
  AuvsiStaticObstacle obs_msg;
  rosplane_msgs::Waypoint wp_msg;

  // waypoints
  for (unsigned int i = 0; i < all_wps.size(); i++)
	{
    for (unsigned int j = 0; j < all_wps[i].size(); j++)
		{
      wp_msg.w[0] = all_wps[i][j].N;
      wp_msg.w[1] = all_wps[i][j].E;
      wp_msg.w[2] = all_wps[i][j].D;
      map_msg.wps.push_back(wp_msg);
    }
  }
  for (unsigned int i = 0; i < map.wps.size(); i++)
	{
    wp_msg.w[0] = map.wps[i].N;
    wp_msg.w[1] = map.wps[i].E;
    wp_msg.w[2] = map.wps[i].D;
    map_msg.primary_wps.push_back(wp_msg);
  }
  // Boundaries
  for (unsigned int i = 0; i < map.boundary_pts.size(); i++)
	{
    bounds_msg.north = map.boundary_pts[i].N;
    bounds_msg.east  = map.boundary_pts[i].E;
    map_msg.bds.push_back(bounds_msg);
  }
  // Static Obstacles
  for (unsigned int i = 0; i < map.cylinders.size(); i++)
  {
    obs_msg.north  = map.cylinders[i].N;
    obs_msg.east   = map.cylinders[i].E;
    obs_msg.radius = map.cylinders[i].R;
    obs_msg.height = map.cylinders[i].H;
    map_msg.obs.push_back(obs_msg);
  }
  mission_map_publisher_.publish(map_msg);
  ROS_INFO("SENT MESSAGE");
}
bool RRT::check_direct_fan(NED_s second_wp, NED_s primary_wp, NED_s coming_from, node* next_root, NED_s* cea_out, double* din, double* anglin)
{
	bool found_at_least_1_good_path = false;
	double R = sqrt(pow(second_wp.N - primary_wp.N,2) + pow(second_wp.E - primary_wp.E,2) + pow(second_wp.D - primary_wp.D,2));

	double approach_angle = atan2(primary_wp.N - coming_from.N, primary_wp.E - coming_from.E) + 3.141592653;
	double approach_angleDEGREES = approach_angle*180.0 / 3.141592653;
	double beta, lambda, Q, phi, theta, zeta, gamma, d;
	NED_s cpa, cea, lea, fake_wp;

	// What should alpha be?
	double leave_angle = atan2(second_wp.N - primary_wp.N, second_wp.E - primary_wp.E);
	double leave_angleDEGREES = leave_angle*180.0 / 3.141592653;
	double alpha = atan2(second_wp.N - primary_wp.N, second_wp.E - primary_wp.E) - approach_angle;
	//double alphaDEGREES = alpha*180.0 / 3.141592653;
	while (alpha < -3.141592653)
		alpha = alpha + 2 * 3.141592653;
	while (alpha > 3.141592653)
		alpha = alpha - 2 * 3.141592653;

	bool positive_angle = true;
	if (alpha < 0.0)
	{
		positive_angle = false;
		alpha = -1.0*alpha;
	}
	if (2.0*input_file->turn_radius / R > 1.0 || 2.0*input_file->turn_radius / R < -1.0)
		return false;
	double minAngle = asin(2.0*input_file->turn_radius / R) + 8*3.141592653/180.0;
	if (alpha < minAngle || (alpha > 3.141592653- minAngle && alpha < 3.141592653 + minAngle))
		return false;

	beta = 3.141592653 / 2 - alpha;
	lambda = 3.141592653 - 2 * beta;
	Q = sqrt(R*(R - input_file->turn_radius*sin(lambda) / sin(beta)) + input_file->turn_radius*input_file->turn_radius);
	phi = 3.141592653 - asin(R*sin(beta) / Q);
	theta = acos(input_file->turn_radius / Q);
	zeta = (2 * 3.141592653 - phi - theta) / 2.0;
	gamma = 3.141592653 - 2 * zeta;
	d = input_file->turn_radius / tan(gamma / 2.0);

	fake_wp.N = primary_wp.N - d*sin(approach_angle);
	fake_wp.E = primary_wp.E - d*cos(approach_angle);
	fake_wp.D = primary_wp.D;

	// What should it be on the positive or on the negative side?
	if (positive_angle)
	{
		// Check the positive side
		cpa.N = primary_wp.N + input_file->turn_radius*cos(approach_angle);
		cpa.E = primary_wp.E - input_file->turn_radius*sin(approach_angle);
		cpa.D = primary_wp.D;

		cea.N = fake_wp.N + d*sin(gamma + approach_angle);
		cea.E = fake_wp.E + d*cos(gamma + approach_angle);
		cea.D = primary_wp.D;

		lea.N = primary_wp.N + R*sin(approach_angle + alpha);
		lea.E = primary_wp.E + R*cos(approach_angle + alpha);
		lea.D = primary_wp.D;

		if (flyZoneCheck(primary_wp, cea, input_file->turn_radius, cpa, clearance, false))
			if (flyZoneCheck(cea, lea, clearance))
			{
				// Looks like things are going to work out for this maneuver!
				found_at_least_1_good_path = true;
				node *fake_child = new node;
				//node *normal_gchild = new node;
				fake_child->NED = fake_wp;
				fake_child->available_dist = 0;
				fake_child->parent = next_root;
				fake_child->distance = 2.0*zeta*input_file->turn_radius;
				fake_child->path_type = 1;
				next_root->children.push_back(fake_child);
			}
	}
	else // Check the negative side
	{
		cpa.N = primary_wp.N - input_file->turn_radius*cos(approach_angle);
		cpa.E = primary_wp.E + input_file->turn_radius*sin(approach_angle);
		cpa.D = primary_wp.D;

		cea.N = fake_wp.N + d*sin(-gamma + approach_angle);
		cea.E = fake_wp.E + d*cos(-gamma + approach_angle);
		cea.D = primary_wp.D;

		lea.N = primary_wp.N + R*sin(approach_angle - alpha);
		lea.E = primary_wp.E + R*cos(approach_angle - alpha);
		lea.D = primary_wp.D;

		if (flyZoneCheck(primary_wp, cea, input_file->turn_radius, cpa, clearance, false))
			if (flyZoneCheck(cea, lea, clearance))
			{
				// Looks like things are going to work out for this maneuver!
				found_at_least_1_good_path = true;
				node *fake_child = new node;
				node *normal_gchild = new node;
				fake_child->NED = fake_wp;
				fake_child->available_dist = 0;
				fake_child->parent = next_root;
				fake_child->distance = 2.0*zeta*input_file->turn_radius;
				fake_child->path_type = 1;
				next_root->children.push_back(fake_child);
			}
	}
	*cea_out = cea;
	*din = sqrt(pow(fake_wp.N - cea.N, 2) + pow(fake_wp.E - cea.E, 2) + pow(fake_wp.D - cea.D, 2));
	*anglin = 2.0*zeta;
	return found_at_least_1_good_path;
}
bool RRT::check_create_fan(NED_s primary_wp, NED_s coming_from, node* next_root)
{
	bool found_at_least_1_good_path = false;
	// Make sure that it is possible to go to the next waypoint

	double alpha = 3.141592653 / 4.0;			// Start with 45.0 degrees
	double R = 3.0 * input_file->turn_radius;
	int num_circle_trials = 10;				// Will try num_circle_trials on one side and num_circle_trials on the other side.
	double dalpha = (3.141592653 - alpha) / num_circle_trials;

	double approach_angle = atan2(primary_wp.N - coming_from.N, primary_wp.E - coming_from.E) + 3.141592653;
	double beta, lambda, Q, phi, theta, zeta, gamma, d;
	NED_s cpa, cea, lea, fake_wp;
	for (int j = 0; j < num_circle_trials; j++)
	{
		alpha = alpha + dalpha;
		beta = 3.141592653 / 2 - alpha;
		lambda = 3.141592653 - 2 * beta;
		Q = sqrt(R*(R - input_file->turn_radius*sin(lambda) / sin(beta)) + input_file->turn_radius*input_file->turn_radius);
		phi = 3.141592653 - asin(R*sin(beta) / Q);
		theta = acos(input_file->turn_radius / Q);
		zeta = (2 * 3.141592653 - phi - theta) / 2.0;
		gamma = 3.141592653 - 2 * zeta;
		d = input_file->turn_radius / tan(gamma / 2.0);

		// Check the positive side
		fake_wp.N = primary_wp.N - d*sin(approach_angle);
		fake_wp.E = primary_wp.E - d*cos(approach_angle);
		fake_wp.D = primary_wp.D;

		cpa.N = primary_wp.N + input_file->turn_radius*cos(approach_angle);
		cpa.E = primary_wp.E - input_file->turn_radius*sin(approach_angle);
		cpa.D = primary_wp.D;

		cea.N = fake_wp.N + d*sin(gamma + approach_angle);
		cea.E = fake_wp.E + d*cos(gamma + approach_angle);
		cea.D = primary_wp.D;

		lea.N = primary_wp.N + R*sin(approach_angle + alpha);
		lea.E = primary_wp.E + R*cos(approach_angle + alpha);
		lea.D = primary_wp.D;

		if (flyZoneCheck(primary_wp, cea, input_file->turn_radius, cpa, clearance, false))
			if (flyZoneCheck(cea, lea, clearance))
			{
				// Looks like things are going to work out for this maneuver!
				found_at_least_1_good_path = true;
				node *fake_child = new node;
				node *normal_gchild = new node;
				fake_child->NED = fake_wp;
				fake_child->available_dist = 0;
				fake_child->parent = next_root;
				fake_child->distance = 2.0*zeta*input_file->turn_radius;
				fake_child->path_type = 1;
				fake_child->line_start = next_root->NED;
				next_root->children.push_back(fake_child);

				normal_gchild->NED = lea;
				normal_gchild->available_dist = sqrt(R*(R - input_file->turn_radius*sin(lambda) / sin(beta)));
				normal_gchild->parent = fake_child;
				normal_gchild->distance = normal_gchild->available_dist;
				normal_gchild->path_type = 1;
				normal_gchild->line_start = cea;
				fake_child->children.push_back(normal_gchild);
			}
		// Check the negative side
		cpa.N = primary_wp.N - input_file->turn_radius*cos(approach_angle);
		cpa.E = primary_wp.E + input_file->turn_radius*sin(approach_angle);
		cpa.D = primary_wp.D;

		cea.N = fake_wp.N + d*sin(-gamma + approach_angle);
		cea.E = fake_wp.E + d*cos(-gamma + approach_angle);
		cea.D = primary_wp.D;

		lea.N = primary_wp.N + R*sin(approach_angle - alpha);
		lea.E = primary_wp.E + R*cos(approach_angle - alpha);
		lea.D = primary_wp.D;

		if (flyZoneCheck(primary_wp, cea, input_file->turn_radius, cpa, clearance, false))
			if (flyZoneCheck(cea, lea, clearance))
			{
				// Looks like things are going to work out for this maneuver!
				found_at_least_1_good_path = true;
				node *fake_child = new node;
				node *normal_gchild = new node;
				fake_child->NED = fake_wp;
				fake_child->available_dist = 0;
				fake_child->parent = next_root;
				fake_child->distance = 2.0*zeta*input_file->turn_radius;
				fake_child->path_type = 1;
				fake_child->line_start = next_root->NED;
				next_root->children.push_back(fake_child);

				normal_gchild->NED = lea;
				normal_gchild->available_dist = sqrt(R*(R - input_file->turn_radius*sin(lambda) / sin(beta)));
				normal_gchild->parent = fake_child;
				normal_gchild->distance = normal_gchild->available_dist;
				normal_gchild->path_type = 1;
				normal_gchild->line_start = cea;
				fake_child->children.push_back(normal_gchild);
			}
	}
	return found_at_least_1_good_path;
}
bool RRT::check_fillet(NED_s par, NED_s mid, NED_s nex, double avail_dis, double* din, double* cangle, NED_s* line_start)
{
	bool found_feasible_link = true;
	// Calculate the fillet and check to see if it is a good fit.
	// a dot b = ||A|| * ||B|| * cos(theta)
	double a_dot_b = (par.E - mid.E)*(nex.E - mid.E) + (par.N - mid.N)*(nex.N - mid.N) + (par.D - mid.D)*(nex.D - mid.D);
	double A = sqrt(pow(par.E - mid.E, 2) + pow(par.N - mid.N, 2) + pow(par.D - mid.D, 2));
	double B = sqrt(pow(nex.N - mid.N, 2) + pow(nex.E - mid.E, 2) + pow(nex.D - mid.D, 2));
	double Fangle = acos((a_dot_b) / (A*B));
	double turn_radius = input_file->turn_radius;
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
		if (flyZoneCheck(ps, pe, turn_radius, cp, clearance, ccw) == false)
			found_feasible_link = false;
		*line_start = pe;
	}
	*din = distance_in;
	*cangle = 2.0*atan(distance_in / turn_radius);
	return found_feasible_link;
}
void RRT::initialize_tree()
{
	// Set up all of the roots
	node *root_in = new node;							// Starting position of the tree (and the waypoint beginning)
	NED_s starting_point;
	starting_point.N = input_file->N0;
	starting_point.E = input_file->E0;
	starting_point.D = input_file->D0;

	root_in->NED = starting_point;
	root_in->parent = NULL;								// No parent
	root_in->distance = 0.0;							// 0 distance.
	root_in->available_dist = 0.0;				// No available distance, (No parent assumption)
	root_in->path_type = 0;								// straight lines for now at the primary waypoints.
	root_in->line_start = root_in->NED;		// The line start is set to it's own location, for now.
	root_ptrs.push_back(root_in);
	for (unsigned int i = 0; i < map.wps.size() - 1; i++)
	{
		node *root_in = new node;							// Starting position of the tree (and the waypoint beginning)
		root_in->NED = map.wps[i];
		root_in->parent = NULL;								// No parent
		root_in->distance = 0.0;							// 0 distance.
		root_in->available_dist = 0.0;			  // No available distance, (No parent assumption)
		root_in->path_type = 0;								// straight lines for now at the primary waypoints.
		root_in->line_start = root_in->NED;		// The line start is set to it's own location, for now.
		root_ptrs.push_back(root_in);
    ROS_INFO("Waypoint %i, North: %f, East %f Down: %f", i, root_in->NED.N, root_in->NED.E, root_in->NED.D);
	}
  unsigned int i = map.wps.size() - 1;
  ROS_INFO("Waypoint %i, North: %f, East %f Down: %f", i, map.wps[i].N, map.wps[i].E, map.wps[i].D);
}
bool RRT::direct_connection(unsigned int i, NED_s* second2last_post_smoothed, double* distance_in, double* fillet_angle, bool* direct_shot)
{
	// Variables created for this chuck of code in a function
	bool reached_next_wp = false;
	NED_s P, line_start;
	node* root = root_ptrs[i];
	double distance;

	// Make sure that the clearance level is right. Mostly this little bit checks to make sure that there is appropriate room to get waypoints close to the floor or ceiling.
	clearance = input_file->clearance;
	clearance = (-map.wps[i].D - input_file->minFlyHeight < clearance) ? -map.wps[i].D - input_file->minFlyHeight : clearance;
	clearance = (map.wps[i].D + input_file->maxFlyHeight < clearance) ? map.wps[i].D + input_file->maxFlyHeight : clearance;
	if (i > 0)
	{
		clearance = (-map.wps[i - 1].D - input_file->minFlyHeight < clearance) ? -map.wps[i - 1].D - input_file->minFlyHeight : clearance;
		clearance = (map.wps[i - 1].D + input_file->maxFlyHeight < clearance) ? map.wps[i - 1].D + input_file->maxFlyHeight : clearance;
	}
	if (clearance <= 0)
		ROS_WARN("ERROR. CLEARANCE IS 0 OR LESS THAN 0.");

	// Check to see if it is possible to go straight to the next path
	ROS_INFO("Checking direct connect.");
	NED_s coming_from;
	if (i == 0)
	{
		coming_from = root->NED;
		reached_next_wp = flyZoneCheck(root->NED, map.wps[i], clearance);
		closest_node = root;
		if (reached_next_wp)
			reached_next_wp = check_slope(root->line_start, map.wps[i]);
		line_start = root->NED;
	}
	else
	{
		P = map.wps[i];
		distance = 9999999999999999999999.0; // Some crazy big number to start out with, maybe look into HUGE_VAL or Inf - worried about embedded implementation.
		double distance_gchild;
		bool found_clean_path = false;
		// Check to see if you can make one turn radius and then go to the next waypoint.
		NED_s cea;
		if (i <= map.wps.size() - 1 && check_direct_fan(map.wps[i], root->NED, *second2last_post_smoothed, root, &cea, distance_in, fillet_angle))
		{
			line_start = cea;
			if (check_slope(line_start, map.wps[i]))
			{
				closest_node = root->children[root->children.size() - 1];
				found_clean_path = true;
				*direct_shot = true;
				coming_from = closest_node->NED;
				reached_next_wp = true;
				closest_node->available_dist = 0.0;
				distance = sqrt(pow(root->NED.N - closest_node->NED.N, 2) + pow(root->NED.E - closest_node->NED.E, 2) + pow(root->NED.D - closest_node->NED.D, 2));
				closest_node->distance = distance;
			}
		}
		else
		{
			for (unsigned int j = 0; j < root->children.size(); j++)
				for (unsigned int k = 0; k < root->children[j]->children.size(); k++)
				{
					distance_gchild = sqrt(pow(P.N - root->children[j]->children[k]->NED.N, 2) + pow(P.E - root->children[j]->children[k]->NED.E, 2) + pow(P.D - root->children[j]->children[k]->NED.D, 2));
					closest_node_gchild = find_closest_node(root->children[j]->children[k], P, root->children[j]->children[k], &distance_gchild);
					if (distance_gchild < distance)
					{
						closest_node = closest_node_gchild;
						distance = distance_gchild;
					}
				}
			coming_from = closest_node->NED;
			reached_next_wp = flyZoneCheck(coming_from, map.wps[i], clearance);
			if (reached_next_wp && check_fillet(closest_node->parent->NED, closest_node->NED, map.wps[i], closest_node->available_dist, distance_in, fillet_angle, &line_start))
				reached_next_wp = check_slope(line_start, map.wps[i]);
		}
	}
	if (reached_next_wp == true && i < map.wps.size() - 1) // This if statement handles setting waypoints to fly out of the primary waypoints.
		if (check_create_fan(map.wps[i], coming_from, root_ptrs[i + 1]) == false)
			reached_next_wp = false;
	if (reached_next_wp && i + 1 < root_ptrs.size())
		root_ptrs[i + 1]->line_start = line_start;
	return reached_next_wp;
}
void RRT::develop_tree(unsigned int i, bool reached_next_wp, node* second2last, NED_s* second2last_post_smoothed, double* distance_in, double* fillet_angle)
{
	// Variables needed to create for this function
	double distance;
	NED_s P, line_start;
	node* root = root_ptrs[i];
	double clearanceP = clearance;
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
			P.N = rg.randLin()*(maxNorth - minNorth) + minNorth;
			P.E = rg.randLin()*(maxEast - minEast) + minEast;
			P.D = map.wps[i].D;
			while (flyZoneCheck(P, clearanceP) == false)
			{
				P.N = rg.randLin()*(maxNorth - minNorth) + minNorth;
				P.E = rg.randLin()*(maxEast - minEast) + minEast;
				P.D = map.wps[i].D;
			}
			p_count++;
			if (p_count == floor(iters_limit / 2.0) || p_count == floor(iters_limit*3.0 / 4.0) || p_count == floor(iters_limit*7.0 / 8.0))
			{
				clearanceP = clearanceP / 6.0;
				ROS_WARN("DECREASING THE CLEARANCE LEVEL");
				clearance = clearance / 2.0;
				ROS_WARN("CLEARANCE: %f",clearance);
				path_clearance = (clearance < path_clearance) ? clearance : path_clearance;
			}
			distance = sqrt(pow(P.N - root->NED.N, 2) + pow(P.E - root->NED.E, 2) + pow(P.D - root->NED.D, 2));

			// Find the closest node to the point P, if you are not trying to get to waypoint 1 (which is the second waypoint), then don't accept the root or the children of the root.
			if (i == 0)
				closest_node = find_closest_node(root, P, root, &distance);
			else
			{
				distance = 9999999999999999999999.0; // Some crazy big number to start out with - maybe look into HUGE_VAL or Inf - worried about embedded implementation.
				double distance_gchild;
				for (unsigned int j = 0; j < root->children.size(); j++)
					for (unsigned int k = 0; k < root->children[j]->children.size(); k++)
					{
						distance_gchild = sqrt(pow(P.N - root->children[j]->children[k]->NED.N, 2) + pow(P.E - root->children[j]->children[k]->NED.E, 2) + pow(P.D - root->children[j]->children[k]->NED.D, 2));
						closest_node_gchild = find_closest_node(root->children[j]->children[k], P, root->children[j]->children[k], &distance_gchild);
						if (distance_gchild < distance)
						{
							closest_node = closest_node_gchild;
							distance = distance_gchild;
						}
					}
			}
			double theta = atan2(P.N - closest_node->NED.N, P.E - closest_node->NED.E);

			// Go a distance D along the line from closest node to P to find the next node position vpos
			if (alg_input.uniform2P)
				D = rg.randLin()*distance;
			else if (alg_input.gaussianD)
				ROS_WARN("gaussianD is depreciated.");
			vpos->NED.N = (closest_node->NED.N) + sin(theta)*D;
			vpos->NED.E = (closest_node->NED.E) + cos(theta)*D;

			if (map.wps[i].D > closest_node->NED.D - tan(input_file->climb_angle)*D && map.wps[i].D < closest_node->NED.D + tan(input_file->descend_angle)*D)
				vpos->NED.D = map.wps[i].D;
			else if (map.wps[i].D > closest_node->NED.D)
				vpos->NED.D = closest_node->NED.D + tan(input_file->descend_angle)*D;
			else
				vpos->NED.D = closest_node->NED.D - tan(input_file->descend_angle)*D;

			// If this path is good move on.
			// Check to see if the straight line path is good.
			if (flyZoneCheck(closest_node->NED, vpos->NED, clearance))
			{
				found_feasible_link = true;
				vpos->parent = closest_node;
				if (alg_input.path_type == 1 && root != vpos->parent)								// If the path type is fillets, check to see if the fillet is possible.
					found_feasible_link = check_fillet(closest_node->parent->NED, closest_node->NED, vpos->NED, closest_node->available_dist, distance_in, fillet_angle, &line_start);
			}
		}
		// add the new node to the tree
		vpos->line_start = line_start;
		closest_node->children.push_back(vpos);
		vpos->distance = sqrt(pow(closest_node->NED.N - vpos->NED.N, 2) + pow(closest_node->NED.E - vpos->NED.E, 2) + pow(closest_node->NED.D - vpos->NED.D, 2));
		if (alg_input.path_type == 1 && vpos->parent != root)
		{
			// Adjust the vpos->distance to account for the turning radius ( a little bit smaller.)
			vpos->available_dist = vpos->distance - *distance_in;
			vpos->distance = vpos->distance - 2.0*(*distance_in) + (*fillet_angle)*input_file->turn_radius;
		}
		else
			vpos->available_dist = vpos->distance;

		// Make provisions so that the algorithm doesn't hang
		added_nodes++;
		if (added_nodes == floor(iters_limit / 2.0) || added_nodes == floor(iters_limit*3.0 / 4.0) || added_nodes == floor(iters_limit*7.0 / 8.0))
		{
			ROS_WARN("DECREASING THE CLEARANCE LEVEL");
			clearance = clearance / (2.0*++clearance_drops);
			ROS_WARN("CLEARANCE: %f",clearance);
			path_clearance = (clearance < path_clearance) ? clearance : path_clearance;
		}
		if (added_nodes >= iters_limit)
		{
			ROS_ERROR("WARNING -- ALGORITHM FAILED TO CONVERGE. RESULTS WILL VIOLATE AN OBSTACLE");
			reached_next_wp = true;
			second2last = vpos;
			line_start = vpos->NED;
		}

		// Check to see if it is possible to go from this newly added node to the next primary waypoint
		if (alg_input.connect_to_end && flyZoneCheck(vpos->NED, map.wps[i], clearance) && check_slope(line_start, map.wps[i]))
		{
			reached_next_wp = true;								// Set the flag
			second2last = vpos;
			if (alg_input.path_type == 1 && root != vpos)								// If the path type is fillets, check to see if the fillet is possible.
				reached_next_wp = check_fillet(vpos->parent->NED, vpos->NED, map.wps[i], vpos->available_dist, distance_in, fillet_angle,&line_start);
		}								// Set the flag
		if (!alg_input.connect_to_end && sqrt(pow(map.wps[i].N - vpos->NED.N, 2) + pow(map.wps[i].E - vpos->NED.E, 2) + pow(map.wps[i].D - vpos->NED.D, 2)) < D && flyZoneCheck(vpos->NED, map.wps[i], clearance) && check_slope(line_start, map.wps[i]))
		{
			reached_next_wp = true;								// Set the flag
			second2last = vpos;
			if (alg_input.path_type == 1 && root != vpos)								// If the path type is fillets, check to see if the fillet is possible.
				reached_next_wp = check_fillet(vpos->parent->NED, vpos->NED, map.wps[i], vpos->available_dist, distance_in, fillet_angle, &line_start);
		}
		if (reached_next_wp == true && i < map.wps.size() - 1) // This if statement handles setting waypoints to get out of the primary waypoints.
		{
			if (check_create_fan(map.wps[i], vpos->NED, root_ptrs[i + 1]) == false)
				reached_next_wp = false;
		}
	}
	// We can go to the next waypoint!
	// The following code wraps up the algorithm.
	clearance = input_file->clearance;									// Bump up the clearance level again.
	node *final_node = new node;
	final_node->NED = map.wps[i];
	final_node->line_start = line_start;
	second2last->children.push_back(final_node);
	*second2last_post_smoothed = second2last->NED;
	final_node->parent = second2last;
	final_node->distance = sqrt(pow(final_node->NED.N - second2last->NED.N, 2) + pow(final_node->NED.E - second2last->NED.E, 2) + pow(final_node->NED.D - second2last->NED.D, 2));
	if (alg_input.path_type == 1 && final_node->parent != root)
		final_node->distance = final_node->distance - 2.0*(*distance_in) + (*fillet_angle)*input_file->turn_radius;
	// populate all wps by working back up the tree
	std::stack<node*> wpstack;
	node *current_node = final_node;
	while (current_node != root)
	{
		wpstack.push(current_node);
		current_node = current_node->parent;
	}
	wpstack.push(root);

	// Now put all of the waypoints into the all_wps vector
	std::vector<NED_s> wps_to_PrimaryWP;
	std::vector<NED_s> line_starts_to_PrimaryWP;
	while (!wpstack.empty())
	{
		wps_to_PrimaryWP.push_back(wpstack.top()->NED);
		line_starts_to_PrimaryWP.push_back(wpstack.top()->line_start);
		wpstack.pop();
	}
	all_wps.push_back(wps_to_PrimaryWP);
	line_starts.push_back(line_starts_to_PrimaryWP);
	wps_to_PrimaryWP.clear();
	line_starts_to_PrimaryWP.clear();
}
void RRT::smoother(bool skip_smoother, unsigned int i, double* distance_in, double* fillet_angle, NED_s* second2last_post_smoothed, bool direct_shot)
{
	node* root = root_ptrs[i];
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
		path_smoothed.push_back(all_wps[i][0]);
		line_starts_smoothed.push_back(line_starts[i][0]);
		NED_s line_start;
		if (i != 0)
		{
			path_smoothed.push_back(all_wps[i][1]);
			line_starts_smoothed.push_back(line_starts[i][1]);
			i_node++;
			j_node++;
			line_Distance = sqrt(pow(path_smoothed[i_node].N - path_smoothed[i_node - 1].N, 2.) + pow(path_smoothed[i_node].E - path_smoothed[i_node - 1].E, 2) + pow(path_smoothed[i_node].D - path_smoothed[i_node - 1].D, 2));
			available_ds.push_back(line_Distance - *distance_in);

			path_smoothed.push_back(all_wps[i][2]);
			line_starts_smoothed.push_back(line_starts[i][2]);
			i_node++;
			j_node++;
			line_Distance = sqrt(pow(path_smoothed[i_node].N - path_smoothed[i_node - 1].N, 2.) + pow(path_smoothed[i_node].E - path_smoothed[i_node - 1].E, 2) + pow(path_smoothed[i_node].D - path_smoothed[i_node - 1].D, 2));
			available_ds.push_back(line_Distance - *distance_in);
		}
		while (j_node < all_wps[i].size())
		{
			bool bad_path_flag = false;
			if ((alg_input.path_type == 0 || i_node == 0) && all_wps[i].size() >= j_node + 2 && (flyZoneCheck(path_smoothed[i_node], all_wps[i][j_node + 1], path_clearance) == false || check_max_slope(line_starts_smoothed[i_node], all_wps[i][j_node + 1]) == false))
				bad_path_flag = true;
			else if (alg_input.path_type == 1 && all_wps[i].size() >= j_node + 2 && (flyZoneCheck(path_smoothed[i_node], all_wps[i][j_node + 1], path_clearance) == false || check_max_slope(line_starts_smoothed[i_node], all_wps[i][j_node + 1]) == false))
				bad_path_flag = true;
			if (alg_input.path_type == 1 && all_wps[i].size() >= j_node + 2 && i_node >= 1 && check_fillet(path_smoothed[i_node - 1], path_smoothed[i_node], all_wps[i][j_node + 1], available_ds[i_node - 1], distance_in, fillet_angle, &line_start) == false)
				bad_path_flag = true;
			if (i != 0 && alg_input.path_type == 1 && j_node == all_wps[i].size() - 3 && bad_path_flag == false)
			{
				double temp_available_ds = sqrt(pow(path_smoothed[i_node].N - all_wps[i][j_node + 2].N, 2) + pow(path_smoothed[i_node].E - all_wps[i][j_node + 2].E, 2) + pow(path_smoothed[i_node].D - all_wps[i][j_node + 2].D, 2)) - *distance_in;
				if (check_fillet(path_smoothed[i_node], all_wps[i][j_node + 1], all_wps[i][j_node + 2], temp_available_ds, distance_in, fillet_angle,&line_start) == false)
				{
					bad_path_flag = true;
					// If this is true then really you should check the next one back.
					// Now we are working backwards to find a good path.
					while (bad_path_flag)
					{
						j_node--;
						temp_available_ds = sqrt(pow(path_smoothed[i_node].N - all_wps[i][j_node + 1].N, 2) + pow(path_smoothed[i_node].E - all_wps[i][j_node + 1].E, 2) + pow(path_smoothed[i_node].D - all_wps[i][j_node + 1].D, 2)) - *distance_in;
						if (flyZoneCheck(path_smoothed[i_node], all_wps[i][j_node + 1], path_clearance))
							if (check_fillet(path_smoothed[i_node - 1], path_smoothed[i_node], all_wps[i][j_node + 1], temp_available_ds, distance_in, fillet_angle, &line_start))
								bad_path_flag = false;
					}
					bad_path_flag = true;
				}
			}
			if (j_node + 1 == all_wps[i].size() - 1 && bad_path_flag == false && i != map.wps.size()-1)
			{
				int previous_fan_nodes = root_ptrs[i + 1]->children.size();
				if (check_create_fan(map.wps[i], path_smoothed[path_smoothed.size() - 1], root_ptrs[i + 1]))
				{
					// Delete the other nodes
					for (unsigned int j = 0; j < root_ptrs[i+1]->children.size(); j++)		// Delete every tree generated
						delete_node(root_ptrs[i+1]->children[j]);
					std::vector<node*>().swap(root_ptrs[i+1]->children);
					root_ptrs[i+1]->children.clear();
					check_create_fan(map.wps[i], path_smoothed[path_smoothed.size() - 1], root_ptrs[i + 1]);
				}
				else
					bad_path_flag == true; // Make sure that jnode + 1 is added.
			}
			if (bad_path_flag) // Try adding the second to last node every time. Avoids problems with smoothing out the last fillet.
			{
				path_smoothed.push_back(all_wps[i][j_node]);
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
		if (i > 0 && i <= map.wps.size() && direct_shot == false && path_smoothed.size() >= 4 && check_direct_fan(path_smoothed[3], root->NED, all_wps[i - 1][all_wps[i - 1].size() - 1], root, &cea, distance_in, fillet_angle))
		{
			if (path_smoothed.size() >= 4 && flyZoneCheck(cea, path_smoothed[3], path_clearance))
			{
				path_smoothed[1] = root->children[root->children.size() - 1]->NED;
				line_starts_smoothed[1] = cea;
				path_smoothed.erase(path_smoothed.begin() + 2);
				line_starts_smoothed.erase(line_starts_smoothed.begin() + 2);
				ROS_INFO("SMOOTHED THE FAN");
			}
		}

		all_wps[i].swap(path_smoothed);
		line_starts[i].swap(line_starts_smoothed);
		*second2last_post_smoothed = all_wps[i][all_wps[i].size() - 1];
	}
	else
		all_wps[i].erase(all_wps[i].begin() + all_wps[i].size() - 1);
	if (taking_off == true)
		taking_off = false;
}
bool RRT::check_slope(NED_s beg, NED_s en)
{
	double slope = atan2(-1.0*(en.D - beg.D), sqrt(pow(beg.N - en.N, 2) + pow(beg.E - en.E, 2)));
	if (slope < -1.0*input_file->descend_angle || slope > input_file->climb_angle)
		return false;
	return true;
}
bool RRT::check_max_slope(NED_s beg, NED_s en)
{
	double slope = atan2(-1.0*(en.D - beg.D), sqrt(pow(beg.N - en.N, 2) + pow(beg.E - en.E, 2)));
	if (slope < -1.0*input_file->max_descend_angle || slope > input_file->max_climb_angle)
		return false;
	return true;
}
void RRT::calc_path_distance(unsigned int i)
{
	// Determine the true path distance. (This kind of covers up some mistakes above...)
	double final_distance = 0;
	for (unsigned j = 0; j < all_wps[i].size() - 1; j++)
	{
		final_distance += sqrt(pow(all_wps[i][j].N - all_wps[i][j + 1].N, 2) + pow(all_wps[i][j].E - all_wps[i][j + 1].E, 2) + pow(all_wps[i][j].D - all_wps[i][j + 1].D, 2));
	}
	final_distance += sqrt(pow(all_wps[i][all_wps[i].size() - 1].N - map.wps[i].N, 2) + pow(all_wps[i][all_wps[i].size() - 1].E - map.wps[i].E, 2) + pow(all_wps[i][all_wps[i].size() - 1].D - map.wps[i].D, 2));
	path_distances.push_back(final_distance);
	ROS_INFO("PATH DISTANCE: %f",path_distances[i]);
}
void RRT::delete_tree()
{
	for (unsigned int i = 0; i < root_ptrs.size(); i++)		// Delete every tree generated
		delete_node(root_ptrs[i]);
}
void RRT::delete_node(node* pn)						// Recursively delete every node
{
	for (unsigned int i = 0; i < pn->children.size();i++)
		delete_node(pn->children[i]);
	pn->children.clear();
	delete pn;
}


void RRT::ppSetup()
{
	// This function is called by the child class at the end of their constructor. There are certain things
	// that need to be set first by that function and then the base class needs to do stuff - the base class constructor
	// would not do it in the right order.

	// This function pulls in the competition boundaries and finds the minimum and maxium North and East positions
	// Even though the mapper class does this, the mapper class won't be put into ROSplane, so it is done here
	// which should be put into ROSplane pathplanner
	is3D = input_file->is3D;
	NED_s boundary_point;
	bool setFirstValues = true;
	for (unsigned int i = 0; i < map.boundary_pts.size(); i++)
	{
		boundary_point = map.boundary_pts[i];
		if (setFirstValues == false)
		{
			maxNorth = (boundary_point.N > maxNorth) ? boundary_point.N : maxNorth; // if new N is greater than maxN, set maxN = new N
			minNorth = (boundary_point.N < minNorth) ? boundary_point.N : minNorth;
			maxEast  = (boundary_point.E > maxEast)  ? boundary_point.E : maxEast;
			minEast  = (boundary_point.E < minEast)  ? boundary_point.E : minEast;
		}
		else
		{
			maxNorth = boundary_point.N;
			minNorth = boundary_point.N;
			maxEast  = boundary_point.E;
			minEast  = boundary_point.E;
			setFirstValues = false;
		}
	}
	clearance = input_file->clearance;		 // Clearance for the path (m)
	minFlyHeight = input_file->minFlyHeight; // 30.48 m = 100 ft. // This still needs to add in the take off altitude
	maxFlyHeight = input_file->maxFlyHeight; // 228.6 m = 750 ft. // This still needs to add in the take off altitude
	iters_limit = input_file->iters_limit;
	taking_off = (input_file->N0 < input_file->minFlyHeight);
	// Also do the all of the calculations on the boundary lines.
	setup_flyZoneCheck();
}
void RRT::setup_flyZoneCheck()				// This function sets up alll of the stuff needed for the flyZoneCheck functions - mostly calculates the y = mx + b for each boundary line
{
	//vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv These lines are used to prep the flyZoneCheck() algorithm
	std::vector<double> NminNmaxEminEmax;				// Yeah, this is a riduculous name...
	std::vector<double> mb;								// Vector of slope and intercepts
	nBPts = map.boundary_pts.size();				// Number of Boundary Points
	double m, b, w, m_w;
	for (unsigned int i = 0; i < nBPts; i++)		// Loop through all points
	{
		// Find the min and max of North and East coordinates on the line connecting two points.
		NminNmaxEminEmax.push_back(std::min(map.boundary_pts[i].N, map.boundary_pts[(i + 1) % nBPts].N));
		NminNmaxEminEmax.push_back(std::max(map.boundary_pts[i].N, map.boundary_pts[(i + 1) % nBPts].N));
		NminNmaxEminEmax.push_back(std::min(map.boundary_pts[i].E, map.boundary_pts[(i + 1) % nBPts].E));
		NminNmaxEminEmax.push_back(std::max(map.boundary_pts[i].E, map.boundary_pts[(i + 1) % nBPts].E));
		lineMinMax.push_back(NminNmaxEminEmax);
		NminNmaxEminEmax.clear();
		// Find the slope and intercept
		m = (map.boundary_pts[(i + 1) % nBPts].N - map.boundary_pts[i].N) / (map.boundary_pts[(i + 1) % nBPts].E - map.boundary_pts[i].E);
		b = -m*map.boundary_pts[i].E + map.boundary_pts[i].N;
		w = (-1.0 / m);
		m_w = m - w;
		mb.push_back(m);
		mb.push_back(b);
		mb.push_back(w);
		mb.push_back(m_w);
		line_Mandb.push_back(mb);
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
	for (unsigned int i = 0; i < nBPts; i++)
	{
		// vvvvvvvvvvvvvvvv Ray Casting, count how many crosses south vvvvvvvvvvvvvvvv
		if (ps.E >= lineMinMax[i][2] && ps.E < lineMinMax[i][3])
		{
			if (ps.N > line_Mandb[i][0] * ps.E + line_Mandb[i][1])
				crossed_lines_ps++;
			else if (ps.N == line_Mandb[i][0] * ps.E + line_Mandb[i][1])
				return false;
		}
		if (pe.E >= lineMinMax[i][2] && pe.E < lineMinMax[i][3])
		{
			if (pe.N > line_Mandb[i][0] * pe.E + line_Mandb[i][1])
				crossed_lines_pe++;
			else if (pe.N == line_Mandb[i][0] * pe.E + line_Mandb[i][1])
				return false;
		}
		// ^^^^^^^^^^^^^^^^ Ray Casting, count how many crosses south ^^^^^^^^^^^^^^^^

		//vvvvvvvvvvvvvvvvvvvvvvvvvvvv Check if any point on the line gets too close to the boundary vvvvvvvvvvvvvvvvvvvvvvvvvvvv
		// Check distance between each endpoint
		if (sqrt(pow(ps.N - map.boundary_pts[i].N, 2) + pow(ps.E - map.boundary_pts[i].E, 2) < r))
			return false;
		if (sqrt(pow(pe.N - map.boundary_pts[i].N, 2) + pow(pe.E - map.boundary_pts[i].E, 2) < r))
			return false;

		// Check if they intersect
		if (line_Mandb[i][0] != path_Mandb[0])
		{
			Ei = (path_Mandb[1] - line_Mandb[i][1]) / (line_Mandb[i][0] - path_Mandb[0]);
			Ni = line_Mandb[i][0] * Ei + line_Mandb[i][1];
			if (Ni > pathMinMax[0] && Ni < pathMinMax[1])
				if (Ni > lineMinMax[i][0] && Ni < lineMinMax[i][1])
					return false;
		}
		// Check distance from bl to each path end point
		bool lp_cleared;
		double lMinMax[4], l_Mandb[4];
		lMinMax[0] = lineMinMax[i][0];
		lMinMax[1] = lineMinMax[i][1];
		lMinMax[2] = lineMinMax[i][2];
		lMinMax[3] = lineMinMax[i][3];
		l_Mandb[0] = line_Mandb[i][0];
		l_Mandb[1] = line_Mandb[i][1];
		l_Mandb[2] = line_Mandb[i][2];
		l_Mandb[3] = line_Mandb[i][3];
		lp_cleared = lineAndPoint2d(map.boundary_pts[i], map.boundary_pts[(i + 1) % nBPts], lMinMax, l_Mandb, ps, r);
		if (lp_cleared == false)
			return false;
		lp_cleared = lineAndPoint2d(map.boundary_pts[i], map.boundary_pts[(i + 1) % nBPts], lMinMax, l_Mandb, pe, r);
		if (lp_cleared == false)
			return false;

		// Check distance from pl to each boundary end point
		lp_cleared = lineAndPoint2d(ps, pe, pathMinMax, path_Mandb, map.boundary_pts[i], r);
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
	if (is3D && taking_off == false)
	{
		if (-ps.D < minFlyHeight + r || -ps.D > maxFlyHeight - r)
			return false;
		if (-pe.D < minFlyHeight + r || -pe.D > maxFlyHeight - r)
			return false;
	}
	// vvvvvvvvvvvvvvvvvvvvv Check to see if the point is within the right fly altitudes vvvvvvvvvvvvvvvvvvvvvv

	//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ Check for Boundary Lines ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	//vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
	//vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv Check for Cylinder Obstacles vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
	bool clearThisCylinder;
	NED_s cylinderPoint;
	for (unsigned int i = 0; i < map.cylinders.size(); i++)
	{
		cylinderPoint.N = map.cylinders[i].N;
		cylinderPoint.E = map.cylinders[i].E;
		cylinderPoint.D = -map.cylinders[i].H;
		clearThisCylinder = lineAndPoint2d(ps, pe, pathMinMax, path_Mandb, cylinderPoint, map.cylinders[i].R + r);

		// Check in 3D. Note that if the above is true, this check does not need to be performed.
		if (is3D && clearThisCylinder == false)
		{
			double dD = (pe.D - ps.D) / sqrt(pow(ps.N - pe.N, 2) + pow(ps.E - pe.E, 2));
			double bt = cylinderPoint.N - path_Mandb[2] * cylinderPoint.E;
			Ei = (bt - path_Mandb[1]) / (path_Mandb[3]);
			Ni = path_Mandb[2] * Ei + bt;
			double bigLength = sqrt(pow(map.cylinders[i].R + r, 2) - pow(Ni - cylinderPoint.N, 2) - pow(Ei - cylinderPoint.E, 2)); // What is bigLength????
			double d2cyl;
			// Check to see if the path is above the cylinder height or into the cylinder
			if (sqrt(pow(ps.N - cylinderPoint.N, 2) + pow(ps.E - cylinderPoint.E, 2)) < map.cylinders[i].R + r && sqrt(pow(pe.N - cylinderPoint.N, 2) + pow(pe.E - cylinderPoint.E, 2)) < map.cylinders[i].R + r)
			{// if BOTH of the endpoints is within the 2d cylinder
				if (-ps.D < map.cylinders[i].H + r)
					return false;
				if (-ps.D < map.cylinders[i].H + r)
					return false;
			}
			else
			{// if at least one waypoint is outside of the 2d cylinder
				if (sqrt(pow(ps.N - cylinderPoint.N, 2) + pow(ps.E - cylinderPoint.E, 2)) < map.cylinders[i].R + r)
				{// if the starting point is within the 2d cylinder
					if (-ps.D < map.cylinders[i].H + r)
						return false;
					// else (check to see if the line that intersects the cylinder is in or out)
					double smallLength = sqrt(pow(Ni - ps.N, 2) + pow(Ei - ps.E, 2));
					if (Ni > pathMinMax[0] && Ni < pathMinMax[1] && Ei > pathMinMax[2] && Ei < pathMinMax[3])
						d2cyl = bigLength + smallLength;
					else
						d2cyl = bigLength - smallLength;
					if (-(dD*d2cyl + ps.D) < map.cylinders[i].H + r)
						return false;
				}
				else if (sqrt(pow(pe.N - cylinderPoint.N, 2) + pow(pe.E - cylinderPoint.E, 2)) < map.cylinders[i].R + r)
				{// if the ending point is within the 2d cylinder
					if (-pe.D < map.cylinders[i].H + r)
						return false;
					// else check to see if the line that intersects the cylinder is in or out
					double smallLength = sqrt(pow(Ni - pe.N, 2) + pow(Ei - pe.E, 2));
					if (Ni > pathMinMax[0] && Ni < pathMinMax[1] && Ei > pathMinMax[2] && Ei < pathMinMax[3])
						d2cyl = bigLength + smallLength;
					else
						d2cyl = bigLength - smallLength;
					if (-(-dD*d2cyl + pe.D) < map.cylinders[i].H + r)
						return false;
				}
				// Now check the two intersection points
				else
				{
					// Calculate the intersection point of the line and the perpendicular line connecting the point
					double d_from_cyl2inter = sqrt(pow(cylinderPoint.N - Ni, 2) + pow(cylinderPoint.E - Ei, 2));
					double daway_from_int = sqrt(pow(r + map.cylinders[i].R, 2) - pow(d_from_cyl2inter, 2)); // WHAT IS THIS?

					// Now test the height at int +- daway_from_int;
					double land_D_ps2i = sqrt(pow(Ni - ps.N, 2) + pow(Ei - ps.E, 2));
					double deltaD = dD*sqrt(pow(Ni - ps.N, 2) + pow(Ei - ps.E, 2));

					double Di = ps.D + dD*sqrt(pow(Ni - ps.N, 2) + pow(Ei - ps.E, 2));

					double height1 = -(Di + dD*daway_from_int);
					double height2 = -(Di - dD*daway_from_int);

					if (-(Di + dD*daway_from_int) < map.cylinders[i].H + r)
						return false;
					if (-(Di - dD*daway_from_int) < map.cylinders[i].H + r)
						return false;
					if (-Di < map.cylinders[i].H + r)
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
	for (unsigned int i = 0; i < nBPts; i++)
	{
		// Find out if the line is either North or South of the line
		if (NED.E >= lineMinMax[i][2] && NED.E < lineMinMax[i][3]) // Only one equal sign solves both the above/ below a vertice problem and the vertical line problem
		{
			if (NED.N > line_Mandb[i][0] * NED.E + line_Mandb[i][1])
				crossed_lines++;
			else if (NED.N == line_Mandb[i][0] * NED.E + line_Mandb[i][1])	// On the rare chance that the point is ON the line
				return false;
		}
		// Check to see if it is too close to the boundary lines
		if (NED.E >= lineMinMax[i][2] - radius && NED.E < lineMinMax[i][3] + radius && NED.N >= lineMinMax[i][0] - radius && NED.N < lineMinMax[i][1] + radius)
		{
			bt = NED.N - line_Mandb[i][2] * NED.E;
			Ei = (bt - line_Mandb[i][1]) / line_Mandb[i][3];
			Ni = line_Mandb[i][2] * Ei + bt;
			// 3 cases first point, second point, or on the line.
			// If the intersection is on the line, dl is the shortest distance
			// Otherwise it is one of the endpoints.
			if (Ni > lineMinMax[i][0] && Ni < lineMinMax[i][1] && Ei > lineMinMax[i][2] && Ei < lineMinMax[i][3])
				shortest_distance = sqrt(pow(Ni - NED.N, 2) + pow(Ei - NED.E, 2));
			else
			{
				de1 = sqrt(pow(map.boundary_pts[i].N - NED.N, 2) + pow(map.boundary_pts[i].E - NED.E, 2));
				de2 = sqrt(pow(map.boundary_pts[(i + 1) % nBPts].N - NED.N, 2) + pow(map.boundary_pts[(i + 1) % nBPts].E - NED.E, 2));
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
	if (is3D && taking_off == false)
		if (-NED.D < minFlyHeight + radius || -NED.D > maxFlyHeight - radius)
			return false;

	// Second, Check for Cylinders
	// Check if the point falls into the volume of the cylinder
	for (unsigned int i = 0; i < map.cylinders.size(); i++)
		if (sqrt(pow(NED.N - map.cylinders[i].N, 2) + pow(NED.E - map.cylinders[i].E, 2)) < map.cylinders[i].R + radius && -NED.D - radius < map.cylinders[i].H)
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
	for (unsigned int i = 0; i < nBPts; i++)
	{
		// vvvvvvvvvvvvvvvv Ray Casting, count how many crosses south vvvvvvvvvvvvvvvv
		if (ps.E >= lineMinMax[i][2] && ps.E < lineMinMax[i][3])
		{
			if (ps.N > line_Mandb[i][0] * ps.E + line_Mandb[i][1])
				crossed_lines_ps++;
			else if (ps.N == line_Mandb[i][0] * ps.E + line_Mandb[i][1])
				return false;
		}
		if (pe.E >= lineMinMax[i][2] && pe.E < lineMinMax[i][3])
		{
			if (pe.N > line_Mandb[i][0] * pe.E + line_Mandb[i][1])
				crossed_lines_pe++;
			else if (pe.N == line_Mandb[i][0] * pe.E + line_Mandb[i][1])
				return false;
		}
		// ^^^^^^^^^^^^^^^^ Ray Casting, count how many crosses south ^^^^^^^^^^^^^^^^

		//vvvvvvvvvvvvvvvvvvvvvvvvvvvv Check if any point on the line gets too close to the boundary vvvvvvvvvvvvvvvvvvvvvvvvvvvv
		if (cp.E >= lineMinMax[i][2] - r - aradius && cp.E <= lineMinMax[i][3] + r + aradius && cp.N >= lineMinMax[i][0] - r - aradius && cp.N <= lineMinMax[i][1] + r + aradius)
		{
			double bt;
			// Calculate the intersection point of the line and the perpendicular line connecting the point
			bt = cp.N - line_Mandb[i][2] * cp.E;
			Ei = (bt - line_Mandb[i][1]) / (line_Mandb[i][3]);
			Ni = line_Mandb[i][2] * Ei + bt;
			if (Ni > lineMinMax[i][0] && Ni < lineMinMax[i][1] && Ei > lineMinMax[i][2] && Ei < lineMinMax[i][3])
			{
				// a dot b = A*B*cos(theta)
				if (line_intersects_arc(Ni, Ei, cp, ps, pe, ccw))
				{
					if (sqrt(pow(Ni - cp.N, 2) + pow(Ei - cp.E, 2)) - aradius < r)
					{
						return false;
					}
				}
				else
				{
					bt = ps.N - line_Mandb[i][2] * ps.E;
					Ei = (bt - line_Mandb[i][1]) / (line_Mandb[i][3]);
					Ni = line_Mandb[i][2] * Ei + bt;
					if (Ni > lineMinMax[i][0] && Ni < lineMinMax[i][1] && Ei > lineMinMax[i][2] && Ei < lineMinMax[i][3])
					{
						if (sqrt(pow(Ni - ps.N, 2) + pow(Ei - ps.E, 2)) < r)
						{
							return false;
						}
					}
					else if (sqrt(pow(map.boundary_pts[i].N - ps.N, 2) + pow(map.boundary_pts[i].E - ps.E, 2)) < r)
					{
						return false;
					}
					else if (sqrt(pow(map.boundary_pts[(i + 1) % nBPts].N - ps.N, 2) + pow(map.boundary_pts[(i + 1) % nBPts].E - ps.E, 2)) < r) { return false; }
					bt = pe.N - line_Mandb[i][2] * pe.E;
					Ei = (bt - line_Mandb[i][1]) / (line_Mandb[i][3]);
					Ni = line_Mandb[i][2] * Ei + bt;
					if (Ni > lineMinMax[i][0] && Ni < lineMinMax[i][1] && Ei > lineMinMax[i][2] && Ei < lineMinMax[i][3])
					{
						if (sqrt(pow(Ni - pe.N, 2) + pow(Ei - pe.E, 2)) < r)
						{
							return false;
						}
					}
					else if (sqrt(pow(map.boundary_pts[i].N - pe.N, 2) + pow(map.boundary_pts[i].E - pe.E, 2)) < r)
					{
						return false;
					}
					//else if (sqrt(pow(map.boundary_pts[(i + 1) % nBPts].N - pe.N, 2) + pow(map.boundary_pts[(i + 1) % nBPts].E - pe.E, 2)) < r) { return false; }
				}
			}
			else
			{
				if (line_intersects_arc(map.boundary_pts[i].N, map.boundary_pts[i].E, cp, ps, pe, ccw))
				{
					if (sqrt(pow(map.boundary_pts[i].N - cp.N, 2) + pow(map.boundary_pts[i].E - cp.E, 2)) - aradius < r)
					{
						return false;
					}
				}
				//if (line_intersects_arc(map.boundary_pts[(i + 1) % nBPts].N, map.boundary_pts[(i + 1) % nBPts].E, cp, ps, pe, ccw))
				//{
				//	if (sqrt(pow(map.boundary_pts[(i + 1) % nBPts].N - cp.N, 2) + pow(map.boundary_pts[(i + 1) % nBPts].E - cp.E, 2)) - aradius < r) { return false; }
				//}
				if (sqrt(pow(map.boundary_pts[i].N - ps.N, 2) + pow(map.boundary_pts[i].E - ps.E, 2)) - aradius < r)
				{
					return false;
				}
				if (sqrt(pow(map.boundary_pts[i].N - pe.N, 2) + pow(map.boundary_pts[i].E - pe.E, 2)) - aradius < r)
				{
					return false;
				}
				//if (sqrt(pow(map.boundary_pts[(i + 1) % nBPts].N - ps.N, 2) + pow(map.boundary_pts[(i + 1) % nBPts].E - ps.E, 2)) - aradius < r) { return false; }
				//if (sqrt(pow(map.boundary_pts[(i + 1) % nBPts].N - pe.N, 2) + pow(map.boundary_pts[(i + 1) % nBPts].E - pe.E, 2)) - aradius < r) { return false; }
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
	if (is3D && taking_off == false)
	{
		if (-ps.D < minFlyHeight + r || -ps.D > maxFlyHeight - r)
			return false;
		if (-pe.D < minFlyHeight + r || -pe.D > maxFlyHeight - r)
			return false;
	}
	// vvvvvvvvvvvvvvvvvvvvv Check to see if the point is within the right fly altitudes vvvvvvvvvvvvvvvvvvvvvv

	//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ Check for Boundary Lines ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	//vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
	//vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv Check for Cylinder Obstacles vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
	bool clearThisCylinder;
	for (unsigned int i = 0; i < map.cylinders.size(); i++)
	{
		if (sqrt(pow(map.cylinders[i].N - cp.N, 2) + pow(map.cylinders[i].E - cp.E, 2)) > r + aradius + map.cylinders[i].R)
			clearThisCylinder = true;
		else if (line_intersects_arc(map.cylinders[i].N, map.cylinders[i].E, cp, ps, pe, ccw))
		{
			if (sqrt(pow(map.cylinders[i].N - cp.N, 2) + pow(map.cylinders[i].E - cp.E, 2)) - aradius - map.cylinders[i].R < r)
			{
				clearThisCylinder = false;
			}
		}
		else
		{
			if (sqrt(pow(map.cylinders[i].N - ps.N, 2) + pow(map.cylinders[i].E - ps.E, 2)) - map.cylinders[i].R < r)
			{
				clearThisCylinder = false;
			}
			else if (sqrt(pow(map.cylinders[i].N - pe.N, 2) + pow(map.cylinders[i].E - pe.E, 2)) - map.cylinders[i].R < r)
			{
				clearThisCylinder = false;
			}
			else { clearThisCylinder = true; }
		}
		if (is3D && clearThisCylinder == false)
		{
			if (ps.D < -map.cylinders[i].H - r && pe.D < -map.cylinders[i].H - r)
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
bool RRT::line_intersects_arc(double Ni, double Ei, NED_s cp, NED_s ps, NED_s pe, bool ccw)
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
void RRT::compute_performance()
{
	total_nWPS = 0;
	for (unsigned int i = 0; i < all_wps.size(); i++)
		total_nWPS += all_wps[i].size();
	total_path_length = 0;
	for (unsigned int i = 0; i < path_distances.size(); i++)
		total_path_length += path_distances[i];
}
} // end namespace theseus
