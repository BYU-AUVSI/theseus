#include <theseus/collision_detection.h>


namespace theseus
{
CollisionDetection::CollisionDetection()
{

}
CollisionDetection::~CollisionDetection()
{
  for (unsigned int i = 0; i < lineMinMax_.size(); i++)
    std::vector<float>().swap(lineMinMax_[i]);
  std::vector<std::vector<float> >().swap(lineMinMax_);
  for (unsigned int i = 0; i < line_Mandb_.size(); i++)
    std::vector<float>().swap(line_Mandb_[i]);
  std::vector<std::vector<float> >().swap(line_Mandb_);
}
bool CollisionDetection::checkFillet(NED_s w_im1, NED_s w_i, NED_s w_ip1, float R, float clearance)
{
  fillet_s fil;
  bool good_fillet = fil.calculate(w_im1, w_i, w_ip1, R);
  if (good_fillet)
    return checkFillet(fil, clearance);
  else
    return false;
}
bool CollisionDetection::checkFillet(fillet_s fil, float clearance)
{
  // check for a collision on the first line, the arc and the second line
  bool first_line, middle_arc, second_line;
  first_line  = checkLine(fil.w_im1, fil.z1, clearance);
  middle_arc  = checkArc(fil.z1, fil.z2, fil.R, fil.c, fil.lambda, clearance);
  second_line = checkLine(fil.z2, fil.w_ip1, clearance);
  // if (first_line) {ROS_DEBUG("first line passed");}
  // else {ROS_FATAL("first line FAILED"); ROS_DEBUG("n_beg: %f, e_beg: %f, d_beg: %f, n_end: %f, e_end: %f, d_end: %f, ",\
  fil.w_im1.N, fil.w_im1.E, fil.w_im1.D, fil.z1.N, fil.z1.E, fil.z1.D);}
  // if (middle_arc) {ROS_DEBUG("arc passed");}
  // else {ROS_FATAL("arc FAILED");}
  // if (second_line) {ROS_DEBUG("second line passed");}
  // else {ROS_FATAL("second line FAILED"); ROS_DEBUG("n_beg: %f, e_beg: %f, d_beg: %f, n_end: %f, e_end: %f, d_end: %f, ",\
  fil.z2.N, fil.z2.E, fil.z2.D, fil.w_ip1.N, fil.w_ip1.E, fil.w_ip1.D);}

  if (first_line && middle_arc && second_line)
    return true;
  else
    return false;
}
bool CollisionDetection::checkPoint(NED_s point, float clearance)
{
  float radius = clearance;
  // This is a more simple version of the collision****() that just checks if the point point is at least radius away from any obstacle.
	// First, Check Within the Boundaries
	bool withinBoundaries;
	// Look at the Point in Polygon Algorithm
	// Focus on rays South.
	int crossed_lines = 0;							// This is a counter of the number of lines that the point is NORTH of.
	float bt, Ei, Ni, de1, de2, shortest_distance;
	for (unsigned int i = 0; i < nBPts_; i++)
	{
		// Find out if the line is either North or South of the line
		if (point.E >= lineMinMax_[i][2] && point.E < lineMinMax_[i][3]) // Only one equal sign solves both the above/ below a vertice problem and the vertical line problem
		{
			if (point.N > line_Mandb_[i][0] * point.E + line_Mandb_[i][1])
				crossed_lines++;
			else if (point.N == line_Mandb_[i][0] * point.E + line_Mandb_[i][1])	// On the rare chance that the point is ON the line
				return false;
		}
		// Check to see if it is too close to the boundary lines
		if (point.E >= lineMinMax_[i][2] - radius && point.E < lineMinMax_[i][3] + radius && point.N >= lineMinMax_[i][0] - radius && point.N < lineMinMax_[i][1] + radius)
		{
			bt = point.N - line_Mandb_[i][2] * point.E;
			Ei = (bt - line_Mandb_[i][1]) / line_Mandb_[i][3];
			Ni = line_Mandb_[i][2] * Ei + bt;
			// 3 cases first point, second point, or on the line.
			// If the intersection is on the line, dl is the shortest distance
			// Otherwise it is one of the endpoints.
			if (Ni > lineMinMax_[i][0] && Ni < lineMinMax_[i][1] && Ei > lineMinMax_[i][2] && Ei < lineMinMax_[i][3])
				shortest_distance = sqrtf(powf(Ni - point.N, 2.0f) + powf(Ei - point.E, 2.0f));
			else
			{
				de1 = sqrtf(powf(map_.boundary_pts[i].N - point.N, 2.0f) + powf(map_.boundary_pts[i].E - point.E, 2.0f));
				de2 = sqrtf(powf(map_.boundary_pts[(i + 1) % nBPts_].N - point.N, 2.0f) + powf(map_.boundary_pts[(i + 1) % nBPts_].E - point.E, 2.0f));
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
	if (taking_off_ == false && landing_now_ == false)
		if (-point.D < minFlyHeight_ + radius || -point.D > maxFlyHeight_ - radius)
			return false;

	// Second, Check for Cylinders
	// Check if the point falls into the volume of the cylinder
	for (unsigned int i = 0; i < map_.cylinders.size(); i++)
		if (sqrtf(powf(point.N - map_.cylinders[i].N, 2.0f) + powf(point.E - map_.cylinders[i].E, 2.0f)) < map_.cylinders[i].R + radius && -point.D - radius < map_.cylinders[i].H)
			return false;
	return true; // The coordinate is in the safe zone if it got to here!
}
bool CollisionDetection::checkLine(NED_s ps, NED_s pe, float clearance)
{
  if (checkClimbAngle(ps, pe) == false)
  {
    //ROS_DEBUG("line exit 22");
    return false;
  }
  // Determines if a line conn ps and pe gets within clearance of any obstacle or boundary
	// Preliminary Calculations about the line connecting ps and pe
  float pathMinMax[4];
	float path_Mandb[4];
	pathMinMax[0] = std::min(ps.N, pe.N);
	pathMinMax[1] = std::max(ps.N, pe.N);
	pathMinMax[2] = std::min(ps.E, pe.E);
	pathMinMax[3] = std::max(ps.E, pe.E);
	path_Mandb[0] = (pe.N - ps.N) / (pe.E - ps.E);
	path_Mandb[1] = pe.N - path_Mandb[0] * pe.E;
	path_Mandb[2] = -1.0 / path_Mandb[0];
	path_Mandb[3] = path_Mandb[0] - path_Mandb[2];
	float Ei, Ni;

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
      {
        //ROS_DEBUG("line exit 1");
        return false;
      }
		}
		if (pe.E >= lineMinMax_[i][2] && pe.E < lineMinMax_[i][3])
		{
			if (pe.N > line_Mandb_[i][0] * pe.E + line_Mandb_[i][1])
				crossed_lines_pe++;
			else if (pe.N == line_Mandb_[i][0] * pe.E + line_Mandb_[i][1])
      {
        //ROS_DEBUG("line exit 2");
        return false;
      }
		}
		// ^^^^^^^^^^^^^^^^ Ray Casting, count how many crosses south ^^^^^^^^^^^^^^^^

		//vvvvvvvvvvvvvvvvvvvvvvvvvvvv Check if any point on the line gets too close to the boundary vvvvvvvvvvvvvvvvvvvvvvvvvvvv
		// Check distance between each endpoint
		if (sqrtf(powf(ps.N - map_.boundary_pts[i].N, 2.0f) + powf(ps.E - map_.boundary_pts[i].E, 2.0f) < clearance))
    {
      //ROS_DEBUG("line exit 3");
      return false;
    }
		if (sqrtf(powf(pe.N - map_.boundary_pts[i].N, 2.0f) + powf(pe.E - map_.boundary_pts[i].E, 2.0f) < clearance))
    {
      //ROS_DEBUG("line exit 4");
      return false;
    }
		// Check if they intersect
		if (line_Mandb_[i][0] != path_Mandb[0])
		{
			Ei = (path_Mandb[1] - line_Mandb_[i][1]) / (line_Mandb_[i][0] - path_Mandb[0]);
			Ni = line_Mandb_[i][0] * Ei + line_Mandb_[i][1];
			if (Ni > pathMinMax[0] && Ni < pathMinMax[1])
				if (Ni > lineMinMax_[i][0] && Ni < lineMinMax_[i][1])
        {
          //ROS_DEBUG("line exit 5");
          return false;
        }
		}
		// Check distance from bl to each path end point
		bool lp_cleared;
		float lMinMax[4], l_Mandb[4];
		lMinMax[0] = lineMinMax_[i][0];
		lMinMax[1] = lineMinMax_[i][1];
		lMinMax[2] = lineMinMax_[i][2];
		lMinMax[3] = lineMinMax_[i][3];
		l_Mandb[0] = line_Mandb_[i][0];
		l_Mandb[1] = line_Mandb_[i][1];
		l_Mandb[2] = line_Mandb_[i][2];
		l_Mandb[3] = line_Mandb_[i][3];
		lp_cleared = lineAndPoint2d(map_.boundary_pts[i], map_.boundary_pts[(i + 1) % nBPts_], lMinMax, l_Mandb, ps, clearance);
		if (lp_cleared == false)
    {
      //ROS_DEBUG("line exit 6");
      return false;
    }
		lp_cleared = lineAndPoint2d(map_.boundary_pts[i], map_.boundary_pts[(i + 1) % nBPts_], lMinMax, l_Mandb, pe, clearance);
		if (lp_cleared == false)
    {
      //ROS_DEBUG("line exit 7");
      return false;
    }
		// Check distance from pl to each boundary end point
		lp_cleared = lineAndPoint2d(ps, pe, pathMinMax, path_Mandb, map_.boundary_pts[i], clearance);
		if (lp_cleared == false)
    {
      //ROS_DEBUG("line exit 8");
      return false;
    }
		//^^^^^^^^^^^^^^^^^^^^^^^^^^^^ Check if any point on the line gets too close to the boundary ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	}

	// vvvvvvvvvvvvvvvv Finish up checking if the end points were both inside the boundary (ray casting) vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
	withinBoundaries_ps = crossed_lines_ps % 2; // If it crosses an even number of boundaries it is NOT inside, if it crosses an odd number it IS inside
	withinBoundaries_pe = crossed_lines_pe % 2;
	if (withinBoundaries_ps == false || withinBoundaries_pe == false)
  {
    //ROS_DEBUG("line exit 9");
    return false;
  }
	// ^^^^^^^^^^^^^^^^ Finish up checking if the end points were both inside the boundary (ray casting) ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	// vvvvvvvvvvvvvvvvvvvvv Check to see if the point is within the right fly altitudes vvvvvvvvvvvvvvvvvvvvvv
	if (taking_off_ == false && landing_now_ == false)
	{
		if (-ps.D < minFlyHeight_ + clearance || -ps.D > maxFlyHeight_ - clearance)
    {
      //ROS_DEBUG("line exit 10");
      return false;
    }
		if (-pe.D < minFlyHeight_ + clearance || -pe.D > maxFlyHeight_ - clearance)
    {
      //ROS_DEBUG("line exit 11");
      return false;
    }
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
		clearThisCylinder = lineAndPoint2d(ps, pe, pathMinMax, path_Mandb, cylinderPoint, map_.cylinders[i].R + clearance);

		// Note that if the above is true, this check does not need to be performed.
		if (clearThisCylinder == false)
		{
			float dD = (pe.D - ps.D) / sqrtf(powf(ps.N - pe.N, 2.0f) + powf(ps.E - pe.E, 2.0f));
			float bt = cylinderPoint.N - path_Mandb[2] * cylinderPoint.E;
			Ei = (bt - path_Mandb[1]) / (path_Mandb[3]);
			Ni = path_Mandb[2] * Ei + bt;
			float bigLength = sqrtf(powf(map_.cylinders[i].R + clearance, 2.0f) - powf(Ni - cylinderPoint.N, 2.0f) - powf(Ei - cylinderPoint.E, 2.0f)); // What is bigLength????
			float d2cyl;
			// Check to see if the path is above the cylinder height or into the cylinder
			if (sqrtf(powf(ps.N - cylinderPoint.N, 2.0f) + powf(ps.E - cylinderPoint.E, 2.0f)) < map_.cylinders[i].R + clearance && sqrtf(powf(pe.N - cylinderPoint.N, 2.0f) + powf(pe.E - cylinderPoint.E, 2.0f)) < map_.cylinders[i].R + clearance)
			{// if BOTH of the endpoints is within the 2d cylinder
				if (-ps.D < map_.cylinders[i].H + clearance)
        {
          //ROS_DEBUG("line exit 12");
          return false;
        }
				if (-ps.D < map_.cylinders[i].H + clearance)
        {
          //ROS_DEBUG("line exit 13");
          return false;
        }
			}
			else
			{// if at least one waypoint is outside of the 2d cylinder
				if (sqrtf(powf(ps.N - cylinderPoint.N, 2.0f) + powf(ps.E - cylinderPoint.E, 2.0f)) < map_.cylinders[i].R + clearance)
				{// if the starting point is within the 2d cylinder
					if (-ps.D < map_.cylinders[i].H + clearance)
          {
            //ROS_DEBUG("line exit 14");
            return false;
          }
					// else (check to see if the line that intersects the cylinder is in or out)
					float smallLength = sqrtf(powf(Ni - ps.N, 2.0f) + powf(Ei - ps.E, 2.0f));
					if (Ni > pathMinMax[0] && Ni < pathMinMax[1] && Ei > pathMinMax[2] && Ei < pathMinMax[3])
						d2cyl = bigLength + smallLength;
					else
						d2cyl = bigLength - smallLength;
					if (-(dD*d2cyl + ps.D) < map_.cylinders[i].H + clearance)
          {
            //ROS_DEBUG("line exit 15");
            return false;
          }
				}
				else if (sqrtf(powf(pe.N - cylinderPoint.N, 2.0f) + powf(pe.E - cylinderPoint.E, 2.0f)) < map_.cylinders[i].R + clearance)
				{// if the ending point is within the 2d cylinder
					if (-pe.D < map_.cylinders[i].H + clearance)
          {
            //ROS_DEBUG("line exit 16");
            return false;
          }
					// else check to see if the line that intersects the cylinder is in or out
					float smallLength = sqrtf(powf(Ni - pe.N, 2.0f) + powf(Ei - pe.E, 2.0f));
					if (Ni > pathMinMax[0] && Ni < pathMinMax[1] && Ei > pathMinMax[2] && Ei < pathMinMax[3])
						d2cyl = bigLength + smallLength;
					else
						d2cyl = bigLength - smallLength;
					if (-(-dD*d2cyl + pe.D) < map_.cylinders[i].H + clearance)
          {
            //ROS_DEBUG("line exit 17");
            return false;
          }
				}
				// Now check the two intersection points
				else
				{
					// Calculate the intersection point of the line and the perpendicular line connecting the point
					float d_from_cyl2inter = sqrtf(powf(cylinderPoint.N - Ni, 2.0f) + powf(cylinderPoint.E - Ei, 2.0f));
					float daway_from_int = sqrtf(powf(clearance + map_.cylinders[i].R, 2.0f) - powf(d_from_cyl2inter, 2.0f)); // WHAT IS THIS?

					// Now test the height at int +- daway_from_int;
					float land_D_ps2i = sqrtf(powf(Ni - ps.N, 2.0f) + powf(Ei - ps.E, 2.0f));
					float deltaD = dD*sqrtf(powf(Ni - ps.N, 2.0f) + powf(Ei - ps.E, 2.0f));

					float Di = ps.D + dD*sqrtf(powf(Ni - ps.N, 2.0f) + powf(Ei - ps.E, 2.0f));

					float height1 = -(Di + dD*daway_from_int);
					float height2 = -(Di - dD*daway_from_int);

					if (-(Di + dD*daway_from_int) < map_.cylinders[i].H + clearance)
          {
            //ROS_DEBUG("line exit 18");
            return false;
          }
					if (-(Di - dD*daway_from_int) < map_.cylinders[i].H + clearance)
          {
            //ROS_DEBUG("line exit 19");
            return false;
          }
					if (-Di < map_.cylinders[i].H + clearance)
          {
            //ROS_DEBUG("line exit 20");
            return false;
          }
				}
			}
			clearThisCylinder = true;
		}
		if (clearThisCylinder == false)
    {
      //ROS_DEBUG("line exit 21");
      return false;
    }
	}
	//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ Check for Cylinder Obstacles ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  return true; // The line is in the safe zone if it got to here!
}
bool CollisionDetection::checkClimbAngle(NED_s beg, NED_s en)
{
  float slope = atan2f(-1.0f*(en.D - beg.D), sqrtf(powf(beg.N - en.N, 2.0f) + powf(beg.E - en.E, 2.0f)));
  if (slope < -1.0f*input_file_.max_descend_angle || slope > input_file_.max_climb_angle)
  {
    //ROS_DEBUG("slope %f, de %f, climb %f", slope*180.0/M_PI, -input_file_.max_descend_angle*180.0/M_PI, input_file_.max_climb_angle*180.0/M_PI);
    return false;
  }
  return true;
}
bool CollisionDetection::checkAfterWP(NED_s p, float chi, float clearance)
{
  bool found_at_least_1_good_path = false;
  // Make sure that it is possible to go to the next waypoint

  float alpha  = M_PI / 4.0;			// Start with 45.0 degrees // TODO this might cause errors depending on the turning radius
  float R      = 3.0*input_file_.turn_radius;
  int num_circle_trials = 10;				// Will try num_circle_trials on one side and num_circle_trials on the other side.
  float dalpha = (M_PI - alpha) / num_circle_trials;

  float approach_angle = -chi + M_PI/2.0f; //atan2(p.N - coming_from.N, p.E - coming_from.E) + M_PI;
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
    d      = input_file_.turn_radius / tanf(gamma / 2.0);

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

    if (checkArc(p, cea, input_file_.turn_radius, cpa, -1, clearance))
      if (checkLine(cea, lea, clearance))
        return true;
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

    if (checkArc(p, cea, input_file_.turn_radius, cpa, 1, clearance))
      if (checkLine(cea, lea, clearance))
        return true;
        // node *fake_child = new node;
        // node *normal_gchild = new node;
        // fake_child->NED = fake_wp;
        // fake_child->available_dist = 0;
        // fake_child->parent = next_root;
        // fake_child->distance = 2.0*zeta*input_file_->turn_radius;
        // fake_child->path_type = 1;
        // fake_child->line_start = next_root->NED;
        // next_root->children.push_back(fake_child);
        //
        // normal_gchild->NED = lea;
        // normal_gchild->available_dist = sqrt(R*(R - input_file_->turn_radius*sin(lambda) / sin(beta)));
        // normal_gchild->parent = fake_child;
        // normal_gchild->distance = normal_gchild->available_dist;
        // normal_gchild->path_type = 1;
        // normal_gchild->line_start = cea;
        // fake_child->children.push_back(normal_gchild);
  }
  return false;
}
bool CollisionDetection::checkArc(NED_s ps, NED_s pe, float R, NED_s cp, int lambda, float clearance)
{
  float r  = clearance;
  bool ccw = lambda < 0 ? true : false; // ccw = lambda(-1),
  float aradius = R;

  // Determines if arc gets within r of an obstacle
  // Preliminary Calculations about the arc connecting ps and pe
  float Ei, Ni;
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
  		float bt;
  		// Calculate the intersection point of the line and the perpendicular line connecting the point
  		bt = cp.N - line_Mandb_[i][2] * cp.E;
  		Ei = (bt - line_Mandb_[i][1]) / (line_Mandb_[i][3]);
  		Ni = line_Mandb_[i][2] * Ei + bt;
  		if (Ni > lineMinMax_[i][0] && Ni < lineMinMax_[i][1] && Ei > lineMinMax_[i][2] && Ei < lineMinMax_[i][3])
  		{
  			// a dot b = A*B*cos(theta)
  			if (lineIntersectsArc(Ni, Ei, cp, ps, pe, ccw))
  			{
  				if (sqrtf(powf(Ni - cp.N, 2.0f) + powf(Ei - cp.E, 2.0f)) - aradius < r)
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
  					if (sqrtf(powf(Ni - ps.N, 2.0f) + powf(Ei - ps.E, 2.0f)) < r)
  					{
  						return false;
  					}
  				}
  				else if (sqrtf(powf(map_.boundary_pts[i].N - ps.N, 2.0f) + powf(map_.boundary_pts[i].E - ps.E, 2.0f)) < r)
  				{
  					return false;
  				}
  				else if (sqrtf(powf(map_.boundary_pts[(i + 1) % nBPts_].N - ps.N, 2.0f) + powf(map_.boundary_pts[(i + 1) % nBPts_].E - ps.E, 2.0f)) < r) { return false; }
  				bt = pe.N - line_Mandb_[i][2] * pe.E;
  				Ei = (bt - line_Mandb_[i][1]) / (line_Mandb_[i][3]);
  				Ni = line_Mandb_[i][2] * Ei + bt;
  				if (Ni > lineMinMax_[i][0] && Ni < lineMinMax_[i][1] && Ei > lineMinMax_[i][2] && Ei < lineMinMax_[i][3])
  				{
  					if (sqrtf(powf(Ni - pe.N, 2.0f) + powf(Ei - pe.E, 2.0f)) < r)
  					{
  						return false;
  					}
  				}
  				else if (sqrtf(powf(map_.boundary_pts[i].N - pe.N, 2.0f) + powf(map_.boundary_pts[i].E - pe.E, 2.0f)) < r)
  				{
  					return false;
  				}
  				//else if (sqrtf(powf(map_.boundary_pts[(i + 1) % nBPts_].N - pe.N, 2.0f) + powf(map_.boundary_pts[(i + 1) % nBPts_].E - pe.E, 2.0f)) < r) { return false; }
  			}
  		}
  		else
  		{
  			if (lineIntersectsArc(map_.boundary_pts[i].N, map_.boundary_pts[i].E, cp, ps, pe, ccw))
  			{
  				if (sqrtf(powf(map_.boundary_pts[i].N - cp.N, 2.0f) + powf(map_.boundary_pts[i].E - cp.E, 2.0f)) - aradius < r)
  				{
  					return false;
  				}
  			}
  			//if (lineIntersectsArc(map_.boundary_pts[(i + 1) % nBPts_].N, map_.boundary_pts[(i + 1) % nBPts_].E, cp, ps, pe, ccw))
  			//{
  			//	if (sqrtf(powf(map_.boundary_pts[(i + 1) % nBPts_].N - cp.N, 2.0f) + powf(map_.boundary_pts[(i + 1) % nBPts_].E - cp.E, 2.0f)) - aradius < r) { return false; }
  			//}
  			if (sqrtf(powf(map_.boundary_pts[i].N - ps.N, 2.0f) + powf(map_.boundary_pts[i].E - ps.E, 2.0f)) - aradius < r)
  			{
  				return false;
  			}
  			if (sqrtf(powf(map_.boundary_pts[i].N - pe.N, 2.0f) + powf(map_.boundary_pts[i].E - pe.E, 2.0f)) - aradius < r)
  			{
  				return false;
  			}
  			//if (sqrtf(powf(map_.boundary_pts[(i + 1) % nBPts_].N - ps.N, 2.0f) + powf(map_.boundary_pts[(i + 1) % nBPts_].E - ps.E, 2.0f)) - aradius < r) { return false; }
  			//if (sqrtf(powf(map_.boundary_pts[(i + 1) % nBPts_].N - pe.N, 2.0f) + powf(map_.boundary_pts[(i + 1) % nBPts_].E - pe.E, 2.0f)) - aradius < r) { return false; }
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
  if (taking_off_ == false && landing_now_ == false)
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
  	if (sqrtf(powf(map_.cylinders[i].N - cp.N, 2.0f) + powf(map_.cylinders[i].E - cp.E, 2.0f)) > r + aradius + map_.cylinders[i].R)
  		clearThisCylinder = true;
  	else if (lineIntersectsArc(map_.cylinders[i].N, map_.cylinders[i].E, cp, ps, pe, ccw))
  	{
  		if (sqrtf(powf(map_.cylinders[i].N - cp.N, 2.0f) + powf(map_.cylinders[i].E - cp.E, 2.0f)) - aradius - map_.cylinders[i].R < r)
  		{
  			clearThisCylinder = false;
  		}
  	}
  	else
  	{
  		if (sqrtf(powf(map_.cylinders[i].N - ps.N, 2.0f) + powf(map_.cylinders[i].E - ps.E, 2.0f)) - map_.cylinders[i].R < r)
  		{
  			clearThisCylinder = false;
  		}
  		else if (sqrtf(powf(map_.cylinders[i].N - pe.N, 2.0f) + powf(map_.cylinders[i].E - pe.E, 2.0f)) - map_.cylinders[i].R < r)
  		{
  			clearThisCylinder = false;
  		}
  		else { clearThisCylinder = true; }
  	}
  	if (clearThisCylinder == false)
  	{
  		if (ps.D < -map_.cylinders[i].H - r && pe.D < -map_.cylinders[i].H - r)
  			clearThisCylinder = true;
  	}
  	if (clearThisCylinder == false)
  		return false;
  }
  //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ Check for Cylinder Obstacles ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  return true; // The arc is in the safe zone if it got to here!
}
bool CollisionDetection::lineAndPoint2d(NED_s ls, NED_s le, float MinMax[], float Mandb[], NED_s p, float r)
{
	// This function is used a lot by the collisionLine.
	// It checks to see if the point p is at least r away from the line segment connecting ls and le. 2 Dimensional Projection
	// This function is pretty important to have right.
	float bt, Ei, Ni, shortest_distance, de1, de2;
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
			shortest_distance = sqrtf(powf(Ni - p.N, 2.0f) + powf(Ei - p.E, 2.0f));
		else
		{
			de1 = sqrtf(powf(ls.N - p.N, 2.0f) + powf(ls.E - p.E, 2.0f));
			de2 = sqrtf(powf(le.N - p.N, 2.0f) + powf(le.E - p.E, 2.0f));
			shortest_distance = std::min(de1, de2);
		}
		if (shortest_distance < r)
			return false;
	}
	return true;	// It is at least r away from the line if it got to here.
}
bool CollisionDetection::lineIntersectsArc(float Ni, float Ei, NED_s cp, NED_s ps, NED_s pe, bool ccw)
{
	// Find angle from cp to ps
	float aC2s = atan2f(ps.N - cp.N, ps.E - cp.E);
	// Find angle from cp to pe
	float aC2e = atan2f(pe.N - cp.N, pe.E - cp.E);
	// Find angle from cp to Ni, Ei
	float aC2i = atan2f(Ni - cp.N, Ei - cp.E);
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

void CollisionDetection::newMap(map_s map_in)
{
  for (unsigned int i = 0; i < lineMinMax_.size(); i++)
    std::vector<float>().swap(lineMinMax_[i]);
  std::vector<std::vector<float> >().swap(lineMinMax_);
  for (unsigned int i = 0; i < line_Mandb_.size(); i++)
    std::vector<float>().swap(line_Mandb_[i]);
  std::vector<std::vector<float> >().swap(line_Mandb_);
  map_        = map_in;          // Get a copy of the terrain map
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
	minFlyHeight_ = input_file_.minFlyHeight;  // 30.48 m = 100 ft. // This still needs to add in the take off altitude
	maxFlyHeight_ = input_file_.maxFlyHeight;  // 228.6 m = 750 ft. // This still needs to add in the take off altitude
  //vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv These lines are used to prep the flyZoneCheck() algorithm
	std::vector<float> NminNmaxEminEmax;       // Yeah, this is a riduculous name...
	std::vector<float> mb;                     // Vector of slope and intercepts
	nBPts_ = map_.boundary_pts.size();          // Number of Boundary Points
	float m, b, w, m_w;
	for (unsigned int i = 0; i < nBPts_; i++)   // Loop through all points
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

// Debug print functions
void CollisionDetection::printBoundsObstacles()
{
  ROS_INFO("Number of Boundary Points: %lu",  map_.boundary_pts.size());
  for (long unsigned int i = 0; i < map_.boundary_pts.size(); i++)
  {
    ROS_INFO("Boundary: %lu, North: %f, East: %f, Down: %f", i, map_.boundary_pts[i].N, map_.boundary_pts[i].E, map_.boundary_pts[i].D);
  }
  ROS_INFO("Number of Cylinders: %lu", map_.cylinders.size());
  for (long unsigned int i = 0; i <  map_.cylinders.size(); i++)
  {
    ROS_INFO("Cylinder: %lu, North: %f, East: %f, Radius: %f, Height: %f", i, map_.cylinders[i].N, map_.cylinders[i].E, map_.cylinders[i].R,  map_.cylinders[i].H);
  }
}
} // end namespace theseus
