#include <theseus/path_planner_base.h>

namespace theseus
{
PathPlannerBase::PathPlannerBase()
{

}
PathPlannerBase::~PathPlannerBase()
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
}
void PathPlannerBase::ppSetup()
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
			maxEast  = (boundary_point.E > maxEast) ? boundary_point.E : maxEast;
			minEast  = (boundary_point.E < minEast) ? boundary_point.E : minEast;
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
void PathPlannerBase::solve_static()				// This is a virtual function that the child class should call
{
	ROS_ERROR("Missing the path solver");	// Needs #include <iostream>
}
void PathPlannerBase::setup_flyZoneCheck()				// This function sets up alll of the stuff needed for the flyZoneCheck functions - mostly calculates the y = mx + b for each boundary line
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
bool PathPlannerBase::flyZoneCheck(const NED_s ps, const NED_s pe, const double r) // Point start, point end, radius (clearance)
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

bool PathPlannerBase::flyZoneCheck(const NED_s NED, const double radius) // Point start, radius
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
bool PathPlannerBase::flyZoneCheck(const NED_s ps, const NED_s pe, const double aradius, const NED_s cp, const double r,const bool ccw) // Point start, point end, arc radius, center point, clearance
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
bool PathPlannerBase::line_intersects_arc(double Ni, double Ei, NED_s cp, NED_s ps, NED_s pe, bool ccw)
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
bool PathPlannerBase::lineAndPoint2d(NED_s ls, NED_s le, double MinMax[], double Mandb[], NED_s p, double r)
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
void PathPlannerBase::compute_performance()
{
	total_nWPS = 0;
	for (unsigned int i = 0; i < all_wps.size(); i++)
		total_nWPS += all_wps[i].size();
	total_path_length = 0;
	for (unsigned int i = 0; i < path_distances.size(); i++)
		total_path_length += path_distances[i];
}
}
