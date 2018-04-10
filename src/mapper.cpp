/*	DESCRIPTION:
*	This is the cpp for the mapper class.
*	The mapper is seeded with an unsigned int. Every time the mapper is seeded with the
*	same seed it will produce the same exact map! (This may not be true if using different
*	compilers etc).
*	The functions includes loading in boundaries, randomly generating
*	static obstacles (cylinders), (eventually) generating .world files, and possibly
*	generating moving obstacles.
*/

#include <theseus/mapper.h>

namespace theseus
{
mapper::mapper()
{

}
mapper::mapper(unsigned int seed, ParamReader *input_file_in)
{
	// Keep the address of the input file class
	input_file = input_file_in;

	// Create the random generator
	RandGen rg_in(seed);
	rg = rg_in;

	// Some settings for Generating the Map
	waypoint_clearance = input_file->waypoint_clearance;// (m) This is the minimum clearance that each waypoint has with other obstacles... Just to make things reasonable.
	is3D               = input_file->is3D;              // This make the board 3D, which pretty much just means the cylinders have a specific height and waypoints can be above them.

														// Set up some competition constants
	minCylRadius = input_file->minCylRadius; // 9.144 m = 30 ft.
	maxCylRadius = input_file->maxCylRadius; // 91.44 m = 300 ft.
	minCylHeight = input_file->minCylHeight; // 9.144 m = 30 ft.
	maxCylHeight = input_file->maxCylHeight; // 228.6 m = 750 ft.
	minFlyHeight = input_file->minFlyHeight; // 23.7744 m = 78 ft.
	maxFlyHeight = input_file->maxFlyHeight; // 221.8944 m = 728 ft.

	// Pull in the competition boundaries
	// Get the NED coordinate frame set up
  double rPhi, rLam, rH;
	rPhi = input_file->lat_ref;
  rLam = input_file->lon_ref;
  rH   = input_file->h_ref;
  gps_struct gps_converter;
  gps_converter.set_reference(rPhi, rLam, rH);

	NED_s boundary_point;
	bool setFirstValues = true;
	double phi, lambda;
  double default_boundaries_lat[12] = {38.146269444444442, 38.151624999999996, 38.151888888888891, 38.150594444444444,\
                                       38.147566666666670, 38.144666666666666, 38.143255555555555, 38.140463888888888,\
                                       38.140719444444443, 38.143761111111111, 38.147347222222223, 38.146130555555558};
  double default_boundaries_lon[12] = {-76.42816388888888, -76.42868333333333, -76.43146666666666, -76.43536111111112,\
                                       -76.43234166666667, -76.43294722222222, -76.43476666666667, -76.43263611111112,\
                                       -76.42601388888888, -76.42120555555555, -76.42321111111111, -76.42665277777779};

  for (int i = 0; i < 12; i++)
  {
    phi            = default_boundaries_lat[i];
    lambda         = default_boundaries_lon[i];
    gps_converter.gps2ned(phi, lambda, rH, boundary_point.N, boundary_point.E, boundary_point.D);
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
		map.boundary_pts.push_back(boundary_point);	// This line puts the boundary points into the map member.
	}

  // float temp_latitude  =   39.969473f;
  // float temp_longitude = -111.966988f;
  // float temp_height    = 1430.860f;
  // gps_converter.set_reference(temp_latitude, temp_longitude, temp_height);
  // for (int i = 0; i < 12; i++)
  // {
  //   double N, E, D;
  //   N = (double) map.boundary_pts[i].N;
  //   E = (double) map.boundary_pts[i].E;
  //   D = (double) map.boundary_pts[i].D;
  //   double temp_lat, temp_long, temp_h;
  //   gps_converter.ned2gps(N, E, D, temp_lat, temp_long, temp_h);
  //   ROS_INFO("point %i, latitude: %f, longitude: %f, height: %f", i, temp_lat, temp_long, temp_h);
	// }
  // gps_converter.set_reference(rPhi, rLam, rH);

	// Set up flyZoneCheck()
	//vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv These lines are used to prep the flyZoneCheck() algorithm
	std::vector<double> NminNmaxEminEmax; // Yeah, this is a riduculous name...
	std::vector<double> mb;               // Vector of slope and intercepts
	nBPts = map.boundary_pts.size();      // Number of Boundary Points
	double m, b, w, m_w;
	for (unsigned int i = 0; i < nBPts; i++)   // Loop through all points
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
	//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ These lines are used to set up the flyZoneCheck() algorithm.

	// Randomly Generate nCyli Cylinders
	// can be up to 10 cylinders in the competition
	nCyli = input_file->nCyli;												// Number of Cylinders
	cyl_s cyl;																// Cylinder object
	for (unsigned int i = 0; i < nCyli; i++)
	{
		// Generate a position and radius
		cyl.N = rg.randLin()*(maxNorth - minNorth) + minNorth;
		cyl.E = rg.randLin()*(maxEast - minEast) + minEast;
		cyl.R = rg.randLin()*(maxCylRadius - minCylRadius) + minCylRadius;
		// Check to see if it can fit there
		while (flyZoneCheck(cyl) == false || sqrt(pow(input_file->N_init - cyl.N, 2.0) + pow(input_file->E_init - cyl.E, 2.0)) < cyl.R + input_file->waypoint_clearance)
		{
			cyl.N = rg.randLin()*(maxNorth - minNorth) + minNorth;
			cyl.E = rg.randLin()*(maxEast - minEast) + minEast;
			cyl.R = rg.randLin()*(maxCylRadius - minCylRadius) + minCylRadius;
		}
		// If we are doing 3 dimensions then generate random height, otherwise give it the maximum height
		if (is3D)
			cyl.H = rg.randLin()*(maxCylHeight - minCylHeight) + minCylHeight;
		else
			cyl.H = maxFlyHeight;
		// Put the cylinder into the terrain map
		map.cylinders.push_back(cyl);
	}
	int numWps = input_file->numWps; // This is the number of Primary Waypoints
	NED_s wp;
	// Randomly generate some waypoints
	for (int i = 0; i < numWps; i++)
	{
		wp.N = rg.randLin()*(maxNorth - minNorth) + minNorth;
		wp.E = rg.randLin()*(maxEast - minEast) + minEast;
		if (is3D)
			wp.D = (rg.randLin()*((maxFlyHeight-waypoint_clearance) - (minFlyHeight+waypoint_clearance)) + minFlyHeight + waypoint_clearance)*-1.0; // Put the MSL into down (make it negative)
		else
			wp.D = 0;
		// Check to see if the placement is good, keep generating a new one until it fits
		while (flyZoneCheck(wp, waypoint_clearance) == false)
		{
			wp.N = rg.randLin()*(maxNorth - minNorth) + minNorth;
			wp.E = rg.randLin()*(maxEast - minEast) + minEast;
			if (is3D)
				wp.D = (rg.randLin()*((maxFlyHeight - waypoint_clearance) - (minFlyHeight + waypoint_clearance)) + minFlyHeight + waypoint_clearance)*-1.0; // Put the MSL into down (make it negative)
			else
				wp.D = 0;
		}
		//Push the waypoint into the terrain map
		map.wps.push_back(wp);
	}
}
mapper::~mapper()
{
	// Free the vector memory... These were causing memory leakages
	// Create a new empty vector and then swap it into the vector with data
	for (unsigned int i = 0; i < lineMinMax.size(); i++)
		std::vector<double>().swap(lineMinMax[i]);
	std::vector<std::vector<double> >().swap(lineMinMax);
	for (unsigned int i = 0; i < line_Mandb.size(); i++)
		std::vector<double>().swap(line_Mandb[i]);
	std::vector<std::vector<double> >().swap(line_Mandb);
}
bool mapper::flyZoneCheck(const cyl_s cyl)								// Returns true if the cylinder within boundaries and not on an obstacle, returns false otherwise
{
	NED_s NED;
	NED.N = cyl.N;
	NED.E = cyl.E;
	NED.D = 0;
	double radius = cyl.R;
	return  flyZoneCheckMASTER(NED, radius);
}
bool mapper::flyZoneCheck(const NED_s NED, const double radius)			// Returns true if it is within boundaries and not on an obstacle, returns false otherwise
{
	return flyZoneCheckMASTER(NED, radius);
}
bool mapper::flyZoneCheckMASTER(const NED_s NED, const double radius)	// This function sees if the point NED, is at least radius away from any obstacle
{
	// First, Check Within the Boundaries
	bool withinBoundaries;
	// Look at the Point in Polygon Algorithm
	// Focus on rays South.
	std::vector<bool> lineConcerns;
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

	// Second, Check for Cylinders
	// Check if the point falls into the volume of the cylinder
	for (unsigned int i = 0; i < map.cylinders.size(); i++)
		if (sqrt(pow(NED.N - map.cylinders[i].N, 2) + pow(NED.E - map.cylinders[i].E, 2)) < map.cylinders[i].R + radius && -NED.D - radius < map.cylinders[i].H)
			return false;
	return true; // The coordinate is in the safe zone if it got to here!
}
} // end namespace theseus
