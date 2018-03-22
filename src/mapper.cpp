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
	is3D = input_file->is3D;							// This make the board 3D, which pretty much just means the cylinders have a specific height and waypoints can be above them.

														// Set up some competition constants
	minCylRadius = input_file->minCylRadius; // 9.144 m = 30 ft.
	maxCylRadius = input_file->maxCylRadius; // 91.44 m = 300 ft.
	minCylHeight = input_file->minCylHeight; // 9.144 m = 30 ft.
	maxCylHeight = input_file->maxCylHeight; // 228.6 m = 750 ft.
	minFlyHeight = input_file->minFlyHeight; // 23.7744 m = 78 ft.
	maxFlyHeight = input_file->maxFlyHeight; // 221.8944 m = 728 ft.

	// Pull in the competition boundaries
	// Get the NED coordinate frame set up

	// Get the Reference Angles
	double piD180 = 3.1415926535897932 / 180.0;
	double a = 6378137.0;						// length of Earth�s semi-major axis in meters
	double be = 6356752.3142;					// length of Earth�s semi-minor axis in meters
	double e2 = 1. - pow((be / a), 2);			// first numerical eccentricity

	rPhi = (strtod(input_file->latitude0.substr(1, 2).c_str(), NULL) + strtod((input_file->latitude0.substr(4, 2)).c_str(),NULL) / 60.0 + strtod(input_file->latitude0.substr(7, 5).c_str(), NULL) / 3600.0)*piD180;
	rLam = (strtod(input_file->longitude0.substr(2, 3).c_str(), NULL) + strtod(input_file->longitude0.substr(5, 2).c_str(),NULL) / 60.0 + strtod(input_file->longitude0.substr(8, 5).c_str(), NULL) / 3600.0)*piD180;
	rH = input_file->height0;

	// Convert the angles into Earth Centered Earth Fixed Reference Frame
	double chi = sqrt(1 - e2*sin(rPhi)*sin(rPhi));
	xr = (a / chi + rH)* cos(rPhi)*cos(rLam);
	yr = (a / chi + rH)* cos(rPhi)*sin(rLam);
	zr = (a*(1 - e2) / chi + rH)*sin(rPhi);


	std::ifstream boundaries_in_file;				// The file that recieves boundaries
	boundaries_in_file.open(input_file->boundaries_in_file.c_str());
	if (!boundaries_in_file)
    ROS_ERROR("Could not open the boundaries file.");
	NED_s boundary_point;
	bool setFirstValues = true;
	std::string LATITUDE;	// North
	std::string LONGITUDE;	// West
	double phi, lambda;
	while (boundaries_in_file.eof() == false)
	{
		boundaries_in_file >> LATITUDE >> LONGITUDE;
		phi = strtod(LATITUDE.substr(1, 2).c_str(), NULL) + strtod(LATITUDE.substr(4, 2).c_str(), NULL) / 60.0 + strtod(LATITUDE.substr(7, 5).c_str(), NULL) / 3600.0;
		lambda = strtod(LONGITUDE.substr(2, 3).c_str(), NULL) + strtod(LONGITUDE.substr(5, 2).c_str(), NULL) / 60.0 + strtod(LONGITUDE.substr(8, 5).c_str(), NULL) / 3600.0;

		boundary_point = GPS2NED(phi, lambda, rH);

		if (setFirstValues == false)
		{
			maxNorth = (boundary_point.N > maxNorth) ? boundary_point.N : maxNorth; // if new N is greater than maxN, set maxN = new N
			minNorth = (boundary_point.N < minNorth) ? boundary_point.N : minNorth;
			maxEast = (boundary_point.E > maxEast) ? boundary_point.E : maxEast;
			minEast = (boundary_point.E < minEast) ? boundary_point.E : minEast;
		}
		else
		{
			maxNorth = boundary_point.N;
			minNorth = boundary_point.N;
			maxEast = boundary_point.E;
			minEast = boundary_point.E;
			setFirstValues = false;
		}

		map.boundary_pts.push_back(boundary_point);	// This line puts the boundary points into the map member.
	}
	boundaries_in_file.close();

	// Set up flyZoneCheck()
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
		while (flyZoneCheck(cyl) == false || sqrt(pow(input_file->N0 - cyl.N, 2.0) + pow(input_file->E0 - cyl.E, 2.0)) < cyl.R + input_file->waypoint_clearance)
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
	int numWps = input_file->numWps;												// This is the number of Primary Waypoints
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
NED_s mapper::GPS2NED_(double phi, double lambda, double h) // Supposedly this one is about 3 times faster... It may be a little less acurate, it uses a taylor series approximation.
{
	// Constants
	double a = 6378137.0;				// length of Earth's semi-major axis in meters
	double b = 6356752.3142;			// length of Earth's semi-minor axis in meters
	double e2 = 1. - pow((b / a), 2);	// first numerical eccentricity
	double chi = sqrt(1 - e2*sin(rPhi)*sin(rPhi));

	// Delta Angles
	double dphi = rPhi - phi*3.1415926535897932 / 180.0;
	double dlam = rLam - lambda*3.1415926535897932 / 180.0;
	double dh = rH - h;

	// Taylor Series Approximation and Rotation
	NED_s ned;
	ned.N = -(a*(1. - e2) / pow(chi, 3) + h)*dphi - 1.5*a*cos(rPhi)*sin(rPhi)*e2*pow(dphi, 2) - dh*dphi*pow(sin(rPhi), 2) - .5*sin(rPhi)*cos(rPhi)*(a / chi + rH)*pow(dlam, 2);
	ned.E = (a / chi + rH)*cos(rPhi)*dlam - (a*(1 - e2) / pow(chi, 3) + rH)*sin(rPhi)*dphi*dlam + cos(rPhi)*dlam*dh;
	ned.D = -dh + .5*a*(1. - 1.5*e2*pow(cos(rPhi), 2) + 0.5*e2 + h / a)*pow(dphi, 2) + 0.5*(a*cos(rPhi)*cos(rPhi) / chi - rH*cos(rPhi)*cos(rPhi))*pow(dlam, 2);
	return ned;
}
NED_s mapper::GPS2NED(double phi, double lambda, double h)
{
	// send in phi (latitude) as an angle in degrees ex.38.14626
	// send in lambda (longitude) as an angle in degrees ex. 76.42816
	// both of those are converted into radians
	// send in h as a altitude (mean sea level) provo = about 1500, maryland = 6.7056

	// rLam is the reference longitude (in RADIANS)
	// rPhi is the reference latitude (in RADIANS)
	// The reference angles define where the 0,0,0 point is in the local NED coordinates

	// CONSTANTS
	double piD180 = 3.1415926535897932 / 180.0;
	double a = 6378137.0;						// length of Earth's semi-major axis in meters
	double b = 6356752.3142;					// length of Earth's semi-minor axis in meters
	double e2 = 1. - pow((b / a), 2);			// first numerical eccentricity
	double chi = sqrt(1 - e2*sin(rPhi)*sin(rPhi));

	// Convert the incoming angles to radians
	phi = phi*piD180;
	lambda = lambda*piD180;

	// Convert the angles into Earth Centered Earth Fixed Reference Frame
	double x = (a / chi + h)* cos(phi)*cos(lambda);
	double y = (a / chi + h)* cos(phi)*sin(lambda);
	double z = (a*(1 - e2) / chi + h)*sin(phi);

	// Find the difference between the point x, y, z to the reference point in ECEF
	double dx = x - xr;
	double dy = y - yr;
	double dz = z - zr;

	// Rotate the point in ECEF to the Local NED
	NED_s ned;
	ned.N = (-sin(rPhi)*cos(rLam)*dx) + (-sin(rPhi)*sin(rLam)*dy) +    cos(rPhi)*dz;
	ned.E =            (sin(rLam)*dx) -            (cos(rLam)*dy)                  ;
	ned.D = (-cos(rPhi)*cos(rLam)*dx) + (-cos(rPhi)*sin(rLam)*dy) + (-sin(rPhi)*dz);
	return ned;
}
} // end namespace theseus
