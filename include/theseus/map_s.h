/*	DESCRIPTION:
 *	This is a header that defines a struct that contains all of the static
 *	information about the competition environment. It contains the boundaries
 *	and the static obstacles (cylinders). Also, the NED_s is really helpful
 *	and is used almost everywhere.
 *
 */
#ifndef MAP_H
#define MAP_H

#include <vector>

namespace theseus
{
struct NED_s
{
	double N;						// North (m)
	double E;						// East  (m)
	double D;						// Down  (m), Remember up is negative!
	bool operator==(const NED_s s)
	{
		return N == s.N && E == s.E && D == s.D;
	}
	bool operator!=(const NED_s s)
	{
		return N != s.N || E != s.E || D != s.D;
	}
};
struct cyl_s
{
	double N;						// North  (m)
	double E;						// East   (m)
	double R;						// Radius (m)
	double H;						// Height (m)
};
struct map_s
{
	std::vector<NED_s> boundary_pts;  // Contains all boundary points, no repeats
	std::vector<cyl_s> cylinders;     // Contains all cylinder obstacles
	std::vector<NED_s> wps;           // Primary Waypoints
};
}

#endif
