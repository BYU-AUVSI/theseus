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
#include <math.h>

namespace theseus
{
struct NED_s
{
	double N;						// North (m)
	double E;						// East  (m)
	double D;						// Down  (m), Remember up is negative!
  NED_s()
  {
    N = 0.0f;
    E = 0.0f;
    D = 0.0f;
  }
  NED_s(float N_in, float E_in, float D_in)
  {
    N = N_in;
    E = E_in;
    D = D_in;
  }
	bool operator==(const NED_s s)
	{
		return N == s.N && E == s.E && D == s.D;
	}
	bool operator!=(const NED_s s)
	{
		return N != s.N || E != s.E || D != s.D;
	}
  NED_s operator+(const NED_s s)
  {
    NED_s n;
    n.N = N + s.N;
    n.E = E + s.E;
    n.D = D + s.D;
    return n;
  }
  NED_s operator-(const NED_s s)
  {
    NED_s n;
    n.N = N - s.N;
    n.E = E - s.E;
    n.D = D - s.D;
    return n;
  }
  float norm()
  {
    return sqrtf(N*N + E*E + D*D);
  }
  NED_s normalize()
  {
    NED_s out;
    float magnitude = norm();
    out.N = N/magnitude;
    out.E = E/magnitude;
    out.D = D/magnitude;
    return out;
  }
  float dot(NED_s in)
  {
    return N*in.N + E*in.E + D*in.D;
  }
  float getChi()
  {
    return atan2f(E, N);
  }
  NED_s operator*(const float num)
  {
    NED_s n;
    n.N = N*num;
    n.E = E*num;
    n.D = D*num;
    return n;
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
