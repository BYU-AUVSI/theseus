#ifndef COLLISION_DETECTION_H
#define COLLISION_DETECTION_H

#include <vector>
#include <algorithm>

#include <theseus/map_s.h>
#include <theseus/fillet_s.h>
#include <theseus/param_reader.h>

namespace theseus
{
  class CollisionDetection
  {
  public:
    CollisionDetection();
    ~CollisionDetection();
    bool checkFillet(NED_s w_im1, NED_s  w_i, NED_s w_ip1, float clearance);
    bool checkFillet(fillet_s fil);
    bool checkPoint(NED_s point, float clearance);
    bool checkLine(NED_s point_s, NED_s point_e, float clearance);
    bool checkAfterWP(NED_s p, float chi);
    void newMap(map_s map_in);

    float maxNorth_;                              // Maximum North coordinate inside the boundaries
    float minNorth_;                              // Minimum North coordinate inside the boundaries
    float maxEast_;                               // Maximum East  coordinate inside the boundaries
    float minEast_;                               // Minimum East  coordinate inside the boundaries
    float minFlyHeight_;                          // Minimum Fly Height (positive value)
    float maxFlyHeight_;                          // Maximum Fly Height (positive value)

  private:
    map_s map_;
    ParamReader input_file_;

    // Map variables
    std::vector<std::vector<float> > lineMinMax_; // (N x 4) vector containing the (min N, max N, min E, max E) for each boundary line
  	std::vector<std::vector<float> > line_Mandb_; // (N x 4) vector that contains the slope and intercept of the line (m, b, (-1/m), (m + 1/m)) from N = m*E + b ... not sure about E = constant lines yet.
    unsigned int nBPts_;                          // Number of boundary points

    bool checkArc(NED_s ps, NED_s pe, float R, NED_s cp, int lambda, float clearance);
    bool checkClimbAngle(NED_s point_s, NED_s point_e);

    bool lineAndPoint2d(NED_s ls, NED_s le, float MinMax[], float Mandb[], NED_s p, float r);
    bool lineIntersectsArc(float Ni, float Ei, NED_s cp, NED_s ps, NED_s pe, bool ccw);

    // Debug print functions
    void printBoundsObstacles();
  };
} // end namespace theseus
