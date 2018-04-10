#ifndef COLLISION_DETECTION_H
#define COLLISION_DETECTION_H

#include <vector>
#include <theseus/map_s.h>
#include <theseus/param_reader.h>

namespace theseus
{
  class CollisionDetection
  {
  public:
    CollisionDetection();
    ~CollisionDetection();
    bool CollisionDetection::collisionFillet(NED_s w_im1, NED_s  w_i, NED_s w_ip1, float clearance);
    bool CollisionDetection::collisionPoint(NED_s point, float clearance);
    void newMap(map_s map_in);
  private:
    map_s map_;
    ParamReader input_file_;

    // Map variables
    std::vector<std::vector<double> > lineMinMax_; // (N x 4) vector containing the (min N, max N, min E, max E) for each boundary line
  	std::vector<std::vector<double> > line_Mandb_; // (N x 4) vector that contains the slope and intercept of the line (m, b, (-1/m), (m + 1/m)) from N = m*E + b ... not sure about E = constant lines yet.
    unsigned int nBPts_;                           // Number of boundary points
  	double maxNorth_;                              // Maximum North coordinate inside the boundaries
  	double minNorth_;                              // Minimum North coordinate inside the boundaries
  	double maxEast_;                               // Maximum East  coordinate inside the boundaries
  	double minEast_;                               // Minimum East  coordinate inside the boundaries
  	double minFlyHeight_;                          // Minimum Fly Height (positive value)
  	double maxFlyHeight_;                          // Maximum Fly Height (positive value)
    bool CollisionDetection::collisionLine(NED_s point_s, NED_s point_e, float clearance);
    bool CollisionDetection::collisionArc(NED_s);
    bool CollisionDetection::collisionClimbAngle(NED_s point_s, NED_s point_e);
    FilletProperties CollisionDetection::grabFilletProperties(NED_s w_im1, NED_s w_i, NED_s w_ip1);

    // Debug print functions
    void printBoundsObstacles();
  };
  struct FilletProperties
  {

  };
} // end namespace theseus
