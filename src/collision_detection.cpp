#include <theseus/collsion_detection.h>


namespace theseus
{
CollisionDetection::CollisionDetection()
{

}
CollisionDetection::~CollisionDetection()
{
  for (unsigned int i = 0; i < lineMinMax_.size(); i++)
    std::vector<double>().swap(lineMinMax_[i]);
  std::vector<std::vector<double> >().swap(lineMinMax_);
  for (unsigned int i = 0; i < line_Mandb_.size(); i++)
    std::vector<double>().swap(line_Mandb_[i]);
  std::vector<std::vector<double> >().swap(line_Mandb_);
}
bool CollisionDetection::collisionFillet(NED_s w_im1, NED_s w_i, NED_s w_ip1, float clearance)
{

}
bool CollisionDetection::collisionPoint(NED_s point, float clearance)
{

}
bool CollisionDetection::collisionLine(NED_s point_s, NED_s point_e, float clearance)
{

}
bool CollisionDetection::collisionArc(NED_s)
{

}
bool CollisionDetection::collisionClimbAngle(NED_s point_s, NED_s point_e)
{

}
FilletProperties CollisionDetection::grabFilletProperties(NED_s w_im1, NED_s w_i, NED_s w_ip1)
{

}
void CollisionDetection::newMap(map_s map_in)
{
  for (unsigned int i = 0; i < lineMinMax_.size(); i++)
    std::vector<double>().swap(lineMinMax_[i]);
  std::vector<std::vector<double> >().swap(lineMinMax_);
  for (unsigned int i = 0; i < line_Mandb_.size(); i++)
    std::vector<double>().swap(line_Mandb_[i]);
  std::vector<std::vector<double> >().swap(line_Mandb_);
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
	clearance_    = input_file_->clearance;		  // Clearance for the path (m)
	minFlyHeight_ = input_file_->minFlyHeight;  // 30.48 m = 100 ft. // This still needs to add in the take off altitude
	maxFlyHeight_ = input_file_->maxFlyHeight;  // 228.6 m = 750 ft. // This still needs to add in the take off altitude
  iters_limit_  = input_file_->iters_limit;
  //vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv These lines are used to prep the flyZoneCheck() algorithm
	std::vector<double> NminNmaxEminEmax;       // Yeah, this is a riduculous name...
	std::vector<double> mb;                     // Vector of slope and intercepts
	nBPts_ = map_.boundary_pts.size();          // Number of Boundary Points
	double m, b, w, m_w;
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
