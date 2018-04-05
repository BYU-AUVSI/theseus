#ifndef GPS_STRUCT_H
#define GPS_STRUCT_H

#include <math.h>

// #include <stdio.h>

namespace theseus
{
  struct gps_struct
  {
    //*********************** VARIABLES **********************//
  private:
    double piD180_;   // pi/180
    double a_;        // length of Earth's semi-major axis in meters
    double b_;        // length of Earth's semi-minor axis in meters
    double e2_;       // e = first numerical eccentricity, e2_ = e^2
    double R_[3][3];  // rotational matrix from ECEF to NED
    double xr_;       // ECEF x coordinate of reference point
    double yr_;       // ECEF y coordinate of reference point
    double zr_;       // ECEF z coordinate of reference point
    double epsilon_;  // used in ned2gps

  public:
    gps_struct()
    {
      piD180_         = M_PI/180.0;
      a_              = 6378137.0;
      b_              = 6356752.3142;
      e2_             = 1.0 - pow((b_/a_), 2.0);
    }
    //****************** OPERATOR FUNCTIONS ******************//

    //********************** FUNCTIONS ***********************//
    void set_reference(double r_lat_deg, double r_lon_deg, double r_height_m)
    {
      double r_phi    = r_lat_deg*piD180_;     // reference latitude  (N)
      double r_lambda = r_lon_deg*piD180_;     // reference longitude (E)
      double r_height = r_height_m;            // reference height in meters
      double nu0      = a_/sqrt(1.0 - e2_*sin(r_phi)*sin(r_phi));
      double s_r_phi  = sin(r_phi);
      double c_r_phi  = cos(r_phi);
      double s_r_lam  = sin(r_lambda);
      double c_r_lam  = cos(r_lambda);
      R_[0][0]        = -s_r_phi*c_r_lam;
      R_[0][1]        = -s_r_phi*s_r_lam;
      R_[0][2]        =  c_r_phi;
      R_[1][0]        = -s_r_lam;
      R_[1][1]        =  c_r_lam;
      R_[1][2]        =  0.0;
      R_[2][0]        = -c_r_phi*c_r_lam;
      R_[2][1]        = -c_r_phi*s_r_lam;
      R_[2][2]        = -s_r_phi;
      xr_             = (nu0 + r_height)*c_r_phi*c_r_lam;
      yr_             = (nu0 + r_height)*c_r_phi*s_r_lam;
      zr_             = (nu0*(1.0 - e2_) + r_height)*s_r_phi;
      epsilon_        = e2_/(1.0 - e2_);
    }
    void gps2ned(double lat_N, double lon_E, double h_M, double& N, double& E, double& D)
    {
      double phi      = lat_N*piD180_;
      double lambda   = lon_E*piD180_;

      // Convert the angles into Earth Centered Earth Fixed Reference Frame
      double s_phi    = sin(phi);
      double c_phi    = cos(phi);
      double s_lam    = sin(lambda);
      double c_lam    = cos(lambda);
      double nu       = a_/sqrt(1.0 - e2_*s_phi*s_phi);
      double x        = (nu + h_M)*c_phi*c_lam;
      double y        = (nu + h_M)*c_phi*s_lam;
      double z        = (nu*(1.0 - e2_) + h_M)*s_phi;

      // Find the difference between the point x, y, z to the reference point in ECEF
      double dx       = x - xr_;
      double dy       = y - yr_;
      double dz       = z - zr_;

      N               = R_[0][0]*dx + R_[0][1]*dy + R_[0][2]*dz;
      E               = R_[1][0]*dx + R_[1][1]*dy + R_[1][2]*dz;
      D               = R_[2][0]*dx + R_[2][1]*dy + R_[2][2]*dz;
      // printf("north: %f east %f down %f\n", N, E, D);
    }
    void ned2gps(double N, double E, double D, double& lat_N, double& lon_E, double& h_M)
    {
      // Convert from NED to ECEF
      double dx       = R_[0][0]*N + R_[1][0]*E + R_[2][0]*D;
      double dy       = R_[0][1]*N + R_[1][1]*E + R_[2][1]*D;
      double dz       = R_[0][2]*N + R_[1][2]*E + R_[2][2]*D;
      double x        = dx + xr_;
      double y        = dy + yr_;
      double z        = dz + zr_;

      // Convert from ECEF to GPS
      double p        = sqrt(x*x + y*y);
      double q        = atan2(z*a_, p*b_);
      double phi      = atan2(z + epsilon_*b_*pow(sin(q),3.0),p - e2_*a_*pow(cos(q),3.0));
      double lambda   = atan2(y,x);
      double nu       = a_/sqrt(1.0 - e2_*sin(phi)*sin(phi));

      h_M             = p/cos(phi) - nu;
      lat_N           = phi/piD180_;
      lon_E           = lambda/piD180_;
    }
  };
} // end namespace theseus
#endif // GPS_STRUCT_H
