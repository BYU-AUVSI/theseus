#ifndef FILLET_H
#define FILLET_H

#include <theseus/map_s.h>
#include <math.h>

namespace theseus
{
struct fillet_s
{
  NED_s w_im1;
  NED_s w_i;
  NED_s w_ip1;

	NED_s z1;
  NED_s z2;
  NED_s c;

  NED_s q_im1;
  NED_s q_i;
  int lambda;

  float R;

  bool calculate(NED_s w_im1, NED_s w_i, NED_s w_ip1, float R_in)
  {
    // calculates fillet variables and determines if it is possible
    // possible is defined as consisting of a straight line, an arc, and a straight line
    // ie it can't go backwards to start the arc
    // see Small Unmanned Aircraft: Theory and Practice (Beard and McLain) Algorithm 6

    R = R_in;
    q_im1 = (w_i   - w_im1).normalize();
    q_i   = (w_ip1 - w_i  ).normalize();
    float n_qim1_dot_qi = -q_im1.dot(q_i);
    float tolerance = 0.0001f;
    n_qim1_dot_qi   = n_qim1_dot_qi < -1.0f + tolerance ? -1.0f + tolerance : n_qim1_dot_qi; // this prevents beta from being nan
    n_qim1_dot_qi   = n_qim1_dot_qi >  1.0f - tolerance ?  1.0f - tolerance : n_qim1_dot_qi; // Still allows for a lot of degrees
    float varrho    = acosf(n_qim1_dot_qi);
    z1              = w_i - q_im1*(R/tanf(varrho/2.0f));
    z2              = w_i + q_i*(R/tanf(varrho/2.0f));
    c               = w_i - ((q_im1 - q_i).normalize())*(R/sinf(varrho/2.0f));
    lambda          = q_im1.N*q_i.E - q_im1.E*q_i.N > 0.0f ? 1 : -1;

    // check to see if this is possible
    if (q_im1.dot(z1 - w_im1) > 0.0f && (q_i*-1.0f).dot(z2 - w_ip1) > 0.0f)
      return true;
    else
      return false;
  }
  bool roomFor(fillet_s next_fillet)
  {
    // returns if two fillet paths right next to each other are compatable (no overlaping arcs)
    if ((w_ip1 - w_i).norm() >= (z2 - w_i).norm() + (w_ip1 - next_fillet.z1).norm())
      return true;
    else
      return false;
  }
};
}
#endif
