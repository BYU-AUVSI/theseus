#include <theseus/rotate_viz.h>

namespace theseus
{
RotateViz::RotateViz() :
  nh_(ros::NodeHandle())
{
  //********************** PARAMETERS **********************//

  //************** SUBSCRIBERS AND PUBLISHERS **************//
  odom_trans_.header.frame_id = "local_ENU";
  odom_trans_.child_frame_id = "base_link";
  initial_time_ = ros::Time::now();

  //***************** CALLBACKS AND TIMERS *****************//
  rotate_viz_timer_ = nh_.createWallTimer(ros::WallDuration(1.0/30.0), &RotateViz::rotateViz, this);
  ParamReader input_file;
  mapper myWorld(1, &input_file);
  plt.displayBoundaries(myWorld.map);

}
RotateViz::~RotateViz()
{

}
void RotateViz::rotateViz(const ros::WallTimerEvent&)
{
  odom_trans_.header.stamp = ros::Time::now();
  ros::Time new_time = ros::Time::now();
  ros::Duration time_step = new_time - initial_time_;
  float ts = time_step.toSec();
  odom_trans_.transform.translation.x =  200.0;
  odom_trans_.transform.translation.y =  300.0;
  odom_trans_.transform.translation.z =  -500.0;
  // Euler Angles in NED to Quaternion in NED to Quaternion in XYZ
  float qx,qy,qz,qw;
  float psi   = M_PI/20.0f*ts; // function of time
  float theta = 0.0f;
  float phi   = 0.0f;
  qx =   cosf(psi/2.0f)*sinf(theta/2.0f)*cosf(phi/2.0f)\
       + sinf(psi/2.0f)*cosf(theta/2.0f)*sinf(phi/2.0f);
  qy =   cosf(psi/2.0f)*cosf(theta/2.0f)*sinf(phi/2.0f)\
       - sinf(psi/2.0f)*sinf(theta/2.0f)*cosf(phi/2.0f);
  qz = - sinf(psi/2.0f)*cosf(theta/2.0f)*cosf(phi/2.0f)\
       + cosf(psi/2.0f)*sinf(theta/2.0f)*sinf(phi/2.0f);
  qw =   cosf(psi/2.0f)*cosf(theta/2.0f)*cosf(phi/2.0f)\
       + sinf(psi/2.0f)*sinf(theta/2.0f)*sinf(phi/2.0f);
  tf::Quaternion q(qx,qy,qz,qw);
  tf::quaternionTFToMsg(q,odom_trans_.transform.rotation);
  pose_broadcaster_.sendTransform(odom_trans_);
  plt.pingBoundaries();
}
} // end namespace theseus

//********************************************************//
//************************ MAIN **************************//
//********************************************************//
int main(int argc, char** argv)
{
  ros::init(argc, argv, "rotating_frame_node");
  theseus::RotateViz rot_obj;

  ros::spin();
  return 0;
} // end main
