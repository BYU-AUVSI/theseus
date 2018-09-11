#include <theseus/groundstation.h>

namespace theseus
{
Groundstation::Groundstation() :
  nh_(ros::NodeHandle())
{
  //********************** PARAMETERS **********************//

  //************** SUBSCRIBERS AND PUBLISHERS **************//
  ground_pub_             = nh_.advertise<visualization_msgs::Marker>("groundstation/visualization_marker", 10);
  state_subscriber_       = nh_.subscribe("/state",100,&theseus::Groundstation::stateCallback, this);
  fstate_subscriber_      = nh_.subscribe("/fixedwing/state",100,&theseus::Groundstation::stateCallback, this);
  mobs_subscriber_        = nh_.subscribe("/moving_obstacles",100,&theseus::Groundstation::movingObsCallback, this);

  //***************** CALLBACKS AND TIMERS *****************//
  update_viz_timer_ = nh_.createWallTimer(ros::WallDuration(1.0/4.0), &Groundstation::updateViz, this);

  //********************** FUNCTIONS ***********************//
  recieved_state_ = false;

  odom_mkr_.header.frame_id    = "/local_ENU";
  odom_mkr_.ns                 = "plane_odom";
  odom_mkr_.type               = visualization_msgs::Marker::POINTS;
  odom_mkr_.action             = visualization_msgs::Marker::ADD;
  odom_mkr_.pose.orientation.x = 0.0;
  odom_mkr_.pose.orientation.y = 0.0;
  odom_mkr_.pose.orientation.z = 0.0;
  odom_mkr_.pose.orientation.w = 1.0;
  odom_mkr_.color.r            = 1.0f;
  odom_mkr_.color.g            = 1.0f;
  odom_mkr_.color.b            = 1.0f;
  odom_mkr_.color.a            = 1.0;
  odom_mkr_.lifetime           = ros::Duration();
  odom_mkr_.scale.x            = 2.5; //5.0; // point width
  odom_mkr_.scale.y            = 2.5; //5.0; // point width

  mobs_mkr_.header.frame_id    = "/local_ENU";
  mobs_mkr_.ns                 = "moving_obstacle";
  mobs_mkr_.type               = visualization_msgs::Marker::SPHERE;
  mobs_mkr_.action             = visualization_msgs::Marker::ADD;
  mobs_mkr_.pose.orientation.x = 0.0;
  mobs_mkr_.pose.orientation.y = 0.0;
  mobs_mkr_.pose.orientation.z = 0.0;
  mobs_mkr_.pose.orientation.w = 1.0;
  mobs_mkr_.color.r            = 1.0;
  mobs_mkr_.color.g            = 0.0;
  mobs_mkr_.color.b            = 0.0;
  mobs_mkr_.color.a            = 0.7;
  mobs_mkr_.lifetime           = ros::Duration();
  mobs_mkr_.id                 = 0;

  pose_mkr_.header.frame_id    = "/local_ENU";
  pose_mkr_.ns                 = "plane_odom_front";
  pose_mkr_.type               = visualization_msgs::Marker::MESH_RESOURCE;
  pose_mkr_.action             = visualization_msgs::Marker::ADD;
  pose_mkr_.pose.orientation.x = 0.0;
  pose_mkr_.pose.orientation.y = 0.0;
  pose_mkr_.pose.orientation.z = 0.0;
  pose_mkr_.pose.orientation.w = 1.0;
  pose_mkr_.color.r            = 1.0f;
  pose_mkr_.color.g            = 1.0f;
  pose_mkr_.color.b            = 1.0f;
  pose_mkr_.color.a            = 1.0;
  pose_mkr_.lifetime           = ros::Duration();
  pose_mkr_.scale.x            = 20.0; // diameter of the plane
  pose_mkr_.scale.y            = 20.0; // diameter of the plane
  pose_mkr_.scale.z            = 20.0; // diameter of the plane
  pose_mkr_.id                 = 1;
  pose_mkr_.mesh_resource      = "package://auvsi_mission/meshes/rotated_fixedwing.dae";

  nh_.param<double>("lat_ref", lat_ref_, 38.144692);
  nh_.param<double>("lon_ref", lon_ref_, -76.428007);
  nh_.param<double>("h_ref", h_ref_, 0.0);
  ROS_WARN("reference latitude: %f", lat_ref_);
  ROS_WARN("reference longitude: %f", lon_ref_);
  ROS_WARN("reference height: %f", h_ref_);
  ROS_INFO("REFERENCE POINT SET");

  gps_converter_.set_reference(lat_ref_, lon_ref_, h_ref_);
}
Groundstation::~Groundstation()
{

}
void Groundstation::movingObsCallback(const uav_msgs::MovingObstacleCollection &msg)
{
  NED_s mobs_pos;
  std::vector<NED_s> points;
  std::vector<float> radius;
  mobs_mkr_.header.stamp = ros::Time::now();
  for (int i = 0; i < msg.moving_obstacles.size(); i++)
  {
    radius.push_back(msg.moving_obstacles[i].sphere_radius);
    gps_converter_.gps2ned(msg.moving_obstacles[i].point.latitude, msg.moving_obstacles[i].point.longitude,\
                           msg.moving_obstacles[i].point.altitude, mobs_pos.N, mobs_pos.E, mobs_pos.D);
    points.push_back(mobs_pos);
    mobs_mkr_.scale.x         = radius[i]*2.0f;
    mobs_mkr_.scale.y         = radius[i]*2.0f;
    mobs_mkr_.scale.z         = radius[i]*2.0f;
    mobs_mkr_.pose.position.x = points[i].N; //points[i].E;
    mobs_mkr_.pose.position.y = -points[i].E; //points[i].N;
    mobs_mkr_.pose.position.z = -points[i].D;
    mobs_mkr_.id = i;
    ground_pub_.publish(mobs_mkr_);

    ros::Duration(0.01).sleep();
  }
}
void Groundstation::stateCallback(const rosplane_msgs::State &msg)
{
  // Extract position data from NED
  odometry_.position.x = msg.position[0];//msg.position[1];
  odometry_.position.y = -msg.position[1];//msg.position[0];
  odometry_.position.z = -msg.position[2];
  // odometry_.N = msg.position[0];
  // odometry_.E = msg.position[1];
  // odometry_.D = msg.position[2];

  // Extract orientation data from phi, theta, psi
  double phi = msg.phi;
  double theta = -msg.theta;
  double psi = -msg.psi;// + 1.570796;
  tf::Quaternion q = tf::createQuaternionFromRPY(phi, theta, psi);
  odometry_.orientation.x = q[0];
  odometry_.orientation.y = q[1];
  odometry_.orientation.z = q[2];
  odometry_.orientation.w = q[3];

  if (recieved_state_ == false)
  {
    recieved_state_ = true;
    // ending_point_ = odometry_;
    ending_chi_   = msg.chi;
    ROS_INFO("Initial state of the UAV recieved.");
  }
}
void Groundstation::updateViz(const ros::WallTimerEvent&)
{
  if (recieved_state_)
  {
    geometry_msgs::Point p;
    p.x = odometry_.position.x;
    p.y = odometry_.position.y;
    p.z = odometry_.position.z;
    // p.x =  odometry_.E;
    // p.y =  odometry_.N;
    // p.z = -odometry_.D;
    // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    odom_mkr_.header.stamp = ros::Time::now();
    odom_mkr_.points.push_back(p);
    ground_pub_.publish(odom_mkr_);
    ros::Duration(0.005).sleep();
    pose_mkr_.header.stamp = ros::Time::now();
    // pose_mkr_.pose.position.x = p.x;
    // pose_mkr_.pose.position.y = p.y;
    // pose_mkr_.pose.position.z = p.z;
    pose_mkr_.pose = odometry_;
    ros::Duration(0.005).sleep();
    ground_pub_.publish(pose_mkr_);
  }
}
} // end namespace theseus

//********************************************************//
//************************ MAIN **************************//
//********************************************************//
int main(int argc, char** argv)
{
  ros::init(argc, argv, "groundstation");
  theseus::Groundstation g;

  ros::spin();
  return 0;
} // end main
