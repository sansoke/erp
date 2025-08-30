#include <virtual_frame_broadcaster/virtual_frame_broadcaster.h>



namespace virtual_frame_broadcaster
{

VirtualFrameBroadcaster::VirtualFrameBroadcaster(const ros::NodeHandle &nh, tf2_ros::TransformBroadcaster &br): _nh(nh), _br(br)
{
  std::string cmd_vel_topic;
  _nh.param("parent_frame", _parent_frame, std::string("base_link"));
  _nh.param("child_frame",  _child_frame,  std::string("virtual_base_link"));
  _nh.param("cmd_vel_topic", cmd_vel_topic,  std::string("/cmd_vel"));
  _nh.param("dt", _dt, 0.05);
  _nh.param("delayed_step", _delayed_step, 6);

  sub = _nh.subscribe(cmd_vel_topic, 1, &VirtualFrameBroadcaster::cmdCallback, this);
  _buffer = new std::list<geometry_msgs::Twist>();
  _buffer_timestamp = new std::list<double>();
}

VirtualFrameBroadcaster::~VirtualFrameBroadcaster()
{
  delete _buffer;
}

void VirtualFrameBroadcaster::spinNode() {
  ros::Rate rate(50);
  while (_nh.ok()) {
    // Update cmdvel Buffer
    ros::spinOnce();

    // Broadcast virtual frame tf
    broadcastVTF();
    
    rate.sleep();
  }
}

void VirtualFrameBroadcaster::broadcastVTF() {

  geometry_msgs::TransformStamped transform_stamped;

  transform_stamped.header.stamp = ros::Time::now();
  transform_stamped.header.frame_id = _parent_frame;
  transform_stamped.child_frame_id = _child_frame;

  // Check if no cmdvel input
  if (transform_stamped.header.stamp.toSec() - 2*_dt > timestamp_last_cmdvel.toSec()) {
    // Update buffer with 0 velocity msg
    geometry_msgs::Twist stop_msg;
    stop_msg.linear.x = 0;
    stop_msg.linear.y = 0;
    stop_msg.linear.z = 0;
    stop_msg.angular.x = 0;
    stop_msg.angular.y = 0;
    stop_msg.angular.z = 0;

    _buffer->push_back(stop_msg);
    if (_buffer->size() > _delayed_step)
    {
      _buffer->pop_front();
    }
  }

  // Predict future pose of ego vehicle
  Eigen::Vector3d pose;
  pose.setZero();
  std::list<geometry_msgs::Twist>::iterator it = _buffer->begin();
  for (it; it != _buffer->end(); it++)
  {
    RK4(pose, it->linear.x, it->angular.z);
  }

  // Update & broadcast virtual frame tf
  transform_stamped.transform.translation.x = pose(0);
  transform_stamped.transform.translation.y = pose(1);
  transform_stamped.transform.translation.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, pose(2));
  transform_stamped.transform.rotation.x = q.x();
  transform_stamped.transform.rotation.y = q.y();
  transform_stamped.transform.rotation.z = q.z();
  transform_stamped.transform.rotation.w = q.w();

  _br.sendTransform(transform_stamped);
}

void VirtualFrameBroadcaster::cmdCallback(const geometry_msgs::Twist& msg){
  // Update cmdvel msg buffer
  _buffer->push_back(msg);
  if (_buffer->size() > _delayed_step)
  {
    _buffer->pop_front();
  }

  // Update timestamp cmdvel msg came in
  timestamp_last_cmdvel = ros::Time::now();
}

void VirtualFrameBroadcaster::RK4(Eigen::Vector3d& x, const double& v, const double& omega)
{
  Eigen::Vector3d k1 = unicycle(x, v, omega);
  Eigen::Vector3d k2 = unicycle(x + k1 * _dt / 2.0, v, omega);
  Eigen::Vector3d k3 = unicycle(x + k2 * _dt / 2.0, v, omega);
  Eigen::Vector3d k4 = unicycle(x + k3 * _dt, v, omega);
  x += (k1 + 2.0 * k2 + 2.0 * k3 + k4) * _dt / 6.0;
}

Eigen::Vector3d VirtualFrameBroadcaster::unicycle(const Eigen::Vector3d& x, const double& v, const double& omega)
{
  Eigen::Vector3d dx;
  dx << v * cos(x(2)), v * sin(x(2)), omega;
  return dx;
}

Eigen::Vector3d VirtualFrameBroadcaster::ackermann(const Eigen::Vector3d& x, const double& v, const double& steer_rad)
{
  Eigen::Vector3d dx;
  double vx = 0;
  double vy = 0;
  double heading = 0;

  const double WHEEL_BASE = 1.212;
  

  // No codes
  double delta_theta = (( (v / WHEEL_BASE) * tan(steer_rad) ) * _dt);


  // update RK
  dx << vx, vy, heading;
  return dx;
}

} // namespace virtual_frame_broadcaster
