#ifndef _VIRTUAL_FRAME_BROADCASTER_H_
#define _VIRTUAL_FRAME_BROADCASTER_H_

#include <ros/ros.h>
#include <list>
#include <Eigen/Core>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>



namespace virtual_frame_broadcaster
{

class VirtualFrameBroadcaster
{

  public:
  
    VirtualFrameBroadcaster(const ros::NodeHandle &nh, tf2_ros::TransformBroadcaster &br);

    ~VirtualFrameBroadcaster();

    void spinNode();

    ros::Subscriber sub;
  
  private:
  
    std::string _parent_frame, _child_frame;
    int _delayed_step;
    double _dt;
    ros::Time timestamp_last_cmdvel = ros::Time::now();
    std::list<geometry_msgs::Twist>* _buffer;
    std::list<double>* _buffer_timestamp;

    ros::NodeHandle _nh;
    tf2_ros::TransformBroadcaster _br;

    void broadcastVTF();
  
    void cmdCallback(const geometry_msgs::Twist& msg);
  
    void RK4(Eigen::Vector3d& x, const double& v, const double& omega);
  
    Eigen::Vector3d unicycle(const Eigen::Vector3d& x, const double& v, const double& omega);

    Eigen::Vector3d ackermann(const Eigen::Vector3d& x, const double& v, const double& steer_rad);

}; // class VirtualFrameBroadcaster

} // namespace virtual_frame_broadcaster

#endif
