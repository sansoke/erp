#include <ros/ros.h>
#include <virtual_frame_broadcaster/virtual_frame_broadcaster.h>

int main(int argc, char** argv){

  ros::init(argc, argv, "virtual_frame_broadcaster");
  ros::NodeHandle nh("~");
  tf2_ros::TransformBroadcaster br;
  virtual_frame_broadcaster::VirtualFrameBroadcaster vfb(nh, br);
  vfb.spinNode();

  return(0);
}
