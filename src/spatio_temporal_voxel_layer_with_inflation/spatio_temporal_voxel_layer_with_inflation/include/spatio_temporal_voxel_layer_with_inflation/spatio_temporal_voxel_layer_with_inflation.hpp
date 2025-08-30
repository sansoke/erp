/*********************************************************************
 *
 * Software License Agreement
 *
 *  Copyright (c) 2018, Simbe Robotics, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Simbe Robotics, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Steve Macenski (steven.macenski@simberobotics.com)
 * Purpose: Replace the ROS voxel grid / obstacle layers using VoxelGrid
 *          with OpenVvirtual inline unsigned char computeCost(double distance) const
  {
    unsigned char cost = 0;
    if (distance == 0)
      cost = LETHAL_OBSTACLE;
    else if (distance * resolution_ <= inscribed_radius_)
      cost = INSCRIBED_INFLATED_OBSTACLE;
    else
    {
      // make sure cost falls off by Euclidean distance
      double euclidean_distance = distance * resolution_;
      double factor = exp(-1.0 * weight_ * (euclidean_distance - inscribed_radius_));
      cost = (unsigned char)((INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
    }
    return cost;
  }DB's more efficient and capacble voxel library with
 *          ray tracing and knn.
 *********************************************************************/

#ifndef VOLUME_GRID_LAYER_H_
#define VOLUME_GRID_LAYER_H_

// voxel grid
#include <spatio_temporal_voxel_layer_with_inflation/spatio_temporal_voxel_grid.hpp>
#include <spatio_temporal_voxel_layer_with_inflation/SpatioTemporalVoxelLayerWithInflationConfig.h>
// ROS
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <dynamic_reconfigure/server.h>
// costmap
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/footprint.h>
#include <costmap_2d/costmap_math.h>
#include <boost/thread.hpp>
#include <pluginlib/class_list_macros.h>
// openVDB
#include <openvdb/openvdb.h>
// STL
#include <vector>
#include <string>
#include <iostream>
#include <time.h>
#include <limits>
#include <cmath>

// msgs
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Point.h>
#include <spatio_temporal_voxel_layer_with_inflation/SaveGrid.h>
#include <std_srvs/SetBool.h>
// projector
#include <laser_geometry/laser_geometry.h>
// tf
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/buffer_core.h>

namespace spatio_temporal_voxel_layer_with_inflation
{

// conveniences for line lengths
typedef std::vector<boost::shared_ptr<message_filters::SubscriberBase> >::iterator observation_subscribers_iter;
typedef std::vector<boost::shared_ptr<buffer::MeasurementBuffer> >::iterator observation_buffers_iter;
typedef spatio_temporal_voxel_layer_with_inflation::SpatioTemporalVoxelLayerWithInflationConfig dynamicReconfigureType;
typedef dynamic_reconfigure::Server<dynamicReconfigureType> dynamicReconfigureServerType;

/**
 * @class CellData
 * @brief Storage for cell information used during obstacle inflation
 */
class CellData
{
public:
  /**
   * @brief  Constructor for a CellData objects
   * @param  i The index of the cell in the cost map
   * @param  x The x coordinate of the cell in the cost map
   * @param  y The y coordinate of the cell in the cost map
   * @param  sx The x coordinate of the closest obstacle cell in the costmap
   * @param  sy The y coordinate of the closest obstacle cell in the costmap
   * @return
   */
  CellData(double i, unsigned int x, unsigned int y, unsigned int sx, unsigned int sy) :
      index_(i), x_(x), y_(y), src_x_(sx), src_y_(sy)
  {
  }
  unsigned int index_;
  unsigned int x_, y_;
  unsigned int src_x_, src_y_;
};


// Core ROS voxel layer class
class SpatioTemporalVoxelLayerWithInflation : public costmap_2d::CostmapLayer
{
public:
  SpatioTemporalVoxelLayerWithInflation(void);
  virtual ~SpatioTemporalVoxelLayerWithInflation(void);

  // Core Functions
  virtual void onInitialize(void);
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, \
                   double* min_x, double* min_y, double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, \
                                                               int max_i, int max_j);

  // Functions to interact with other layers
  virtual void matchSize(void);

  // Functions for layer high level operations
  virtual void reset(void);
  virtual void activate(void);
  virtual void deactivate(void);


  // Functions for sensor feeds
  bool GetMarkingObservations(std::vector<observation::MeasurementReading>& marking_observations) const;
  bool GetClearingObservations(std::vector<observation::MeasurementReading>& marking_observations) const;
  void ObservationsResetAfterReading() const;

  // Functions to interact with maps
  void UpdateROSCostmap(double* min_x, double* min_y, double* max_x, double* max_y, \
                        std::unordered_set<volume_grid::occupany_cell>& cleared_cells, double robot_yaw);
  bool updateFootprint(double robot_x, double robot_y, double robot_yaw, \
                       double* min_x, double* min_y, double* max_x, double* max_y);
  void ResetGrid(void);

  // Saving grids callback for openVDB
  bool SaveGridCallback(spatio_temporal_voxel_layer_with_inflation::SaveGrid::Request& req, \
                        spatio_temporal_voxel_layer_with_inflation::SaveGrid::Response& resp);

private:
  // Sensor callbacks
  void LaserScanCallback(const sensor_msgs::LaserScanConstPtr& message, \
                         const boost::shared_ptr<buffer::MeasurementBuffer>& buffer);
  void LaserScanValidInfCallback(const sensor_msgs::LaserScanConstPtr& raw_message, \
                                 const boost::shared_ptr<buffer::MeasurementBuffer>& buffer);
  void PointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& message, \
                          const boost::shared_ptr<buffer::MeasurementBuffer>& buffer);

  // Functions for adding static obstacle zones
  bool AddStaticObservations(const observation::MeasurementReading& obs);
  bool RemoveStaticObservations(void);

  // Dynamic reconfigure
  void DynamicReconfigureCallback(dynamicReconfigureType &config, uint32_t level);

  // Enable/Disable callback
  bool BufferEnablerCallback( std_srvs::SetBool::Request & request,    \
                              std_srvs::SetBool::Response & response,  \
                              boost::shared_ptr<buffer::MeasurementBuffer>& buffer, \
                              boost::shared_ptr<message_filters::SubscriberBase>& subcriber);

  void setInflationParameters(double inflation_radius, double cost_scaling_factor);
  void computeCaches();
  void deleteKernels();
  unsigned int cellDistance(double world_dist)
  {
    return layered_costmap_->getCostmap()->cellDistance(world_dist);
  }
  inline double distanceLookup(int mx, int my, int src_x, int src_y)
  {
    unsigned int dx = abs(mx - src_x);
    unsigned int dy = abs(my - src_y);
    return cached_distances_[dx][dy];
  }
  inline void enqueue(unsigned int index, unsigned int mx, unsigned int my,
                      unsigned int src_x, unsigned int src_y, double robot_yaw);

  inline unsigned char costLookup(int mx, int my, int src_x, int src_y)
  {
    unsigned int dx = abs(mx - src_x);
    unsigned int dy = abs(my - src_y);
    return cached_costs_[dx][dy];
  }                      
  virtual inline unsigned char computeCost(double distance) const
  {
    unsigned char cost = 0;
    if (distance == 0)
      cost = costmap_2d::LETHAL_OBSTACLE;
    else if (distance * resolution_ <= inscribed_radius_)
      cost = costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
    else
    {
      // make sure cost falls off by Euclidean distance
      double euclidean_distance = distance * resolution_;
      double factor = exp(-1.0 * weight_ * (euclidean_distance - inscribed_radius_));
      cost = (unsigned char)((costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
    }
    return cost;
  }

  laser_geometry::LaserProjection                                  _laser_projector;
  std::vector<boost::shared_ptr<message_filters::SubscriberBase> > _observation_subscribers;
  std::vector<boost::shared_ptr<tf2_ros::MessageFilterBase> >           _observation_notifiers;
  std::vector<boost::shared_ptr<buffer::MeasurementBuffer> >       _observation_buffers;
  std::vector<boost::shared_ptr<buffer::MeasurementBuffer> >       _marking_buffers;
  std::vector<boost::shared_ptr<buffer::MeasurementBuffer> >       _clearing_buffers;
  std::vector<ros::ServiceServer>                                  _buffer_enabler_servers;
  dynamicReconfigureServerType*                                    _dynamic_reconfigure_server;

  bool                                 _publish_voxels, _mapping_mode;
  ros::Publisher                       _voxel_pub;
  ros::ServiceServer                   _grid_saver;
  ros::Duration                        _map_save_duration;
  ros::Time                            _last_map_save_time;
  std::string                          _global_frame;
  double                               _voxel_size, _voxel_decay;
  int                                  _combination_method, _mark_threshold;
  volume_grid::GlobalDecayModel        _decay_model;
  bool                                 _update_footprint_enabled, _enabled;
  std::vector<geometry_msgs::Point>    _transformed_footprint;
  std::vector<observation::MeasurementReading> _static_observations;
  volume_grid::SpatioTemporalVoxelGrid*        _voxel_grid;
  boost::recursive_mutex                       _voxel_grid_lock;
  double resolution_;
  double inflation_radius_;
  double inscribed_radius_;
  double weight_;
  unsigned int cell_inflation_radius_;
  unsigned int cached_cell_inflation_radius_;
  bool need_reinflation_;
  unsigned char** cached_costs_;
  bool* seen_;
  int seen_size_;
  std::map<double, std::vector<CellData> > inflation_cells_;
  bool inflate_unknown_;
  boost::recursive_mutex* inflation_access_;
  double** cached_distances_;
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
  

  

};

}; // end namespace
#endif
