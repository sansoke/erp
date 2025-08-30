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
 *********************************************************************/

#include "spatio_temporal_voxel_layer_with_inflation/spatio_temporal_voxel_layer_with_inflation.hpp"

namespace spatio_temporal_voxel_layer_with_inflation {

/*****************************************************************************/
SpatioTemporalVoxelLayerWithInflation::SpatioTemporalVoxelLayerWithInflation(void)
  : resolution_(0)
  , inflation_radius_(0)
  , inscribed_radius_(0)
  , weight_(0)
  , inflate_unknown_(false)
  , cell_inflation_radius_(0)
  , cached_cell_inflation_radius_(0)
  , seen_(NULL)
  , cached_costs_(NULL)
  , cached_distances_(NULL)
  , last_min_x_(-std::numeric_limits<float>::max())
  , last_min_y_(-std::numeric_limits<float>::max())
  , last_max_x_(std::numeric_limits<float>::max())
  , last_max_y_(std::numeric_limits<float>::max())
/*****************************************************************************/
{
  inflation_access_ = new boost::recursive_mutex();
}

/*****************************************************************************/
SpatioTemporalVoxelLayerWithInflation::~SpatioTemporalVoxelLayerWithInflation(void)
/*****************************************************************************/
{
  deleteKernels();
  if (seen_)
  {
    delete seen_;
  }
  if (_dynamic_reconfigure_server)
  {
    delete _dynamic_reconfigure_server;
  }
  if (_voxel_grid)
  {
    delete _voxel_grid;    
  }
}

/*****************************************************************************/
void SpatioTemporalVoxelLayerWithInflation::onInitialize(void)
/*****************************************************************************/
{
  ROS_INFO("%s being initialized as SpatioTemporalVoxelLayer with voxel layer!", \
           getName().c_str());
  
  // initialize parameters, grid, and sub/pubs
  ros::NodeHandle nh("~/" + name_), g_nh, prefix_nh;

  _global_frame = std::string(layered_costmap_->getGlobalFrameID());
  ROS_INFO("%s's global frame is %s.", \
                                    getName().c_str(), _global_frame.c_str());

  bool track_unknown_space;
  double transform_tolerance, map_save_time;
  std::string topics_string;
  int decay_model_int;
  // source names
  nh.param("observation_sources", topics_string, std::string(""));
  // timeout in seconds for transforms
  nh.param("transform_tolerance", transform_tolerance, 0.2);
  // whether to default on
  nh.param("enabled", _enabled, true);
  enabled_ = _enabled; // costmap_2d for some unexplicable reason uses globals
  // publish the voxel grid to visualize
  nh.param("publish_voxel_map", _publish_voxels, false);
  // size of each voxel in meters
  nh.param("voxel_size", _voxel_size, 0.05);
  // 1=takes highest in layers, 0=takes current layer
  nh.param("combination_method", _combination_method, 1);
  // number of voxels per vertical needed to have obstacle
  nh.param("mark_threshold", _mark_threshold, 0);
  // clear under robot footprint
  nh.param("update_footprint_enabled", _update_footprint_enabled, true);
  // keep tabs on unknown space
  nh.param("track_unknown_space", track_unknown_space, \
                                  layered_costmap_->isTrackingUnknown());
  nh.param("decay_model", decay_model_int, 0);
  _decay_model = static_cast<volume_grid::GlobalDecayModel>(decay_model_int);
  // decay param
  nh.param("voxel_decay", _voxel_decay, -1.);
  // whether to map or navigate
  nh.param("mapping_mode", _mapping_mode, false);
  // if mapping, how often to save a map for safety
  nh.param("map_save_duration", map_save_time, 60.);
  ROS_INFO("%s loaded parameters from parameter server.", getName().c_str());

  
  if (_mapping_mode)
  {
    _map_save_duration = ros::Duration(map_save_time);
    _last_map_save_time = ros::Time::now() - _map_save_duration;
  }

  if (track_unknown_space)
  {
    default_value_ = costmap_2d::NO_INFORMATION;
  }
  else
  {
    default_value_ = costmap_2d::FREE_SPACE;
  }

  _voxel_pub = nh.advertise<sensor_msgs::PointCloud2>("voxel_grid", 1);
  _grid_saver = nh.advertiseService("spatiotemporal_voxel_grid/save_grid", \
                                 &SpatioTemporalVoxelLayerWithInflation::SaveGridCallback, \
                                  this);

  _voxel_grid = new volume_grid::SpatioTemporalVoxelGrid(_voxel_size, \
                                                        (double)default_value_, \
                                                        _decay_model, \
                                                        _voxel_decay, \
                                                        _publish_voxels);
  matchSize();
  
  current_ = true;
  ROS_INFO("%s created underlying voxel grid.", getName().c_str());

  const std::string tf_prefix = tf::getPrefixParam(prefix_nh);
  std::stringstream ss(topics_string);
  std::string source;
  while (ss >> source)
  {
    ros::NodeHandle source_node(nh, source);

    // get the parameters for the specific topic
    double observation_keep_time, expected_update_rate, min_obstacle_height;
    double max_obstacle_height, min_z, max_z, vFOV, vFOVPadding;
    double hFOV, decay_acceleration;
    std::string topic, sensor_frame, data_type;
    bool inf_is_valid, clearing, marking, voxel_filter, clear_after_reading, enabled;
    int voxel_min_points;

    source_node.param("topic", topic, source);
    source_node.param("sensor_frame", sensor_frame, std::string(""));
    source_node.param("observation_persistence", observation_keep_time, 0.0);
    source_node.param("expected_update_rate", expected_update_rate, 0.0);
    source_node.param("data_type", data_type, std::string("PointCloud2"));
    source_node.param("min_obstacle_height", min_obstacle_height, 0.0);
    source_node.param("max_obstacle_height", max_obstacle_height, 3.0);
    source_node.param("inf_is_valid", inf_is_valid, false);
    source_node.param("clearing", clearing, false);
    source_node.param("marking", marking, true);
    // minimum distance from camera it can see
    source_node.param("min_z", min_z, 0.);
    // maximum distance from camera it can see
    source_node.param("max_z", max_z, 10.);
    // vertical FOV angle in rad
    source_node.param("vertical_fov_angle", vFOV, 0.7);
    // vertical FOV padding in meters (3D lidar frustum only)
    source_node.param("vertical_fov_padding", vFOVPadding, 0.0);
    // horizontal FOV angle in rad
    source_node.param("horizontal_fov_angle", hFOV, 1.04);
    // acceleration scales the model's decay in presence of readings
    source_node.param("decay_acceleration", decay_acceleration, 0.);
    // performs an approximate voxel filter over the data to reduce
    source_node.param("voxel_filter", voxel_filter, false);
    // minimum points per voxel for voxel filter
    source_node.param("voxel_min_points", voxel_min_points, 0);
    // clears measurement buffer after reading values from it
    source_node.param("clear_after_reading", clear_after_reading, false);
    // Whether the frustum is enabled on startup. Can be toggled with service
    source_node.param("enabled", enabled, true);
    // model type - default depth camera frustum model
    int model_type_int;
    source_node.param("model_type", model_type_int, 0);
    ModelType model_type = static_cast<ModelType>(model_type_int);

    if (!sensor_frame.empty())
    {
     sensor_frame = tf::resolve(tf_prefix, sensor_frame);
    }

    if (!(data_type == "PointCloud2" || data_type == "LaserScan"))
    {
      throw std::runtime_error( \
          "Only topics that use pointclouds or laser scans are supported.");
    }

    std::string obstacle_range_param_name;
    double obstacle_range = 3.0;
    if (source_node.searchParam("obstacle_range", obstacle_range_param_name))
    {
      source_node.getParam(obstacle_range_param_name, obstacle_range);
    }

    // create an observation buffer
    _observation_buffers.push_back(
        boost::shared_ptr <buffer::MeasurementBuffer>
        (new buffer::MeasurementBuffer(topic, observation_keep_time,      \
        expected_update_rate, min_obstacle_height, max_obstacle_height,   \
        obstacle_range, *tf_, _global_frame, sensor_frame,                \
        transform_tolerance, min_z, max_z, vFOV, vFOVPadding, hFOV,       \
        decay_acceleration, marking, clearing, _voxel_size,               \
        voxel_filter, voxel_min_points, enabled, clear_after_reading,     \
        model_type)));

    // Add buffer to marking observation buffers
    if (marking == true)
    {
      _marking_buffers.push_back(_observation_buffers.back());
    }

    // Add buffer to clearing observation buffers
    if (clearing == true)
    {
      _clearing_buffers.push_back(_observation_buffers.back());
    }

    // create a callback for the topic
    if (data_type == "LaserScan")
    {
      boost::shared_ptr < message_filters::Subscriber<sensor_msgs::LaserScan>
          > sub(new message_filters::Subscriber<sensor_msgs::LaserScan>(g_nh, \
                                                                   topic, 50));
      _observation_subscribers.push_back(sub);

      boost::shared_ptr < tf2_ros::MessageFilter<sensor_msgs::LaserScan>
          > filter(new tf2_ros::MessageFilter<sensor_msgs::LaserScan>(*sub, \
                                                     *tf_, _global_frame, 50,0));

      if (inf_is_valid)
      {
        filter->registerCallback(
            boost::bind(&SpatioTemporalVoxelLayerWithInflation::LaserScanValidInfCallback, \
                                        this, _1,_observation_buffers.back()));
      } else {
        filter->registerCallback(
            boost::bind(&SpatioTemporalVoxelLayerWithInflation::LaserScanCallback, \
                                        this, _1, _observation_buffers.back()));
      }

      _observation_subscribers.push_back(sub);
      _observation_notifiers.push_back(filter);

      _observation_notifiers.back()->setTolerance(ros::Duration(0.05));
    }

    else if (data_type == "PointCloud2")
    {
      boost::shared_ptr < message_filters::Subscriber<sensor_msgs::PointCloud2>
          > sub(new message_filters::Subscriber<sensor_msgs::PointCloud2>(g_nh, \
                                                                     topic, 50));
      _observation_subscribers.push_back(sub);

      boost::shared_ptr < tf2_ros::MessageFilter<sensor_msgs::PointCloud2>
          > filter(new tf2_ros::MessageFilter<sensor_msgs::PointCloud2>(*sub, \
                                                  *tf_, _global_frame, 50,0));
      filter->registerCallback(
          boost::bind(&SpatioTemporalVoxelLayerWithInflation::PointCloud2Callback, this, _1, \
                                                   _observation_buffers.back()));

      _observation_subscribers.push_back(sub);
      _observation_notifiers.push_back(filter);
    }

    ros::ServiceServer server;
    boost::function < bool(std_srvs::SetBool::Request&, \
                           std_srvs::SetBool::Response&) > serv_callback;

    serv_callback = boost::bind(&SpatioTemporalVoxelLayerWithInflation::BufferEnablerCallback, \
                                this, _1, _2, _observation_buffers.back(),  \
                                _observation_subscribers.back());

    std::string toggle_topic = source +  "/toggle_enabled";
    server = nh.advertiseService(toggle_topic, serv_callback);

    _buffer_enabler_servers.push_back(server);

    if (sensor_frame != "")
    {
      std::vector<std::string> target_frames;
      target_frames.reserve(2);
      target_frames.push_back(_global_frame);
      target_frames.push_back(sensor_frame);
      _observation_notifiers.back()->setTargetFrames(target_frames);
    }
  }
  

  // Dynamic reconfigure
  dynamic_reconfigure::Server<dynamicReconfigureType>::CallbackType f;
  f = boost::bind(&SpatioTemporalVoxelLayerWithInflation::DynamicReconfigureCallback, \
                                                                 this, _1, _2);
  _dynamic_reconfigure_server = new dynamicReconfigureServerType(nh);
  _dynamic_reconfigure_server->setCallback(f);

  need_reinflation_ = false;

  ROS_INFO("%s initialization complete!", getName().c_str());
}

/*****************************************************************************/
void SpatioTemporalVoxelLayerWithInflation::LaserScanCallback( \
                const sensor_msgs::LaserScanConstPtr& message, \
                const boost::shared_ptr<buffer::MeasurementBuffer>& buffer)
/*****************************************************************************/
{
  // laser scan where infinity is invalid callback function
  sensor_msgs::PointCloud2 cloud;
  cloud.header = message->header;
  try
  {
    _laser_projector.transformLaserScanToPointCloud(\
                             message->header.frame_id, *message, cloud, *tf_);
  } catch (tf::TransformException &ex) {
    ROS_WARN("TF returned a transform exception to frame %s: %s", \
            _global_frame.c_str(), ex.what());
    _laser_projector.projectLaser(*message, cloud);
  }
  // buffer the point cloud
  buffer->Lock();
  buffer->BufferROSCloud(cloud);
  buffer->Unlock();
}

/*****************************************************************************/
void SpatioTemporalVoxelLayerWithInflation::LaserScanValidInfCallback( \
                const sensor_msgs::LaserScanConstPtr& raw_message, \
                const boost::shared_ptr<buffer::MeasurementBuffer>& buffer)
/*****************************************************************************/
{
  // Filter infinity to max_range
  float epsilon = 0.0001;
  sensor_msgs::LaserScan message = *raw_message;
  for (size_t i = 0; i < message.ranges.size(); i++)
  {
    float range = message.ranges[i];
    if (!std::isfinite(range) && range > 0)
    {
      message.ranges[i] = message.range_max - epsilon;
    }
  }
  sensor_msgs::PointCloud2 cloud;
  cloud.header = message.header;
  try {
    _laser_projector.transformLaserScanToPointCloud( \
                              message.header.frame_id, message, cloud, *tf_);
  } catch (tf::TransformException &ex) {
    ROS_WARN("TF returned a transform exception to frame %s: %s", \
             _global_frame.c_str(), ex.what());
    _laser_projector.projectLaser(message, cloud);
  }
  // buffer the point cloud
  buffer->Lock();
  buffer->BufferROSCloud(cloud);
  buffer->Unlock();
}

/*****************************************************************************/
void SpatioTemporalVoxelLayerWithInflation::PointCloud2Callback( \
                const sensor_msgs::PointCloud2ConstPtr& message, \
                const boost::shared_ptr<buffer::MeasurementBuffer>& buffer)
/*****************************************************************************/
{
  // buffer the point cloud
  buffer->Lock();
  buffer->BufferROSCloud(*message);
  buffer->Unlock();
}

/*****************************************************************************/
bool SpatioTemporalVoxelLayerWithInflation::BufferEnablerCallback(   \
                std_srvs::SetBool::Request& request,   \
                std_srvs::SetBool::Response& response, \
                boost::shared_ptr<buffer::MeasurementBuffer>& buffer, \
                boost::shared_ptr<message_filters::SubscriberBase>& subcriber)
/*****************************************************************************/
{
  buffer->Lock();
  if (buffer->IsEnabled() != request.data)
  {
    buffer->SetEnabled(request.data);
    if (request.data)
    {
      subcriber->subscribe();
      buffer->ResetLastUpdatedTime();
      response.message = "Enabling sensor";
    }
    else if (subcriber)
    {
      subcriber->unsubscribe();
      response.message = "Disabling sensor";
    }
  }
  else
  {
    response.message = "Sensor already in the required state doing nothing";
  }
  buffer->Unlock();
  response.success = true;
  return response.success;
}


/*****************************************************************************/
bool SpatioTemporalVoxelLayerWithInflation::GetMarkingObservations( \
      std::vector<observation::MeasurementReading>& marking_observations) const
/*****************************************************************************/
{
  // get marking observations and static marked areas
  bool current = true;

  for (unsigned int i = 0; i != _marking_buffers.size(); ++i)
  {
    _marking_buffers[i]->Lock();
    _marking_buffers[i]->GetReadings(marking_observations);
    current = _marking_buffers[i]->UpdatedAtExpectedRate();
    _marking_buffers[i]->Unlock();
  }
  marking_observations.insert(marking_observations.end(),   \
                              _static_observations.begin(), \
                              _static_observations.end());
  return current;
}

/*****************************************************************************/
bool SpatioTemporalVoxelLayerWithInflation::GetClearingObservations( \
     std::vector<observation::MeasurementReading>& clearing_observations) const
/*****************************************************************************/
{
  // get clearing observations
  bool current = true;
  for (unsigned int i = 0; i != _clearing_buffers.size(); ++i)
  {
    _clearing_buffers[i]->Lock();
    _clearing_buffers[i]->GetReadings(clearing_observations);
    current = _clearing_buffers[i]->UpdatedAtExpectedRate();
    _clearing_buffers[i]->Unlock();
  }
  return current;
}

/*****************************************************************************/
void SpatioTemporalVoxelLayerWithInflation::ObservationsResetAfterReading() const
/*****************************************************************************/
{
  for (unsigned int i = 0; i != _clearing_buffers.size(); ++i)
  {
    _clearing_buffers[i]->Lock();
    if (_clearing_buffers[i]->ClearAfterReading())
    {
      _clearing_buffers[i]->ResetAllMeasurements();
    }
    _clearing_buffers[i]->Unlock();
  }

  for (unsigned int i = 0; i != _marking_buffers.size(); ++i)
  {
    _marking_buffers[i]->Lock();
    if (_marking_buffers[i]->ClearAfterReading())
    {
      _marking_buffers[i]->ResetAllMeasurements();
    }
    _marking_buffers[i]->Unlock();
  }
  return;
}

/*****************************************************************************/
bool SpatioTemporalVoxelLayerWithInflation::updateFootprint(double robot_x, double robot_y, \
                                               double robot_yaw, double* min_x,\
                                               double* min_y, double* max_x,   \
                                               double* max_y)
/*****************************************************************************/
{
  // updates layer costmap to include footprint for clearing in voxel grid
  if (!_update_footprint_enabled)
  {
    return false;
  }
  costmap_2d::transformFootprint(robot_x, robot_y, robot_yaw, getFootprint(), \
                                 _transformed_footprint);
  for (unsigned int i = 0; i < _transformed_footprint.size(); i++)
  {
    touch(_transformed_footprint[i].x, _transformed_footprint[i].y, \
          min_x, min_y, max_x, max_y);
  }
  return true;
}

/*****************************************************************************/
void SpatioTemporalVoxelLayerWithInflation::activate(void)
/*****************************************************************************/
{
  // subscribe and place info in buffers from sensor sources
  ROS_INFO("%s was activated.", getName().c_str());

  observation_subscribers_iter sub_it = _observation_subscribers.begin();
  for (sub_it; sub_it != _observation_subscribers.end(); ++sub_it)
  {
    (*sub_it)->subscribe();
  }

  observation_buffers_iter buf_it = _observation_buffers.begin();
  for (buf_it; buf_it != _observation_buffers.end(); ++buf_it)
  {
    (*buf_it)->ResetLastUpdatedTime();
  }
}

/*****************************************************************************/
void SpatioTemporalVoxelLayerWithInflation::deactivate(void)
/*****************************************************************************/
{
  // unsubscribe from all sensor sources
  ROS_INFO("%s was deactivated.", getName().c_str());

  observation_subscribers_iter sub_it = _observation_subscribers.begin();
  for (sub_it; sub_it != _observation_subscribers.end(); ++sub_it)
  {
    if (*sub_it != NULL)
    {
      (*sub_it)->unsubscribe();
    }
  }
}

/*****************************************************************************/
void SpatioTemporalVoxelLayerWithInflation::reset(void)
/*****************************************************************************/
{
  boost::recursive_mutex::scoped_lock lock(_voxel_grid_lock);
  // reset layer
  Costmap2D::resetMaps();
  this->ResetGrid();
  current_ = true;
  observation_buffers_iter it = _observation_buffers.begin();
  for (it; it != _observation_buffers.end(); ++it)
  {
    (*it)->ResetLastUpdatedTime();
  }
}

/*****************************************************************************/
bool SpatioTemporalVoxelLayerWithInflation::AddStaticObservations( \
                                    const observation::MeasurementReading& obs)
/*****************************************************************************/
{
  // observations to always be added to the map each update cycle not marked
  ROS_INFO("%s: Adding static observation to map.", getName().c_str());

  try {
    _static_observations.push_back(obs);
    return true;
  } catch(...) {
    ROS_WARN("Could not add static observations to voxel layer");
    return false;
  }
}

/*****************************************************************************/
bool SpatioTemporalVoxelLayerWithInflation::RemoveStaticObservations(void)
/*****************************************************************************/
{
  // kill all static observations added to each update cycle
  ROS_INFO("%s: Removing static observations to map.", getName().c_str());

  try {
    _static_observations.clear();
    return true;
  } catch(...) {
    ROS_WARN("Couldn't remove static observations from %s.", \
             getName().c_str());
    return false;
  }
}

/*****************************************************************************/
void SpatioTemporalVoxelLayerWithInflation::DynamicReconfigureCallback( \
                        SpatioTemporalVoxelLayerWithInflationConfig& config, uint32_t level)
/*****************************************************************************/
{
  boost::recursive_mutex::scoped_lock lock(_voxel_grid_lock);

  setInflationParameters(config.inflation_radius, config.cost_scaling_factor);

  _enabled = config.enabled;
  _combination_method = config.combination_method;
  _mark_threshold = config.mark_threshold;
  _update_footprint_enabled = config.update_footprint_enabled;
  _mapping_mode = config.mapping_mode;
  _map_save_duration = ros::Duration(config.map_save_duration);

  if (level >=1) //update grid
  {
    auto default_value = (config.track_unknown_space) ? \
                            costmap_2d::NO_INFORMATION : costmap_2d::FREE_SPACE;
    default_value_ = default_value;
    _voxel_size = config.voxel_size;
    _voxel_decay = config.voxel_decay;
    _decay_model = static_cast<volume_grid::GlobalDecayModel>(config.decay_model);
    _publish_voxels = config.publish_voxel_map;

    delete _voxel_grid;
    _voxel_grid = new volume_grid::SpatioTemporalVoxelGrid(_voxel_size, \
      static_cast<double>(default_value_), _decay_model, \
      _voxel_decay, _publish_voxels);
  }
}

/*****************************************************************************/
void SpatioTemporalVoxelLayerWithInflation::ResetGrid(void)
/*****************************************************************************/
{
  if (!_voxel_grid->ResetGrid())
  {
   ROS_WARN("Did not clear level set in %s!", getName().c_str());
  }
}

/*****************************************************************************/
void SpatioTemporalVoxelLayerWithInflation::matchSize(void)
/*****************************************************************************/
{
  boost::unique_lock < boost::recursive_mutex > lock(*inflation_access_);
  costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
  resolution_ = costmap->getResolution();
  inscribed_radius_ = 0.5;
  // inscribed_radius_ = layered_costmap_->getInscribedRadius();
  cell_inflation_radius_ = cellDistance(inflation_radius_);
  computeCaches();

  unsigned int size_x = costmap->getSizeInCellsX(), size_y = costmap->getSizeInCellsY();
  if (seen_)
    delete[] seen_;
  seen_size_ = size_x * size_y;
  seen_ = new bool[seen_size_];
  // match the master costmap size, volume_grid maintains full w/ expiration.
  CostmapLayer::matchSize();
}

/*****************************************************************************/
void SpatioTemporalVoxelLayerWithInflation::updateCosts( \
                                    costmap_2d::Costmap2D& master_grid, \
                                    int min_i, int min_j, int max_i, int max_j)
/*****************************************************************************/
{
  
  // update costs in master_grid with costmap_
  if(!_enabled)
  {
    return;
  }

  if (_update_footprint_enabled)
  {
    setConvexPolygonCost(_transformed_footprint, costmap_2d::FREE_SPACE);
  }

  switch (_combination_method)
  {
  case 0:
    updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
  case 1:
    updateWithMax(master_grid, min_i, min_j, max_i, max_j);
  default:
    break;
  }
  return;
}

/*****************************************************************************/
void SpatioTemporalVoxelLayerWithInflation::UpdateROSCostmap( \
                        double* min_x, double* min_y, \
                        double* max_x, double* max_y, \
                        std::unordered_set<volume_grid::occupany_cell>& cleared_cells, \
                        double robot_yaw)
/*****************************************************************************/
{
  // ROS_INFO("%s robot_yaw: %f", getName().c_str(),robot_yaw);
  costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
  unsigned int size_x = costmap->getSizeInCellsX(), size_y = costmap->getSizeInCellsY();
  // grabs map of occupied cells from grid and adds to costmap_
  Costmap2D::resetMaps();
  inflation_cells_.empty();
  if (seen_ == NULL) {
    seen_size_ = size_x * size_y;
    seen_ = new bool[seen_size_];
  }
  else if (seen_size_ != size_x * size_y)
  {
    ROS_WARN("InflationLayer::updateCosts(): seen_ array size is wrong");
    delete[] seen_;
    seen_size_ = size_x * size_y;
    seen_ = new bool[seen_size_];
  }
  memset(seen_, false, size_x * size_y * sizeof(bool));
  std::vector<CellData>& obs_bin = inflation_cells_[0.0];
  std::unordered_map<volume_grid::occupany_cell, uint>::iterator it;
  for (it = _voxel_grid->GetFlattenedCostmap()->begin();
       it != _voxel_grid->GetFlattenedCostmap()->end(); ++it)
  {
    uint map_x, map_y;
    if ( it->second >= _mark_threshold && \
         worldToMap(it->first.x, it->first.y, map_x, map_y))
    {
      costmap_[getIndex(map_x, map_y)] = costmap_2d::LETHAL_OBSTACLE;
      touch(it->first.x, it->first.y, min_x, min_y, max_x, max_y);

      // Start with lethal obstacles: by definition distance is 0.0
      obs_bin.push_back(CellData(getIndex(map_x, map_y), map_x, map_y, map_x, map_y));
    }
  }

  std::unordered_set<volume_grid::occupany_cell>::iterator cell;
  for (cell = cleared_cells.begin(); cell != cleared_cells.end(); ++cell)
  {
    touch(cell->x, cell->y, min_x, min_y, max_x, max_y);
  }

  // Process cells by increasing distance; new cells are appended to the corresponding distance bin, so they
  // can overtake previously inserted but farther away cells
  std::map<double, std::vector<CellData> >::iterator bin;
  for (bin = inflation_cells_.begin(); bin != inflation_cells_.end(); ++bin)
  {
    for (int i = 0; i < bin->second.size(); ++i)
    {
      // process all cells at distance dist_bin.first
      const CellData& cell = bin->second[i];

      unsigned int index = cell.index_;

      // ignore if already visited
      if (seen_[index])
      {
        continue;
      }

      seen_[index] = true;

      unsigned int mx = cell.x_;
      unsigned int my = cell.y_;
      unsigned int sx = cell.src_x_;
      unsigned int sy = cell.src_y_;

      // assign the cost associated with the distance from an obstacle to the cell
      unsigned char cost = costLookup(mx, my, sx, sy);
      unsigned char old_cost = costmap_[index];
      
      if (old_cost == costmap_2d::NO_INFORMATION && (inflate_unknown_ ? (cost > costmap_2d::FREE_SPACE) : (cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)))
        costmap_[index] = cost;
      else
        costmap_[index] = std::max(old_cost, cost);

      // attempt to put the neighbors of the current cell onto the inflation list
      if (mx > 0)
        enqueue(index - 1, mx - 1, my, sx, sy, robot_yaw);
      if (my > 0)
        enqueue(index - size_x, mx, my - 1, sx, sy, robot_yaw);
      if (mx < size_x - 1)
        enqueue(index + 1, mx + 1, my, sx, sy, robot_yaw);
      if (my < size_y - 1)
        enqueue(index + size_x, mx, my + 1, sx, sy, robot_yaw);
    }
  }

  inflation_cells_.clear();

}

/*****************************************************************************/
void SpatioTemporalVoxelLayerWithInflation::updateBounds( \
                    double robot_x, double robot_y, double robot_yaw, \
                    double* min_x, double* min_y, double* max_x, double* max_y)
/*****************************************************************************/
{
  // grabs new max bounds for the costmap
  if (!_enabled)
  {
    return;
  }

  boost::recursive_mutex::scoped_lock lock(_voxel_grid_lock);

  // Steve's Note June 22, 2018
  // I dislike this necessity, I can't remove the master grid's knowledge about
  // STVL on the fly so I have play games with the API even though this isn't
  // really a rolling plugin implementation. It works, but isn't ideal.
  if (layered_costmap_->isRolling())
  {
    updateOrigin(robot_x-getSizeInMetersX()/2, robot_y-getSizeInMetersY()/2);
  }

  useExtraBounds(min_x, min_y, max_x, max_y);

  double tmp_min_x = last_min_x_;
  double tmp_min_y = last_min_y_;
  double tmp_max_x = last_max_x_;
  double tmp_max_y = last_max_y_;
  last_min_x_ = *min_x;
  last_min_y_ = *min_y;
  last_max_x_ = *max_x;
  last_max_y_ = *max_y;
  *min_x = std::min(tmp_min_x, *min_x) - inflation_radius_;
  *min_y = std::min(tmp_min_y, *min_y) - inflation_radius_;
  *max_x = std::max(tmp_max_x, *max_x) + inflation_radius_;
  *max_y = std::max(tmp_max_y, *max_y) + inflation_radius_;

  bool current = true;
  std::vector<observation::MeasurementReading> marking_observations, \
                                               clearing_observations;
  current = GetMarkingObservations(marking_observations) && current;
  current = GetClearingObservations(clearing_observations) && current;
  ObservationsResetAfterReading();
  current_ = current;

  std::unordered_set<volume_grid::occupany_cell> cleared_cells;

  // navigation mode: clear observations, mapping mode: save maps and publish
  if (!_mapping_mode)
  {
    _voxel_grid->ClearFrustums(clearing_observations, cleared_cells);
  }
  else if (ros::Time::now() - _last_map_save_time > _map_save_duration)
  {
    _last_map_save_time = ros::Time::now();
    time_t rawtime;
    struct tm* timeinfo;
    char time_buffer[100];
    time (&rawtime);
    timeinfo = localtime (&rawtime);
    strftime(time_buffer, 100, "%F-%r", timeinfo);

    spatio_temporal_voxel_layer_with_inflation::SaveGrid srv;
    srv.request.file_name.data = time_buffer;
    SaveGridCallback(srv.request, srv.response);
  }

  // mark observations
  _voxel_grid->Mark(marking_observations);

  // update the ROS Layered Costmap
  UpdateROSCostmap(min_x, min_y, max_x, max_y, cleared_cells, robot_yaw);

  // publish point cloud in navigation mode
  if (_publish_voxels && !_mapping_mode)
  {
    sensor_msgs::PointCloud2::Ptr pc2(new sensor_msgs::PointCloud2());
    _voxel_grid->GetOccupancyPointCloud(pc2);
    pc2->header.frame_id = _global_frame;
    pc2->header.stamp = ros::Time::now();
    _voxel_pub.publish(*pc2);
  }

  // update footprint
  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
  return;
}

/*****************************************************************************/
bool SpatioTemporalVoxelLayerWithInflation::SaveGridCallback( \
                         spatio_temporal_voxel_layer_with_inflation::SaveGrid::Request& req, \
                         spatio_temporal_voxel_layer_with_inflation::SaveGrid::Response& resp)
/*****************************************************************************/
{
  boost::recursive_mutex::scoped_lock lock(_voxel_grid_lock);
  double map_size_bytes;

  if( _voxel_grid->SaveGrid(req.file_name.data, map_size_bytes) )
  {
    ROS_INFO( \
      "SpatioTemporalVoxelGrid: Saved %s grid! Has memory footprint of %f bytes.",
      req.file_name.data.c_str(), map_size_bytes);
    resp.map_size_bytes = map_size_bytes;
    resp.status = true;
    return true;
  }

  ROS_WARN("SpatioTemporalVoxelGrid: Failed to save grid.");
  resp.status = false;
  return false;
}

void SpatioTemporalVoxelLayerWithInflation::setInflationParameters(double inflation_radius, double cost_scaling_factor)
{
  if (weight_ != cost_scaling_factor || inflation_radius_ != inflation_radius)
  {
    // Lock here so that reconfiguring the inflation radius doesn't cause segfaults
    // when accessing the cached arrays
    boost::unique_lock < boost::recursive_mutex > lock(*inflation_access_);

    inflation_radius_ = inflation_radius;
    cell_inflation_radius_ = cellDistance(inflation_radius_);
    weight_ = cost_scaling_factor;
    need_reinflation_ = true;
    computeCaches();
  }
}

void SpatioTemporalVoxelLayerWithInflation::computeCaches()
{
  if (cell_inflation_radius_ == 0)
    return;

  // based on the inflation radius... compute distance and cost caches
  if (cell_inflation_radius_ != cached_cell_inflation_radius_)
  {
    
    deleteKernels();

    cached_costs_ = new unsigned char*[cell_inflation_radius_ + 2];
    cached_distances_ = new double*[cell_inflation_radius_ + 2];

    for (unsigned int i = 0; i <= cell_inflation_radius_ + 1; ++i)
    {
      cached_costs_[i] = new unsigned char[cell_inflation_radius_ + 2];
      cached_distances_[i] = new double[cell_inflation_radius_ + 2];
      for (unsigned int j = 0; j <= cell_inflation_radius_ + 1; ++j)
      {
        cached_distances_[i][j] = hypot(i, j);
      }
    }

    cached_cell_inflation_radius_ = cell_inflation_radius_;
  }

  for (unsigned int i = 0; i <= cell_inflation_radius_ + 1; ++i)
  {
    for (unsigned int j = 0; j <= cell_inflation_radius_ + 1; ++j)
    {
      cached_costs_[i][j] = computeCost(cached_distances_[i][j]);
    }
  }
}

void SpatioTemporalVoxelLayerWithInflation::deleteKernels()
{
  if (cached_distances_ != NULL)
  {
    for (unsigned int i = 0; i <= cached_cell_inflation_radius_ + 1; ++i)
    {
      if (cached_distances_[i])
        delete[] cached_distances_[i];
    }
    if (cached_distances_)
      delete[] cached_distances_;
    cached_distances_ = NULL;
  }

  if (cached_costs_ != NULL)
  {
    for (unsigned int i = 0; i <= cached_cell_inflation_radius_ + 1; ++i)
    {
      if (cached_costs_[i])
        delete[] cached_costs_[i];
    }
    delete[] cached_costs_;
    cached_costs_ = NULL;
  }
}

inline void SpatioTemporalVoxelLayerWithInflation::enqueue(unsigned int index, unsigned int mx, unsigned int my,
                                    unsigned int src_x, unsigned int src_y, double robot_yaw)
{
  int _mx = (int)mx;
  int _my = (int)my;
  int _src_x = (int)src_x;
  int _src_y = (int)src_y;
  if(fabs(atan2(_src_y-_my,_src_x-_mx)-robot_yaw) > 0.78 && fabs(_src_y-_my)+fabs(_src_x-_mx)>1) return;

  if (!seen_[index])
  {
    // we compute our distance table one cell further than the inflation radius dictates so we can make the check below
    double distance = distanceLookup(mx, my, src_x, src_y);

    // we only want to put the cell in the list if it is within the inflation radius of the obstacle point
    if (distance > cell_inflation_radius_)
      return;

    // push the cell data onto the inflation list and mark
    inflation_cells_[distance].push_back(CellData(index, mx, my, src_x, src_y));
  }
}

}; // end namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(spatio_temporal_voxel_layer_with_inflation::SpatioTemporalVoxelLayerWithInflation, costmap_2d::Layer);
