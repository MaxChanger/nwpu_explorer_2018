-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.
--这个里面对参数的调整也会关系到出图的效果
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  
  map_frame = "map",
  tracking_frame = "base_link",   --base_link
  published_frame = "base_link",  ----原来是horizontal_laser_link
  odom_frame = "odom", 
  --“provide_odom_frame”为真时使用，位于“published_frame ”和“map_frame”之间，用来发布本地SLAM结果（非闭环），通常是“odom”。
  
  provide_odom_frame = true,
  use_odometry = false,
  --use_nav_sat = false,          
  --如果使能，将订阅nav_msgs/Odometry类型的topic “odom”，这种情况下，必须提供里程计，而且里程计的信息将被包含在SLAM中。
 
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,                   --订阅的点云topics的个数

  lookup_transform_timeout_sec = 0.2,     --使用tf2进行转换搜素的超时时间的秒数。
                                          --老机子给的0.1，但是这里给了出问题
  submap_publish_period_sec = 0.2,       --发布submap 位置的间隔秒数，如0.3s。

  pose_publish_period_sec = 5e-3,         --发布位置的间隔秒数，如5e-3对应200Hz。
  trajectory_publish_period_sec = 30e-3,  --发布轨迹标记的间隔秒数，如30e-3对应30ms。
  
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35 --submaps的尺寸
TRAJECTORY_BUILDER_2D.min_range = 0.3
TRAJECTORY_BUILDER_2D.max_range = 8.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

--激光的上和参数?
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimize_every_n_nodes = 35
          --POSE_GRAPH.optimize_every_n_nodes = 0 关掉全局SLAM
POSE_GRAPH.constraint_builder.min_score = 0.65



return options
