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

include "map_builder.lua"
include "trajectory_builder.lua"

options = {    -- 主要用于和ROS2进行通信和数据收发的配置
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",                  -- 用来发布子地图的ROS坐标系ID，位姿的父坐标系，通常是map。
  tracking_frame = "base_link",       -- SLAM算法跟随的坐标系ID
  published_frame = "base_link",      -- 将发布map到published_frame之间的tf
  odom_frame = "odom",                -- 位于“published_frame ”和“map_frame”之间，用来发布本地SLAM结果（非闭环），通常是“odom”
  provide_odom_frame = true,          -- 是否提供里程计
  publish_frame_projected_to_2d = false,  -- 只发布二维位姿态（不包含俯仰角）
  use_pose_extrapolator = true,
  use_odometry = false,               -- 是否使用里程计数据
  use_nav_sat = false,                -- 是否使用GPS定位
  use_landmarks = false,              -- 是否使用路标
  num_laser_scans = 0,                -- 订阅的laser scan topics的个数
  num_multi_echo_laser_scans = 1,     -- 订阅多回波技术laser scan topics的个数
  num_subdivisions_per_laser_scan = 10,  -- 分割雷达数据的个数
  num_point_clouds = 0,               -- 订阅的点云topics的个数
  lookup_transform_timeout_sec = 0.2, -- 使用tf2查找变换的超时秒数
  submap_publish_period_sec = 0.3,    -- 发布submap的周期间隔
  pose_publish_period_sec = 5e-3,     -- 发布姿态的周期间隔
  trajectory_publish_period_sec = 30e-3, -- 轨迹发布周期间隔
  rangefinder_sampling_ratio = 1.,    -- 测距仪的采样率
  odometry_sampling_ratio = 1.,       --里程记数据采样率
  fixed_frame_pose_sampling_ratio = 1.,  -- 固定的frame位姿采样率
  imu_sampling_ratio = 1.,            -- IMU数据采样率
  landmarks_sampling_ratio = 1.,      -- 路标采样率
}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 10

return options
