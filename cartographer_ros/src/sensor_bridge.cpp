/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer_ros/sensor_bridge.h"

#include "absl/memory/memory.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/time_conversion.h"

namespace cartographer_ros {

namespace carto = ::cartographer;

using carto::transform::Rigid3d;

namespace {

// 检查坐标框架 ID 是否以斜杠开头，如果是，则抛出异常。
const std::string& CheckNoLeadingSlash(const std::string& frame_id) {
  if (frame_id.size() > 0) {
    CHECK_NE(frame_id[0], '/') << "The frame_id " << frame_id
                               << " should not start with a /. See 1.7 in "
                                  "http://wiki.ros.org/tf2/Migration.";
  }
  return frame_id;
}

}  // namespace

// 主要工作就是把参数表赋值给成员函数
SensorBridge::SensorBridge(
    const int num_subdivisions_per_laser_scan,
    const std::string& tracking_frame,
    const double lookup_transform_timeout_sec, tf2_ros::Buffer* const tf_buffer,
    carto::mapping::TrajectoryBuilderInterface* const trajectory_builder)
    : num_subdivisions_per_laser_scan_(num_subdivisions_per_laser_scan),
      tf_bridge_(tracking_frame, lookup_transform_timeout_sec, tf_buffer),
      trajectory_builder_(trajectory_builder) {}

// 定义 SensorBridge 类的 ToOdometryData 成员函数，接收一个指向
// nav_msgs::msg::Odometry 消息的常量共享指针 返回一个指向
// carto::sensor::OdometryData 的唯一指针
std::unique_ptr<carto::sensor::OdometryData> SensorBridge::ToOdometryData(
    const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
  //  将ROS2 消息中的时间戳转换为 Cartographer 使用的时间戳格式
  const carto::common::Time time = FromRos(msg->header.stamp);
  // 使用 tf_bridge_ 查找从里程计传感器坐标系到跟踪坐标系的变换。
  // CheckNoLeadingSlash(msg->child_frame_id)
  // 确保传感器坐标系的名称没有以斜杠开头。
  const auto sensor_to_tracking = tf_bridge_.LookupToTracking(
      time, CheckNoLeadingSlash(msg->child_frame_id));
  if (sensor_to_tracking == nullptr) {
    return nullptr;
  }
  // 使用absl::make_unique 创建一个新的 carto::sensor::OdometryData 对象，
  // 将转换后的时间和经过坐标系变换后的位姿赋值给该对象
  return absl::make_unique<carto::sensor::OdometryData>(
      carto::sensor::OdometryData{
          time, ToRigid3d(msg->pose.pose) * sensor_to_tracking->inverse()});
}

void SensorBridge::HandleOdometryMessage(
    const std::string& sensor_id,
    const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
  std::unique_ptr<carto::sensor::OdometryData> odometry_data =
      ToOdometryData(msg);
  if (odometry_data != nullptr) {
    trajectory_builder_->AddSensorData(
        sensor_id,
        carto::sensor::OdometryData{odometry_data->time, odometry_data->pose});
  }
}

void SensorBridge::HandleNavSatFixMessage(
    const std::string& sensor_id,
    const sensor_msgs::msg::NavSatFix::ConstSharedPtr& msg) {
  const carto::common::Time time = FromRos(msg->header.stamp);
  if (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) {
    trajectory_builder_->AddSensorData(
        sensor_id,
        carto::sensor::FixedFramePoseData{time, absl::optional<Rigid3d>()});
    return;
  }

  if (!ecef_to_local_frame_.has_value()) {
    ecef_to_local_frame_ =
        ComputeLocalFrameFromLatLong(msg->latitude, msg->longitude);
    LOG(INFO) << "Using NavSatFix. Setting ecef_to_local_frame with lat = "
              << msg->latitude << ", long = " << msg->longitude << ".";
  }

  trajectory_builder_->AddSensorData(
      sensor_id, carto::sensor::FixedFramePoseData{
                     time, absl::optional<Rigid3d>(Rigid3d::Translation(
                               ecef_to_local_frame_.value() *
                               LatLongAltToEcef(msg->latitude, msg->longitude,
                                                msg->altitude)))});
}

void SensorBridge::HandleLandmarkMessage(
    const std::string& sensor_id,
    const cartographer_ros_msgs::msg::LandmarkList::ConstSharedPtr& msg) {
  auto landmark_data = ToLandmarkData(*msg);

  auto tracking_from_landmark_sensor = tf_bridge_.LookupToTracking(
      landmark_data.time, CheckNoLeadingSlash(msg->header.frame_id));
  if (tracking_from_landmark_sensor != nullptr) {
    for (auto& observation : landmark_data.landmark_observations) {
      observation.landmark_to_tracking_transform =
          *tracking_from_landmark_sensor *
          observation.landmark_to_tracking_transform;
    }
  }
  trajectory_builder_->AddSensorData(sensor_id, landmark_data);
}

// 将ros中的IMU消息转换为Cartographer使用的IMU数据格式
std::unique_ptr<carto::sensor::ImuData> SensorBridge::ToImuData(
    const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
  // 检查线性加速度的协方差
  CHECK_NE(msg->linear_acceleration_covariance[0], -1)
      << "Your IMU data claims to not contain linear acceleration measurements "
         "by setting linear_acceleration_covariance[0] to -1. Cartographer "
         "requires this data to work. See "
         "http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html.";
  // 检查角速度的协方差
  CHECK_NE(msg->angular_velocity_covariance[0], -1)
      << "Your IMU data claims to not contain angular velocity measurements "
         "by setting angular_velocity_covariance[0] to -1. Cartographer "
         "requires this data to work. See "
         "http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html.";
  // 将ROS2消息中的时间戳转换为Cartographer使用的时间戳格式
  const carto::common::Time time = FromRos(msg->header.stamp);
  // 使用tf_bridge_查找从IMU传感器坐标系到跟踪坐标系的变换。
  const auto sensor_to_tracking = tf_bridge_.LookupToTracking(
      time, CheckNoLeadingSlash(msg->header.frame_id));
  if (sensor_to_tracking == nullptr) {
    return nullptr;
  }
  // 检查IMU传感器坐标系与跟踪坐标系是否重合
  CHECK(sensor_to_tracking->translation().norm() < 1e-5)
      << "The IMU frame must be colocated with the tracking frame. "
         "Transforming linear acceleration into the tracking frame will "
         "otherwise be imprecise.";
  // 将IMU消息中的线性加速度和角速度转换为Eigen格式，并应用传感器到跟踪坐标系的旋转变换
  return absl::make_unique<carto::sensor::ImuData>(carto::sensor::ImuData{
      time, sensor_to_tracking->rotation() * ToEigen(msg->linear_acceleration),
      sensor_to_tracking->rotation() * ToEigen(msg->angular_velocity)});
}

// 该函数的主要作用是处理接收到的 IMU 消息，并将其转换为 Cartographer 所需的数据格式，然后将其添加到轨迹构建器中。
void SensorBridge::HandleImuMessage(
    const std::string& sensor_id,
    const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
  std::unique_ptr<carto::sensor::ImuData> imu_data = ToImuData(msg);
  if (imu_data != nullptr) {
    trajectory_builder_->AddSensorData(
        sensor_id,
        carto::sensor::ImuData{imu_data->time, imu_data->linear_acceleration,
                               imu_data->angular_velocity});
  }
}

void SensorBridge::HandleLaserScanMessage(
    const std::string& sensor_id,
    const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg) {
  carto::sensor::PointCloudWithIntensities point_cloud;
  carto::common::Time time;
  std::tie(point_cloud, time) = ToPointCloudWithIntensities(*msg);
  HandleLaserScan(sensor_id, time, msg->header.frame_id, point_cloud);
}

void SensorBridge::HandleMultiEchoLaserScanMessage(
    const std::string& sensor_id,
    const sensor_msgs::msg::MultiEchoLaserScan::ConstSharedPtr& msg) {
  carto::sensor::PointCloudWithIntensities point_cloud;
  carto::common::Time time;
  std::tie(point_cloud, time) = ToPointCloudWithIntensities(*msg);
  HandleLaserScan(sensor_id, time, msg->header.frame_id, point_cloud);
}

void SensorBridge::HandlePointCloud2Message(
    const std::string& sensor_id,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
  carto::sensor::PointCloudWithIntensities point_cloud;
  carto::common::Time time;
  std::tie(point_cloud, time) = ToPointCloudWithIntensities(*msg);
  HandleRangefinder(sensor_id, time, msg->header.frame_id, point_cloud.points);
}

const TfBridge& SensorBridge::tf_bridge() const { return tf_bridge_; }

void SensorBridge::HandleLaserScan(
    const std::string& sensor_id, const carto::common::Time time,
    const std::string& frame_id,
    const carto::sensor::PointCloudWithIntensities& points) {
  if (points.points.empty()) {
    return;
  }
  CHECK_LE(points.points.back().time, 0.f);
  // TODO(gaschler): Use per-point time instead of subdivisions.
  for (int i = 0; i != num_subdivisions_per_laser_scan_; ++i) {
    const size_t start_index =
        points.points.size() * i / num_subdivisions_per_laser_scan_;
    const size_t end_index =
        points.points.size() * (i + 1) / num_subdivisions_per_laser_scan_;
    carto::sensor::TimedPointCloud subdivision(
        points.points.begin() + start_index, points.points.begin() + end_index);
    if (start_index == end_index) {
      continue;
    }
    const double time_to_subdivision_end = subdivision.back().time;
    // `subdivision_time` is the end of the measurement so sensor::Collator will
    // send all other sensor data first.
    const carto::common::Time subdivision_time =
        time + carto::common::FromSeconds(time_to_subdivision_end);
    auto it = sensor_to_previous_subdivision_time_.find(sensor_id);
    if (it != sensor_to_previous_subdivision_time_.end() &&
        it->second >= subdivision_time) {
      LOG(WARNING) << "Ignored subdivision of a LaserScan message from sensor "
                   << sensor_id << " because previous subdivision time "
                   << it->second << " is not before current subdivision time "
                   << subdivision_time;
      continue;
    }
    sensor_to_previous_subdivision_time_[sensor_id] = subdivision_time;
    for (auto& point : subdivision) {
      point.time -= time_to_subdivision_end;
    }
    CHECK_EQ(subdivision.back().time, 0.f);
    HandleRangefinder(sensor_id, subdivision_time, frame_id, subdivision);
  }
}

void SensorBridge::HandleRangefinder(
    const std::string& sensor_id, const carto::common::Time time,
    const std::string& frame_id, const carto::sensor::TimedPointCloud& ranges) {
  if (!ranges.empty()) {
    CHECK_LE(ranges.back().time, 0.f);
  }

  // This was added to get rid of the TimedPointCloudData warning for a missing
  // argument
  std::vector<float> intensities_;

  const auto sensor_to_tracking =
      tf_bridge_.LookupToTracking(time, CheckNoLeadingSlash(frame_id));
  if (sensor_to_tracking != nullptr) {
    trajectory_builder_->AddSensorData(
        sensor_id, carto::sensor::TimedPointCloudData{
                       time, sensor_to_tracking->translation().cast<float>(),
                       carto::sensor::TransformTimedPointCloud(
                           ranges, sensor_to_tracking->cast<float>()),
                       intensities_});
  }
}

}  // namespace cartographer_ros
