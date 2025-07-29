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

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_SENSOR_BRIDGE_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_SENSOR_BRIDGE_H

#include <memory>

#include "absl/types/optional.h"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/tf_bridge.h"
#include "cartographer_ros_msgs/msg/landmark_list.hpp"
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/multi_echo_laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace cartographer_ros
{

    // Converts ROS messages into SensorData in tracking frame for the MapBuilder.
    // 处理和转换传感器数据的接口
    // 负责将来自不同传感器（如里程计、IMU、激光扫描等）的数据转换为Cartographer所需的格式，并提供相应的处理函数。
    class SensorBridge
    {
    public:
        // 构造函数，初始化传感器桥接器，设置激光扫描的子分段数量、跟踪框架、变换查找超时时间、TF缓冲区和轨迹构建器。
        // num_subdivisions_per_laser_scan: 每个激光扫描的子分段数量，用于精细化处理激光数据。
        // tracking_frame: 跟踪框架的名称
        // lookup_transform_timeout_sec: 查找变换的超时时间（秒）
        // tf_buffer: 用于处理坐标变换的TF缓冲区
        // trajectory_builder: 轨迹构建器接口，用于处理传感器数据
        explicit SensorBridge(
            int num_subdivisions_per_laser_scan, const std::string &tracking_frame,
            double lookup_transform_timeout_sec, tf2_ros::Buffer *tf_buffer,
            ::cartographer::mapping::TrajectoryBuilderInterface *trajectory_builder);

        // 禁止拷贝构造和赋值运算，以防止不必要的复制操作
        SensorBridge(const SensorBridge &) = delete;
        SensorBridge &operator=(const SensorBridge &) = delete;

        // 将传感器数据转换为对应的 SensorData 类型，并处理相应的消息。
        // 将 nav_msgs::msg::Odometry 消息转换为 Cartographer 使用的 OdometryData 对象。
        std::unique_ptr<::cartographer::sensor::OdometryData> ToOdometryData(
            const nav_msgs::msg::Odometry::ConstSharedPtr &msg);
        // 处理来自特定传感器的里程计消息，将其传递给轨迹构建器。
        void HandleOdometryMessage(const std::string &sensor_id,
                                   const nav_msgs::msg::Odometry::ConstSharedPtr &msg);
        // 处理来自 GPS 传感器的定位消息，以增强定位精度。
        void HandleNavSatFixMessage(const std::string &sensor_id,
                                    const sensor_msgs::msg::NavSatFix::ConstSharedPtr &msg);
        // 处理地标消息，通常用于增强地图构建的准确性。
        void HandleLandmarkMessage(
            const std::string &sensor_id,
            const cartographer_ros_msgs::msg::LandmarkList::ConstSharedPtr &msg);

        // 将传感器数据转换为 IMU 数据，适用于 Cartographer 的处理。
        std::unique_ptr<::cartographer::sensor::ImuData> ToImuData(
            const sensor_msgs::msg::Imu::ConstSharedPtr &msg);
        // 处理 IMU 消息，将其转换为 Cartographer 使用的 IMU 数据，并传递给轨迹构建器。
        void HandleImuMessage(const std::string &sensor_id,
                              const sensor_msgs::msg::Imu::ConstSharedPtr &msg);
        // 处理laser扫描消息，将其转换为 Cartographer 使用的点云数据，并传递给轨迹构建器。
        void HandleLaserScanMessage(const std::string &sensor_id,
                                    const sensor_msgs::msg::LaserScan::ConstSharedPtr &msg);
        // 处理多回波激光扫描消息，将其转换为 Cartographer 使用的点云数据，并传递给轨迹构建器。
        void HandleMultiEchoLaserScanMessage(
            const std::string &sensor_id,
            const sensor_msgs::msg::MultiEchoLaserScan::ConstSharedPtr &msg);
        // 处理点云2消息，将其转换为 Cartographer 使用的点云数据，并传递给轨迹构建器。
        void HandlePointCloud2Message(const std::string &sensor_id,
                                      const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);

        // tf_bridge: 获取当前的 TF 桥接器，用于处理坐标变换。
        const TfBridge &tf_bridge() const;

    private:
        /**
         * @brief 处理激光扫描数据，将其分割为多个子分段，并将每个子分段传递给轨迹构建器。
         *
         * @param sensor_id 传感器 ID，用于标识数据来源
         * @param start_time 激光扫描开始的时间戳
         * @param frame_id 激光扫描数据的坐标框架 ID
         * @param points 激光扫描点云数据，包含点的坐标和强度信息
         */
        void HandleLaserScan(
            const std::string &sensor_id, ::cartographer::common::Time start_time,
            const std::string &frame_id,
            const ::cartographer::sensor::PointCloudWithIntensities &points);
        /**
         * @brief 处理范围传感器数据，将其转换为 Cartographer 使用的格式，并传递给轨迹构建器。
         *
         * @param sensor_id 传感器 ID，用于标识数据来源
         * @param time 时间戳，表示数据采集的时间
         * @param frame_id 坐标框架 ID，表示数据所属的坐标系
         * @param ranges 范围传感器采集的点云数据，包含点的坐标和强度信息
         */
        void HandleRangefinder(const std::string &sensor_id,
                               ::cartographer::common::Time time,
                               const std::string &frame_id,
                               const ::cartographer::sensor::TimedPointCloud &ranges);

        const int num_subdivisions_per_laser_scan_; // 每个激光扫描的子分段数量，用于精细化处理激光数据
        std::map<std::string, cartographer::common::Time>
            sensor_to_previous_subdivision_time_; // 传感器 ID 到上一个子分段时间的映射，用于跟踪每个传感器的最新数据时间
        const TfBridge tf_bridge_;                // TF 桥接器，用于处理坐标变换
        ::cartographer::mapping::TrajectoryBuilderInterface *const
            trajectory_builder_; // 轨迹构建器接口，用于处理传感器数据并构建地图

        absl::optional<::cartographer::transform::Rigid3d> ecef_to_local_frame_; // ECEF（地心地固坐标系）到本地坐标系的变换，用于处理 GPS 数据
    };

} // namespace cartographer_ros

#endif // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_SENSOR_BRIDGE_H
