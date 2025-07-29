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

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_MAP_BUILDER_BRIDGE_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_MAP_BUILDER_BRIDGE_H

#include <memory>
#include <set>
#include <string>
#include <unordered_map>

#include "absl/synchronization/mutex.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/sensor_bridge.h"
#include "cartographer_ros/tf_bridge.h"
#include "cartographer_ros/trajectory_options.h"
#include "cartographer_ros_msgs/msg/submap_entry.hpp"
#include "cartographer_ros_msgs/msg/submap_list.hpp"
#include "cartographer_ros_msgs/srv/submap_query.hpp"
#include "cartographer_ros_msgs/srv/trajectory_query.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

// Abseil unfortunately pulls in winnt.h, which #defines DELETE.
// Clean up to unbreak visualization_msgs::msg::Marker::DELETE.
#ifdef DELETE
#undef DELETE
#endif
#include "visualization_msgs/msg/marker_array.hpp"

namespace cartographer_ros
{

    class MapBuilderBridge
    {
    public:
        struct LocalTrajectoryData
        {
            // Contains the trajectory data received from local SLAM, after
            // it had processed accumulated 'range_data_in_local' and estimated
            // current 'local_pose' at 'time'.
            // 包含从局部 SLAM 接收到的轨迹数据，在处理累积的 'range_data_in_local' 后，
            // 并在 'time' 时估计当前的 'local_pose'。
            // 该数据结构用于存储局部 SLAM 的结果。
            struct LocalSlamData
            {
                ::cartographer::common::Time time;                     // 时间戳
                ::cartographer::transform::Rigid3d local_pose;         // 局部坐标系下的刚体变换
                ::cartographer::sensor::RangeData range_data_in_local; // 局部坐标系下的传感器数据
            };
            std::shared_ptr<const LocalSlamData> local_slam_data;                    // 局部 SLAM 数据的共享指针
            cartographer::transform::Rigid3d local_to_map;                           // 局部坐标系到地图坐标系的变换
            std::unique_ptr<cartographer::transform::Rigid3d> published_to_tracking; // 发布到跟踪坐标系的变换
            TrajectoryOptions trajectory_options;                                    // 轨迹选项，包含了传感器配置和其他参数
        };

        MapBuilderBridge(
            const NodeOptions &node_options,                                         // 节点选项，包含地图构建器配置等
            std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder, // 地图构建器接口的唯一指针
            tf2_ros::Buffer *tf_buffer                                               // TF 缓冲区，用于处理坐标变换
        );

        MapBuilderBridge(const MapBuilderBridge &) = delete;            // 禁止拷贝构造函数
        MapBuilderBridge &operator=(const MapBuilderBridge &) = delete; // 禁止赋值操作符

        void LoadState(const std::string &state_filename, bool load_frozen_state); // 从指定的状态文件加载状态，是否加载冻结状态
        int AddTrajectory(
            const std::set<
                ::cartographer::mapping::TrajectoryBuilderInterface::SensorId> &
                expected_sensor_ids,
            const TrajectoryOptions &trajectory_options);           // 添加一个新的轨迹，返回轨迹ID
        void FinishTrajectory(int trajectory_id);                   // 完成指定 ID 的轨迹
        void RunFinalOptimization();                                // 运行最终优化，通常在所有轨迹完成后调用
        bool SerializeState(const std::string &filename,
                            const bool include_unfinished_submaps); // 序列化当前状态到指定文件

        void HandleSubmapQuery(
            const cartographer_ros_msgs::srv::SubmapQuery::Request::SharedPtr request,
            cartographer_ros_msgs::srv::SubmapQuery::Response::SharedPtr response);
        void HandleTrajectoryQuery(
            const cartographer_ros_msgs::srv::TrajectoryQuery::Request::SharedPtr request,
            cartographer_ros_msgs::srv::TrajectoryQuery::Response::SharedPtr response);

        std::map<int /* trajectory_id */,
                 ::cartographer::mapping::PoseGraphInterface::TrajectoryState>
        GetTrajectoryStates();                                                        // 获取所有轨迹的状态，返回一个映射，键为轨迹 ID，值为轨迹状态
        cartographer_ros_msgs::msg::SubmapList GetSubmapList(rclcpp::Time node_time); // 获取子地图列表，返回一个包含子地图信息的消息
        std::unordered_map<int, LocalTrajectoryData> GetLocalTrajectoryData()
            LOCKS_EXCLUDED(mutex_);                                                         // 获取局部轨迹数据，返回一个映射，键为轨迹 ID，值为局部轨迹数据
        visualization_msgs::msg::MarkerArray GetTrajectoryNodeList(rclcpp::Time node_time); // 获取轨迹节点列表，返回一个包含轨迹节点的可视化标记数组
        visualization_msgs::msg::MarkerArray GetLandmarkPosesList(rclcpp::Time node_time);  // 获取地标位置列表，返回一个包含地标位置的可视化标记数组
        visualization_msgs::msg::MarkerArray GetConstraintList(rclcpp::Time node_time);     // 获取约束列表，返回一个包含约束信息的可视化标记数组

        SensorBridge *sensor_bridge(int trajectory_id); // 获取指定轨迹 ID 的传感器桥接器，返回一个指向 SensorBridge 的指针

    private:
        void OnLocalSlamResult(const int trajectory_id,
                               const ::cartographer::common::Time time,
                               const ::cartographer::transform::Rigid3d local_pose,
                               ::cartographer::sensor::RangeData range_data_in_local)
            LOCKS_EXCLUDED(mutex_); // 处理局部 SLAM 结果，更新局部轨迹数据

        absl::Mutex mutex_;              // 互斥锁，用于保护共享数据的访问
        const NodeOptions node_options_; // 节点选项，包含地图构建器配置等
        std::unordered_map<int,
                           std::shared_ptr<const LocalTrajectoryData::LocalSlamData>>
            local_slam_data_ GUARDED_BY(mutex_);                                  // 局部 SLAM 数据，键为轨迹 ID，值为局部 SLAM 数据的共享指针
        std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder_; // 地图构建器接口的唯一指针
        tf2_ros::Buffer *const tf_buffer_;                                        // TF 缓冲区，用于处理坐标变换

        std::unordered_map<std::string /* landmark ID */, int> landmark_to_index_; // 地标 ID 到索引的映射

        // These are keyed with 'trajectory_id'.
        std::unordered_map<int, TrajectoryOptions> trajectory_options_;         // 轨迹选项，键为轨迹 ID，值为轨迹选项
        std::unordered_map<int, std::unique_ptr<SensorBridge>> sensor_bridges_; // 传感器桥接器，键为轨迹 ID，值为传感器桥接器的唯一指针
        std::unordered_map<int, size_t> trajectory_to_highest_marker_id_;       // 轨迹 ID 到最高标记 ID 的映射，键为轨迹 ID，值为最高标记 ID
    };

} // namespace cartographer_ros

#endif // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_MAP_BUILDER_BRIDGE_H
