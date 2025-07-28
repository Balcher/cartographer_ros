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

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_H

#include <map>
#include <memory>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "absl/synchronization/mutex.h"
#include "cartographer/common/fixed_ratio_sampler.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/pose_extrapolator.h"
#include "cartographer_ros/map_builder_bridge.h"
#include "cartographer_ros/metrics/family_factory.h"
#include "cartographer_ros/node_constants.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/trajectory_options.h"
#include "cartographer_ros_msgs/srv/finish_trajectory.hpp"
#include "cartographer_ros_msgs/srv/get_trajectory_states.hpp"
#include "cartographer_ros_msgs/srv/read_metrics.hpp"
#include "cartographer_ros_msgs/srv/start_trajectory.hpp"
#include "cartographer_ros_msgs/msg/status_response.hpp"
#include "cartographer_ros_msgs/msg/submap_entry.hpp"
#include "cartographer_ros_msgs/msg/submap_list.hpp"
#include "cartographer_ros_msgs/srv/submap_query.hpp"
#include "cartographer_ros_msgs/srv/write_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/multi_echo_laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace cartographer_ros
{

    // Wires up ROS topics to SLAM.
    class Node
    {
    public:
        //  定义Node类，构造函数接受节点选项、地图构建器、TF缓冲区、ROS节点和是否收集指标的标志。
        Node(const NodeOptions &node_options,
             std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder,
             std::shared_ptr<tf2_ros::Buffer> tf_buffer,
             rclcpp::Node::SharedPtr node,
             bool collect_metrics);
        ~Node();

        Node(const Node &) = delete;            // 禁止拷贝构造函数
        Node &operator=(const Node &) = delete; // 禁止拷贝赋值运算符

        // Finishes all yet active trajectories.
        void FinishAllTrajectories(); // 结束所有活动的轨迹
        // Finishes a single given trajectory. Returns false if the trajectory did not
        // exist or was already finished.
        bool FinishTrajectory(int trajectory_id); // 结束指定的轨迹，如果轨迹不存在或已结束则返回false

        // Runs final optimization. All trajectories have to be finished when calling.
        void RunFinalOptimization(); // 运行最终优化，所有轨迹都必须先结束

        // Starts the first trajectory with the default topics.
        void StartTrajectoryWithDefaultTopics(const TrajectoryOptions &options); // 以默认主题启动轨迹

        // Returns unique SensorIds for multiple input bag files based on
        // their TrajectoryOptions.
        // 'SensorId::id' is the expected ROS topic name.
        std::vector<
            std::set<::cartographer::mapping::TrajectoryBuilderInterface::SensorId>>
        ComputeDefaultSensorIdsForMultipleBags(
            const std::vector<TrajectoryOptions> &bags_options) const; // 计算多个输入包文件的唯一传感器ID，基于它们的轨迹选项

        // Adds a trajectory for offline processing, i.e. not listening to topics.
        int AddOfflineTrajectory(
            const std::set<
                cartographer::mapping::TrajectoryBuilderInterface::SensorId> &
                expected_sensor_ids,
            const TrajectoryOptions &options); // 添加离线轨迹，不监听主题

        // The following functions handle adding sensor data to a trajectory.
        // 处理传感器数据的函数，添加到指定的轨迹中。分别处理不同类型的消息。
        void HandleOdometryMessage(int trajectory_id, const std::string &sensor_id,
                                   const nav_msgs::msg::Odometry::ConstSharedPtr &msg); // 处理里程计消息
        void HandleNavSatFixMessage(int trajectory_id, const std::string &sensor_id,
                                    const sensor_msgs::msg::NavSatFix::ConstSharedPtr &msg); // 处理导航卫星修复消息
        void HandleLandmarkMessage(
            int trajectory_id, const std::string &sensor_id,
            const cartographer_ros_msgs::msg::LandmarkList::ConstSharedPtr &msg); // 处理地标消息
        void HandleImuMessage(int trajectory_id, const std::string &sensor_id,
                              const sensor_msgs::msg::Imu::ConstSharedPtr &msg); // 处理IMU消息
        void HandleLaserScanMessage(int trajectory_id, const std::string &sensor_id,
                                    const sensor_msgs::msg::LaserScan::ConstSharedPtr &msg); // 处理激光扫描消息
        void HandleMultiEchoLaserScanMessage(
            int trajectory_id, const std::string &sensor_id,
            const sensor_msgs::msg::MultiEchoLaserScan::ConstSharedPtr &msg); // 处理多回波激光扫描消息
        void HandlePointCloud2Message(int trajectory_id, const std::string &sensor_id,
                                      const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg); // 处理点云2消息

        // Serializes the complete Node state.
        // 序列化和加载状态
        void SerializeState(const std::string &filename,
                            const bool include_unfinished_submaps); // 序列化节点状态到文件

        // Loads a serialized SLAM state from a .pbstream file.
        void LoadState(const std::string &state_filename, bool load_frozen_state); // 从.pbstream文件加载序列化的SLAM状态

    private:
        // 订阅者结构体，包含订阅者和主题名称
        struct Subscriber
        {
            rclcpp::SubscriptionBase::SharedPtr subscriber;

            // ::ros::Subscriber::getTopic() does not necessarily return the same
            // std::string
            // it was given in its constructor. Since we rely on the topic name as the
            // unique identifier of a subscriber, we remember it ourselves.
            std::string topic;
        };
        // 处理服务请求的函数
        bool handleSubmapQuery(
            const cartographer_ros_msgs::srv::SubmapQuery::Request::SharedPtr request,
            cartographer_ros_msgs::srv::SubmapQuery::Response::SharedPtr response); // 处理子图查询请求
        bool handleTrajectoryQuery(
            const cartographer_ros_msgs::srv::TrajectoryQuery::Request::SharedPtr request,
            cartographer_ros_msgs::srv::TrajectoryQuery::Response::SharedPtr response); // 处理轨迹查询请求
        bool handleStartTrajectory(
            const cartographer_ros_msgs::srv::StartTrajectory::Request::SharedPtr request,
            cartographer_ros_msgs::srv::StartTrajectory::Response::SharedPtr response); // 处理启动轨迹请求
        bool handleFinishTrajectory(
            const cartographer_ros_msgs::srv::FinishTrajectory::Request::SharedPtr request,
            cartographer_ros_msgs::srv::FinishTrajectory::Response::SharedPtr response); // 处理结束轨迹请求
        bool handleWriteState(
            const cartographer_ros_msgs::srv::WriteState::Request::SharedPtr request,
            cartographer_ros_msgs::srv::WriteState::Response::SharedPtr response); // 处理写入状态请求
        bool handleGetTrajectoryStates(
            const cartographer_ros_msgs::srv::GetTrajectoryStates::Request::SharedPtr,
            cartographer_ros_msgs::srv::GetTrajectoryStates::Response::SharedPtr response); // 处理获取轨迹状态请求
        bool handleReadMetrics(const cartographer_ros_msgs::srv::ReadMetrics::Request::SharedPtr,
                               cartographer_ros_msgs::srv::ReadMetrics::Response::SharedPtr response); // 处理读取指标请求

        // Returns the set of SensorIds expected for a trajectory.
        // 'SensorId::id' is the expected ROS topic name.
        // 计算轨迹的预期传感器ID集合
        std::set<::cartographer::mapping::TrajectoryBuilderInterface::SensorId>
        ComputeExpectedSensorIds(const TrajectoryOptions &options) const;
        int AddTrajectory(const TrajectoryOptions &options);                         // 添加轨迹，返回轨迹ID
        void LaunchSubscribers(const TrajectoryOptions &options, int trajectory_id); // 启动订阅者，监听指定的主题
        void PublishSubmapList();                                                    // 发布子图列表
        void AddExtrapolator(int trajectory_id, const TrajectoryOptions &options);   // 添加外推器，用于预测轨迹
        void AddSensorSamplers(int trajectory_id, const TrajectoryOptions &options); // 添加传感器采样器，用于从传感器获取数据
        void PublishLocalTrajectoryData();                                           // 发布本地轨迹数据
        void PublishTrajectoryNodeList();                                            // 发布轨迹节点列表
        void PublishLandmarkPosesList();                                             // 发布地标位姿列表
        void PublishConstraintList();                                                // 发布约束列表
        bool ValidateTrajectoryOptions(const TrajectoryOptions &options);            // 验证轨迹选项的有效性
        bool ValidateTopicNames(const TrajectoryOptions &options);                   // 验证主题名称的有效性
        cartographer_ros_msgs::msg::StatusResponse FinishTrajectoryUnderLock(
            int trajectory_id) EXCLUSIVE_LOCKS_REQUIRED(mutex_); // 在互斥锁下结束轨迹
        void MaybeWarnAboutTopicMismatch();                      // 检查主题不匹配并发出警告

        // Helper function for service handlers that need to check trajectory states.
        // 将给定轨迹ID的状态转换为状态响应消息
        cartographer_ros_msgs::msg::StatusResponse TrajectoryStateToStatus(
            int trajectory_id,                                                                       // 轨迹ID
            const std::set<cartographer::mapping::PoseGraphInterface::TrajectoryState> &valid_states // 有效状态集合
        );

        // 定义类的成员变量，包括节点选项、TF广播器、互斥锁、指标注册表、地图构建器桥接等。
        const NodeOptions node_options_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        absl::Mutex mutex_;
        std::unique_ptr<cartographer_ros::metrics::FamilyFactory> metrics_registry_;
        std::shared_ptr<MapBuilderBridge> map_builder_bridge_ GUARDED_BY(mutex_);

        rclcpp::Node::SharedPtr node_;                                                                          // ROS 2节点指针
        ::rclcpp::Publisher<::cartographer_ros_msgs::msg::SubmapList>::SharedPtr submap_list_publisher_;        // 子图列表发布器
        ::rclcpp::Publisher<::visualization_msgs::msg::MarkerArray>::SharedPtr trajectory_node_list_publisher_; // 轨迹节点列表发布器
        ::rclcpp::Publisher<::visualization_msgs::msg::MarkerArray>::SharedPtr landmark_poses_list_publisher_;  // 地标位姿列表发布器
        ::rclcpp::Publisher<::visualization_msgs::msg::MarkerArray>::SharedPtr constraint_list_publisher_;      // 约束列表发布器
        ::rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tracked_pose_publisher_;                // 跟踪位姿发布器
        ::rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_matched_point_cloud_publisher_;      // 扫描匹配点云发布器
        // These ros service servers need to live for the lifetime of the node.
        ::rclcpp::Service<cartographer_ros_msgs::srv::SubmapQuery>::SharedPtr submap_query_server_;                  // 子图查询服务
        ::rclcpp::Service<cartographer_ros_msgs::srv::TrajectoryQuery>::SharedPtr trajectory_query_server;           // 轨迹查询服务
        ::rclcpp::Service<cartographer_ros_msgs::srv::StartTrajectory>::SharedPtr start_trajectory_server_;          // 启动轨迹服务
        ::rclcpp::Service<cartographer_ros_msgs::srv::FinishTrajectory>::SharedPtr finish_trajectory_server_;        // 结束轨迹服务
        ::rclcpp::Service<cartographer_ros_msgs::srv::WriteState>::SharedPtr write_state_server_;                    // 写入状态服务
        ::rclcpp::Service<cartographer_ros_msgs::srv::GetTrajectoryStates>::SharedPtr get_trajectory_states_server_; // 获取轨迹状态服务
        ::rclcpp::Service<cartographer_ros_msgs::srv::ReadMetrics>::SharedPtr read_metrics_server_;                  // 读取指标服务

        // These are used to sample sensor data at a fixed rate.
        // The sampling ratio is the fraction of messages that are passed to SLAM.
        // 传感器采样器，按固定比例采样传感器数据
        // 采样器的比例是传递给SLAM的消息的比例。
        // 这些采样器用于从传感器获取数据，按固定比例采样。
        struct TrajectorySensorSamplers
        {
            TrajectorySensorSamplers(const double rangefinder_sampling_ratio,
                                     const double odometry_sampling_ratio,
                                     const double fixed_frame_pose_sampling_ratio,
                                     const double imu_sampling_ratio,
                                     const double landmark_sampling_ratio)
                : rangefinder_sampler(rangefinder_sampling_ratio),
                  odometry_sampler(odometry_sampling_ratio),
                  fixed_frame_pose_sampler(fixed_frame_pose_sampling_ratio),
                  imu_sampler(imu_sampling_ratio),
                  landmark_sampler(landmark_sampling_ratio) {}

            ::cartographer::common::FixedRatioSampler rangefinder_sampler;      // 激光雷达采样器
            ::cartographer::common::FixedRatioSampler odometry_sampler;         // 里程计采样器
            ::cartographer::common::FixedRatioSampler fixed_frame_pose_sampler; // 固定帧位姿采样器
            ::cartographer::common::FixedRatioSampler imu_sampler;              // IMU采样器
            ::cartographer::common::FixedRatioSampler landmark_sampler;         // 地标采样器
        };

        // These are keyed with 'trajectory_id'.
        std::map<int, ::cartographer::mapping::PoseExtrapolator> extrapolators_; // 位姿外推器，用于估计轨迹中的位姿
        std::map<int, builtin_interfaces::msg::Time> last_published_tf_stamps_;  // 上次发布TF变换的时间戳，用于避免重复发布
        std::unordered_map<int, TrajectorySensorSamplers> sensor_samplers_;      // 传感器采样器，用于按固定比例采样传感器数据
        std::unordered_map<int, std::vector<Subscriber>> subscribers_;           // 订阅者，用于接收传感器数据
        std::unordered_set<std::string> subscribed_topics_;                      // 已订阅的主题，用于避免重复订阅
        std::unordered_set<int> trajectories_scheduled_for_finish_;              // 已计划结束的轨迹ID，用于避免重复结束

        // The timer for publishing local trajectory data (i.e. pose transforms and
        // range data point clouds) is a regular timer which is not triggered when
        // simulation time is standing still. This prevents overflowing the transform
        // listener buffer by publishing the same transforms over and over again.
        ::rclcpp::TimerBase::SharedPtr submap_list_timer_;                     // 子图列表发布定时器
        ::rclcpp::TimerBase::SharedPtr local_trajectory_data_timer_;           // 本地轨迹数据发布定时器
        ::rclcpp::TimerBase::SharedPtr trajectory_node_list_timer_;            // 轨迹节点列表发布定时器
        ::rclcpp::TimerBase::SharedPtr landmark_pose_list_timer_;              // 地标位姿列表发布定时器
        ::rclcpp::TimerBase::SharedPtr constrain_list_timer_;                  // 约束列表发布定时器
        ::rclcpp::TimerBase::SharedPtr maybe_warn_about_topic_mismatch_timer_; // 主题不匹配警告定时器
    };

} // namespace cartographer_ros

#endif // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_H
