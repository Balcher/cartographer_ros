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

#include "absl/memory/memory.h"                // Google的Abseil库，用于内存管理
#include "cartographer/mapping/map_builder.h"  // Cartographer的地图构建器
#include "cartographer_ros/node.h"             // Cartographer ROS节点
#include "cartographer_ros/node_options.h"     // Cartographer ROS节点选项
#include "cartographer_ros/ros_log_sink.h"     // Cartographer ROS日志接收器
#include "gflags/gflags.h"                     // Google的命令行标志库
#include "rclcpp/rclcpp.hpp"                   // ROS 2 C++客户端库
#include "tf2_ros/transform_listener.h"        // ROS 2变换监听器

// 定义命令行标志
// 这里定义了一些命令行参数，用于控制节点的行为，如是否收集运行时指标、配置文件的目录和名称、加载和保存状态的文件名等。
DEFINE_bool(collect_metrics, false,
            "Activates the collection of runtime metrics. If activated, the "
            "metrics can be accessed via a ROS service.");
DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(load_state_filename, "",
              "If non-empty, filename of a .pbstream file to load, containing "
              "a saved SLAM state.");
DEFINE_bool(load_frozen_state, true,
            "Load the saved state as frozen (non-optimized) trajectories.");
DEFINE_bool(
    start_trajectory_with_default_topics, true,
    "Enable to immediately start the first trajectory with default topics.");
DEFINE_string(
    save_state_filename, "",
    "If non-empty, serialize state and write it to disk before shutting down.");

namespace cartographer_ros {
namespace {

void Run() {
  // 创建一个ROS 2节点，设置TF缓冲区和监听器
  rclcpp::Node::SharedPtr cartographer_node =
      rclcpp::Node::make_shared("cartographer_node");
  constexpr double kTfBufferCacheTimeInSeconds = 10.;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer =
      std::make_shared<tf2_ros::Buffer>(
          cartographer_node->get_clock(),
          tf2::durationFromSec(kTfBufferCacheTimeInSeconds), cartographer_node);

  std::shared_ptr<tf2_ros::TransformListener> tf_listener =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  // 加载节点选项和轨迹选项
  NodeOptions node_options;
  TrajectoryOptions trajectory_options;
  std::tie(node_options, trajectory_options) =
      LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);

  // 创建地图构建器
  auto map_builder =
      cartographer::mapping::CreateMapBuilder(node_options.map_builder_options);

  // 节点初始化
  auto node = std::make_shared<cartographer_ros::Node>(
      node_options, std::move(map_builder), tf_buffer, cartographer_node,
      FLAGS_collect_metrics);

  // 加载状态文件（如果指定）
  if (!FLAGS_load_state_filename.empty()) {
    node->LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);
  }
  // 启动轨迹，根据设置决定是否立即启动轨迹
  if (FLAGS_start_trajectory_with_default_topics) {
    node->StartTrajectoryWithDefaultTopics(trajectory_options);
  }

  //  spinning 循环，调用ROS 2的spin函数来处理回调和事件
  rclcpp::spin(cartographer_node);

  node->FinishAllTrajectories();  // 结束所有轨迹
  node->RunFinalOptimization();   // 运行最终优化

  // 保存状态文件（如果指定）
  if (!FLAGS_save_state_filename.empty()) {
    node->SerializeState(FLAGS_save_state_filename,
                         true /* include_unfinished_submaps */);
  }
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  // Init rclcpp first because gflags reorders command line flags in argv
  rclcpp::init(argc, argv);  // 初始化ros2

  // 初始化Google的gflags库和日志记录
  google::AllowCommandLineReparsing();
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);

  // 检查配置目录和配置文件名是否提供
  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";

  // 创建一个ScopedRosLogSink实例，用于将Cartographer的日志输出到ROS日志系统
  cartographer_ros::ScopedRosLogSink ros_log_sink;
  cartographer_ros::Run();  // 调用Run函数来启动Cartographer ROS节点
  ::rclcpp::shutdown();
}
