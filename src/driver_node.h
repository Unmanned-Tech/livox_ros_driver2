//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#ifndef LIVOX_DRIVER_NODE_H
#define LIVOX_DRIVER_NODE_H

#include "include/ros_headers.h"
#include "livox_ros_driver2/srv/set_work_mode.hpp"
#include "livox_ros_driver2/msg/lidar_status.hpp"

namespace livox_ros {

class Lddc;

#ifdef BUILDING_ROS1
class DriverNode final : public ros::NodeHandle {
 public:
  DriverNode() = default;
  DriverNode(const DriverNode &) = delete;
  ~DriverNode();
  DriverNode &operator=(const DriverNode &) = delete;

  DriverNode& GetNode() noexcept;

  void PointCloudDataPollThread();
  void ImuDataPollThread();

  std::unique_ptr<Lddc> lddc_ptr_;
  std::shared_ptr<std::thread> pointclouddata_poll_thread_;
  std::shared_ptr<std::thread> imudata_poll_thread_;
  std::shared_future<void> future_;
  std::promise<void> exit_signal_;
};

#elif defined BUILDING_ROS2
class DriverNode final : public rclcpp::Node {
 public:
  explicit DriverNode(const rclcpp::NodeOptions& options);
  DriverNode(const DriverNode &) = delete;
  ~DriverNode();
  DriverNode &operator=(const DriverNode &) = delete;

  DriverNode& GetNode() noexcept;

 private:
  void PointCloudDataPollThread();
  void ImuDataPollThread();
  void WorkModeStatusPollThread();
  void SendStandby();

  void OnSetWorkMode(
    const std::shared_ptr<livox_ros_driver2::srv::SetWorkMode::Request> request,
    std::shared_ptr<livox_ros_driver2::srv::SetWorkMode::Response> response);

  rclcpp::Service<livox_ros_driver2::srv::SetWorkMode>::SharedPtr set_work_mode_srv_;
  rclcpp::Publisher<livox_ros_driver2::msg::LidarStatus>::SharedPtr work_mode_status_pub_;
  std::shared_ptr<std::thread> work_mode_status_poll_thread_;
  std::map<uint32_t, uint8_t> lidar_work_modes_;
  std::mutex work_mode_mtx_;

  std::unique_ptr<Lddc> lddc_ptr_;
  std::shared_ptr<std::thread> pointclouddata_poll_thread_;
  std::shared_ptr<std::thread> imudata_poll_thread_;
  std::shared_future<void> future_;
  std::promise<void> exit_signal_;
};
#endif

} // namespace livox_ros

#endif // LIVOX_DRIVER_NODE_H