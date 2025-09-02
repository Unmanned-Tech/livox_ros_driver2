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

#include "driver_node.h"
#include "lddc.h"

#include <mutex>
#include <condition_variable>
#include <chrono>
#include "livox_lidar_api.h"
#include "livox_lidar_def.h"

namespace livox_ros {

void DriverNode::OnSetWorkMode(
  const std::shared_ptr<livox_ros_driver2::srv::SetWorkMode::Request> request,
  std::shared_ptr<livox_ros_driver2::srv::SetWorkMode::Response> response) {

    RCLCPP_INFO(this->get_logger(),
                "Запрос смены режима: handle=%u, mode=%u",
                request->handle, request->mode);

    bool found = false;
    for (uint8_t i = 0; i < lddc_ptr_->lds_->lidar_count_; ++i) {
      if (lddc_ptr_->lds_->lidars_[i].handle == request->handle) {
        found = true;
        break;
      }
    }
    if (!found) {
      RCLCPP_ERROR(this->get_logger(), "Лидар с handle=%u не найден", request->handle);
      response->success = false;
      response->message = "Lidar with handle=%u not found";
      return;
    }

    switch (request->mode) {
        case kLivoxLidarNormal:
        case kLivoxLidarWakeUp:
        case kLivoxLidarSleep:
        case kLivoxLidarPowerOnSelfTest:
        case kLivoxLidarMotorStarting:
        case kLivoxLidarMotorStoping:
        case kLivoxLidarUpgrade:
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Invalid work mode: %u", request->mode);
            response->success = false;
            response->message = "Invalid work mode: %u";
            return;
    }

    struct Context {
      std::mutex mtx;
      std::condition_variable cv;
      bool success = false;
      bool done = false;
    };

    Context ctx;

    auto callback = [](livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse* response, void* client_data) {
      Context* ctx = static_cast<Context*>(client_data);
      if (ctx) {
        std::lock_guard<std::mutex> lock(ctx->mtx);
        ctx->success = (status == kLivoxLidarStatusSuccess);
        ctx->done = true;
        ctx->cv.notify_one();
      }
    };

    livox_status ret = SetLivoxLidarWorkMode(
      request->handle,
      static_cast<LivoxLidarWorkMode>(request->mode),  // Приводим uint8 к enum
      callback,
      &ctx
    );

    if (ret != kLivoxLidarStatusSuccess) {
      RCLCPP_ERROR(this->get_logger(), "Ошибка вызова SetLivoxLidarWorkMode: status=%d", ret);
      response->success = false;
      return;
    }
    
    std::unique_lock<std::mutex> lock(ctx.mtx);
    bool timed_out = !ctx.cv.wait_for(lock, std::chrono::seconds(5), [&ctx] { return ctx.done; });

    if (timed_out) {
        RCLCPP_ERROR(this->get_logger(), "Таймаут ожидания ответа от лидара для handle=%u", request->handle);
        response->success = false;
    } else {
        response->success = ctx.success;
        if (ctx.success) {
            RCLCPP_INFO(this->get_logger(), "Режим работы успешно изменен для handle=%u", request->handle);

            {
              std::lock_guard<std::mutex> wm_lock(work_mode_mtx_);
              lidar_work_modes_[request->handle] = request->mode;
            }

            response->message = "Успешный успех";
        } else {
            RCLCPP_ERROR(this->get_logger(), "Ошибка изменения режима работы для handle=%u", request->handle);
        }
    }
}

DriverNode& DriverNode::GetNode() noexcept {
  return *this;
}

std::string GetWorkModeName(uint8_t mode) {
  switch(mode) {
    case kLivoxLidarNormal: return "Normal";
    case kLivoxLidarWakeUp: return "WakeUp";
    case kLivoxLidarSleep: return "Sleep";
    case kLivoxLidarPowerOnSelfTest: return "PowerOnSelfTest";
    case kLivoxLidarMotorStarting: return "MotorStarting";
    case kLivoxLidarMotorStoping: return "MotorStopping";
    case kLivoxLidarUpgrade: return "Upgrade";
    default: return "Unknown";
  }
}

void DriverNode::WorkModeStatusPollThread() {
  std::future_status status;
  while (true) {
    if (lddc_ptr_ && lddc_ptr_->lds_) {

      std::lock_guard<std::mutex> lock(work_mode_mtx_);

      for (uint8_t i = 0; i < lddc_ptr_->lds_->lidar_count_; ++i) {
        uint32_t handle = lddc_ptr_->lds_->lidars_[i].handle;
        auto it = lidar_work_modes_.find(handle);
        if (it != lidar_work_modes_.end()) {
          auto msg = livox_ros_driver2::msg::LidarStatus();
          msg.handle = handle;
          msg.mode = it->second;
          msg.mode_name = GetWorkModeName(it->second);
          msg.valid = true;
          work_mode_status_pub_->publish(msg);

          RCLCPP_INFO(this->get_logger(), "Published work mode status: handle=%u, mode=%u (%s)",
                      handle, msg.mode, msg.mode_name.c_str());
        }
      }
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
    status = future_.wait_for(std::chrono::microseconds(0));
    if (status != std::future_status::timeout) {
      break;
    }
  }
}

void DriverNode::SendStandby() {
  if (!lddc_ptr_ || !lddc_ptr_->lds_) return;

  std::lock_guard<std::mutex> lock(work_mode_mtx_);

  for (uint8_t i = 0; i < lddc_ptr_->lds_->lidar_count_; ++i) {
    uint32_t handle = lddc_ptr_->lds_->lidars_[i].handle;

    struct Context {
      std::mutex mtx;
      std::condition_variable cv;
      bool done = false;
    } ctx;

    auto callback = [](livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse* response, void* client_data) {
      Context* ctx = static_cast<Context*>(client_data);
      if (ctx) {
        std::lock_guard<std::mutex> lock(ctx->mtx);
        ctx->done = true;
        ctx->cv.notify_one();
      }
    };

    SetLivoxLidarWorkMode(handle, kLivoxLidarWakeUp, callback, &ctx);

    std::unique_lock<std::mutex> ul(ctx.mtx);
    ctx.cv.wait_for(ul, std::chrono::seconds(3), [&ctx]{ return ctx.done; });

    lidar_work_modes_[handle] = kLivoxLidarWakeUp;
    auto msg = livox_ros_driver2::msg::LidarStatus();
    msg.handle = handle;
    msg.mode = kLivoxLidarWakeUp;
    msg.mode_name = GetWorkModeName(kLivoxLidarWakeUp);
    msg.valid = true;
    work_mode_status_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Send Standby to handle=%u", handle);
  }
}

DriverNode::~DriverNode() {
  SendStandby();
  lddc_ptr_->lds_->RequestExit();
  exit_signal_.set_value();
  pointclouddata_poll_thread_->join();
  imudata_poll_thread_->join();
}

} // namespace livox_ros





