// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025 Yao Yuzhuo (yaoyuzhuo6@gmail.com)

#include <csignal>
#include <cstdlib>
#include <cstring>

#include <rclcpp/rclcpp.hpp>

#include "calibration_validator.h"

namespace {

std::atomic<bool> g_shutdown_requested{false};

void signalHandler(int sig) {
  (void)sig;
  g_shutdown_requested.store(true, std::memory_order_release);
  rclcpp::shutdown();
}

void printStartupInfo(const rclcpp::Logger& logger) {
  RCLCPP_INFO(logger, "========================================");
  RCLCPP_INFO(logger, "  LiDAR-Camera Calibration Validator   ");
  RCLCPP_INFO(logger, "========================================");
  RCLCPP_INFO(logger, "Version: 5.0.0 (ROS2)");
  RCLCPP_INFO(logger, "Features:");
  RCLCPP_INFO(logger, "  - Real-time LiDAR-camera fusion visualization");
  RCLCPP_INFO(logger, "  - Quantitative calibration quality metrics");
  RCLCPP_INFO(logger, "  - Runtime parameter adjustment via ROS2 params");
  RCLCPP_INFO(logger, "  - Edge overlap score and absolute gray-depth Pearson correlation");
  RCLCPP_INFO(logger, "========================================");
}

bool checkEnvironment(const rclcpp::Logger& logger) {
  const char* display = std::getenv("DISPLAY");
  if (!display || std::strlen(display) == 0) {
    RCLCPP_WARN(logger, "[ValidatorNode] No DISPLAY environment variable found.");
    RCLCPP_WARN(logger, "[ValidatorNode] OpenCV windows may not display properly.");
  }
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  auto node = rclcpp::Node::make_shared("validator_node");

  printStartupInfo(node->get_logger());
  if (!checkEnvironment(node->get_logger())) {
    RCLCPP_ERROR(node->get_logger(), "[ValidatorNode] Environment check failed");
    return -1;
  }

  try {
    RCLCPP_INFO(node->get_logger(), "[ValidatorNode] Initializing CalibrationValidator...");
    lidar_cam_validator::CalibrationValidator validator(node);

    RCLCPP_INFO(node->get_logger(), "[ValidatorNode] CalibrationValidator initialized successfully");
    RCLCPP_INFO(node->get_logger(), "[ValidatorNode] Waiting for image and point cloud data...");

    node->declare_parameter<double>("loop_rate", 30.0);
    const double loop_rate_hz = node->get_parameter("loop_rate").as_double();
    rclcpp::Rate loop_rate(loop_rate_hz);

    while (rclcpp::ok() && !g_shutdown_requested.load(std::memory_order_acquire)) {
      rclcpp::spin_some(node);

      double since_last = (node->now() - validator.getLastDataTime()).seconds();
      if (since_last > 10.0) {
        RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 10000,
                             "[ValidatorNode] No image/cloud message for %.1fs. Check topics & timestamps.",
                             since_last);
      }

      loop_rate.sleep();
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "[ValidatorNode] Exception caught: %s", e.what());
    return -1;
  } catch (...) {
    RCLCPP_ERROR(node->get_logger(), "[ValidatorNode] Unknown exception caught");
    return -1;
  }

  cv::destroyAllWindows();
  return 0;
}
