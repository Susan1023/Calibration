// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025 Yao Yuzhuo (yaoyuzhuo6@gmail.com)

#include "calibration_validator.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <builtin_interfaces/msg/time.hpp>
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <sensor_msgs/image_encodings.hpp>
#include <rmw/qos_profiles.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <limits>
#include <numeric>
#include <regex>
#include <tbb/blocked_range.h>
#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>

namespace lidar_cam_validator {

namespace {
constexpr int kMinRenderRateHz = 1;
constexpr int kMaxRenderRateHz = 120;
constexpr double kMinCacheTtlSec = 0.01;

int clampRenderRateHz(int value) {
  return std::max(kMinRenderRateHz, std::min(kMaxRenderRateHz, value));
}
}  // namespace

CalibrationValidator::CalibrationValidator(const rclcpp::Node::SharedPtr& node)
    : node_(node), last_data_time_(node_->now()) {
  projected_points_cache_.reserve(200000);
  depths_cache_.reserve(200000);
  colors_cache_.reserve(200000);
  last_process_time_ = std::chrono::steady_clock::now();

  declareAndLoadParameters();

  if (!loadCalibrationParameters()) {
    RCLCPP_ERROR(node_->get_logger(), "[CalibrationValidator] Failed to load calibration parameters");
  }

  recreateFusedImagePublisher();
  validation_info_pub_ = node_->create_publisher<std_msgs::msg::String>(info_topic_, 10);

  if (use_latest_sync_) {
    setupLatestSync();
    RCLCPP_INFO(node_->get_logger(), "[CalibrationValidator] Sync mode: latest-pair (threshold %.2fs)",
                latest_pair_time_threshold_);
  } else {
    setupApproximateSync();
    RCLCPP_INFO(node_->get_logger(), "[CalibrationValidator] Sync mode: approximate time synchronizer");
  }

  params_callback_handle_ = node_->add_on_set_parameters_callback(
      std::bind(&CalibrationValidator::onParameterUpdate, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(), "[CalibrationValidator] Initialized successfully");
  RCLCPP_INFO(node_->get_logger(), "[CalibrationValidator] Subscribed to image: %s", image_topic_.c_str());
  RCLCPP_INFO(node_->get_logger(), "[CalibrationValidator] Subscribed to cloud: %s", cloud_topic_.c_str());
  RCLCPP_INFO(node_->get_logger(), "[CalibrationValidator] Publishing fused image: %s", fused_topic_.c_str());
}

CalibrationValidator::~CalibrationValidator() {
  if (window_initialized_) cv::destroyAllWindows();
}

void CalibrationValidator::declareAndLoadParameters() {
  node_->declare_parameter<std::string>("image_topic", "/image_raw");
  node_->declare_parameter<std::string>("cloud_topic", "/cloudpoints");
  node_->declare_parameter<std::string>("fused_topic", "/validator/fused_image");
  node_->declare_parameter<std::string>("fused_image_qos_reliability", "best_effort");
  node_->declare_parameter<std::string>("info_topic", "/validator/validation_info");
  node_->declare_parameter<std::string>("calibration_file",
                                        "$(find lidar_cam_validator)/config/sample_calibration.yaml");

  node_->declare_parameter<std::string>("sync_mode", "approximate");
  node_->declare_parameter<double>("latest_pair_time_threshold", 0.5);
  node_->declare_parameter<int>("queue_size", 10);
  node_->declare_parameter<int>("render_rate_hz", 30);
  node_->declare_parameter<double>("projection_cache_ttl_sec", 0.1);
  node_->declare_parameter<int>("metrics_compute_interval", 3);

  node_->declare_parameter<int>("point_size", 3);
  node_->declare_parameter<double>("min_depth", 0.5);
  node_->declare_parameter<double>("max_depth", 50.0);
  node_->declare_parameter<bool>("show_statistics", true);
  node_->declare_parameter<bool>("show_edge_overlay", false);
  node_->declare_parameter<bool>("show_depth_colorbar", true);
  show_window_ = node_->declare_parameter<bool>("show_window", true);

  node_->declare_parameter<bool>("enable_downsampling", false);
  node_->declare_parameter<int>("max_points", 1000000);

  node_->declare_parameter<bool>("enable_accumulation", false);
  node_->declare_parameter<double>("accumulation_time_sec", 0.1);
  node_->declare_parameter<int>("accumulation_frames", 3);
  node_->declare_parameter<int>("accumulation_max_points", 200000);

  node_->declare_parameter<bool>("filter_by_distance", true);
  node_->declare_parameter<double>("min_distance", 0.3);
  node_->declare_parameter<double>("max_distance", 100.0);

  node_->declare_parameter<bool>("enable_metrics", true);
  node_->declare_parameter<int>("edge_threshold", 50);
  node_->declare_parameter<double>("edge_distance_threshold", 5.0);
  node_->declare_parameter<int>("min_points_for_nmi", 100);
  node_->declare_parameter<bool>("reset_to_defaults", false);

  node_->get_parameter("image_topic", image_topic_);
  node_->get_parameter("cloud_topic", cloud_topic_);
  node_->get_parameter("fused_topic", fused_topic_);
  node_->get_parameter("fused_image_qos_reliability", fused_image_qos_reliability_);
  node_->get_parameter("info_topic", info_topic_);
  node_->get_parameter("calibration_file", calibration_file_);
  node_->get_parameter("sync_mode", sync_mode_);
  node_->get_parameter("latest_pair_time_threshold", latest_pair_time_threshold_);
  node_->get_parameter("render_rate_hz", render_rate_hz_);
  node_->get_parameter("projection_cache_ttl_sec", projection_cache_ttl_sec_);
  node_->get_parameter("metrics_compute_interval", metrics_compute_interval_);

  render_rate_hz_ = clampRenderRateHz(render_rate_hz_);
  projection_cache_ttl_sec_ = std::max(kMinCacheTtlSec, projection_cache_ttl_sec_);
  metrics_compute_interval_ = std::max(1, metrics_compute_interval_);

  std::transform(sync_mode_.begin(), sync_mode_.end(), sync_mode_.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  use_latest_sync_ = (sync_mode_ == "latest" || sync_mode_ == "latest_pair");

  updateRuntimeConfigFromNode();
}

void CalibrationValidator::updateRuntimeConfigFromNode() {
  std::lock_guard<std::mutex> lock(config_mutex_);

  node_->get_parameter("point_size", config_.point_size);
  node_->get_parameter("min_depth", config_.min_depth);
  node_->get_parameter("max_depth", config_.max_depth);
  node_->get_parameter("show_statistics", config_.show_statistics);
  node_->get_parameter("show_edge_overlay", config_.show_edge_overlay);
  node_->get_parameter("show_depth_colorbar", config_.show_depth_colorbar);

  node_->get_parameter("enable_downsampling", config_.enable_downsampling);
  node_->get_parameter("max_points", config_.max_points);

  node_->get_parameter("enable_accumulation", config_.enable_accumulation);
  node_->get_parameter("accumulation_time_sec", config_.accumulation_time_sec);
  node_->get_parameter("accumulation_frames", config_.accumulation_frames);
  node_->get_parameter("accumulation_max_points", config_.accumulation_max_points);

  node_->get_parameter("filter_by_distance", config_.filter_by_distance);
  node_->get_parameter("min_distance", config_.min_distance);
  node_->get_parameter("max_distance", config_.max_distance);

  node_->get_parameter("enable_metrics", config_.enable_metrics);
  node_->get_parameter("edge_threshold", config_.edge_threshold);
  node_->get_parameter("edge_distance_threshold", config_.edge_distance_threshold);
  node_->get_parameter("min_points_for_nmi", config_.min_points_for_nmi);
  node_->get_parameter("reset_to_defaults", config_.reset_to_defaults);

  int metrics_interval = metrics_compute_interval_;
  double cache_ttl = projection_cache_ttl_sec_;
  int render_rate = render_rate_hz_;
  node_->get_parameter("metrics_compute_interval", metrics_interval);
  node_->get_parameter("projection_cache_ttl_sec", cache_ttl);
  node_->get_parameter("render_rate_hz", render_rate);
  metrics_compute_interval_ = std::max(1, metrics_interval);
  projection_cache_ttl_sec_ = std::max(kMinCacheTtlSec, cache_ttl);
  render_rate_hz_ = clampRenderRateHz(render_rate);
}

rcl_interfaces::msg::SetParametersResult CalibrationValidator::onParameterUpdate(
    const std::vector<rclcpp::Parameter>& parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "accepted";

  std::string new_image_topic = image_topic_;
  std::string new_cloud_topic = cloud_topic_;
  std::string new_fused_topic = fused_topic_;
  std::string new_fused_qos = fused_image_qos_reliability_;
  std::string new_info_topic = info_topic_;
  std::string new_sync_mode = sync_mode_;
  std::string new_calib_file = calibration_file_;
  double new_latest_pair_time_threshold = latest_pair_time_threshold_;
  int new_render_rate_hz = render_rate_hz_;
  double new_projection_cache_ttl_sec = projection_cache_ttl_sec_;
  int new_metrics_compute_interval = metrics_compute_interval_;
  bool reset_requested = false;
  RuntimeConfig updated_config;
  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    updated_config = config_;
  }

  for (const auto& param : parameters) {
    const auto& name = param.get_name();
    if (name == "image_topic") {
      new_image_topic = param.as_string();
    } else if (name == "cloud_topic") {
      new_cloud_topic = param.as_string();
    } else if (name == "fused_topic") {
      new_fused_topic = param.as_string();
    } else if (name == "fused_image_qos_reliability") {
      new_fused_qos = param.as_string();
    } else if (name == "info_topic") {
      new_info_topic = param.as_string();
    } else if (name == "sync_mode") {
      new_sync_mode = param.as_string();
    } else if (name == "latest_pair_time_threshold") {
      new_latest_pair_time_threshold = param.as_double();
    } else if (name == "calibration_file") {
      new_calib_file = param.as_string();
    } else if (name == "render_rate_hz") {
      new_render_rate_hz = param.as_int();
    } else if (name == "projection_cache_ttl_sec") {
      new_projection_cache_ttl_sec = param.as_double();
    } else if (name == "metrics_compute_interval") {
      new_metrics_compute_interval = param.as_int();
    } else if (name == "point_size") {
      updated_config.point_size = param.as_int();
    } else if (name == "min_depth") {
      updated_config.min_depth = param.as_double();
    } else if (name == "max_depth") {
      updated_config.max_depth = param.as_double();
    } else if (name == "show_statistics") {
      updated_config.show_statistics = param.as_bool();
    } else if (name == "show_edge_overlay") {
      updated_config.show_edge_overlay = param.as_bool();
    } else if (name == "show_depth_colorbar") {
      updated_config.show_depth_colorbar = param.as_bool();
    } else if (name == "enable_downsampling") {
      updated_config.enable_downsampling = param.as_bool();
    } else if (name == "max_points") {
      updated_config.max_points = param.as_int();
    } else if (name == "enable_accumulation") {
      updated_config.enable_accumulation = param.as_bool();
    } else if (name == "accumulation_time_sec") {
      updated_config.accumulation_time_sec = param.as_double();
    } else if (name == "accumulation_frames") {
      updated_config.accumulation_frames = param.as_int();
    } else if (name == "accumulation_max_points") {
      updated_config.accumulation_max_points = param.as_int();
    } else if (name == "filter_by_distance") {
      updated_config.filter_by_distance = param.as_bool();
    } else if (name == "min_distance") {
      updated_config.min_distance = param.as_double();
    } else if (name == "max_distance") {
      updated_config.max_distance = param.as_double();
    } else if (name == "enable_metrics") {
      updated_config.enable_metrics = param.as_bool();
    } else if (name == "edge_threshold") {
      updated_config.edge_threshold = param.as_int();
    } else if (name == "edge_distance_threshold") {
      updated_config.edge_distance_threshold = param.as_double();
    } else if (name == "min_points_for_nmi") {
      updated_config.min_points_for_nmi = param.as_int();
    } else if (name == "reset_to_defaults" && param.as_bool()) {
      reset_requested = true;
    }
  }

  std::transform(new_sync_mode.begin(), new_sync_mode.end(), new_sync_mode.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (new_sync_mode != "approximate" && new_sync_mode != "latest" && new_sync_mode != "latest_pair") {
    RCLCPP_WARN(node_->get_logger(),
                "[CalibrationValidator] Invalid sync_mode='%s', fallback to current '%s'",
                new_sync_mode.c_str(), sync_mode_.c_str());
    new_sync_mode = sync_mode_;
  }

  new_render_rate_hz = clampRenderRateHz(new_render_rate_hz);
  new_projection_cache_ttl_sec = std::max(kMinCacheTtlSec, new_projection_cache_ttl_sec);
  new_metrics_compute_interval = std::max(1, new_metrics_compute_interval);

  bool topics_changed = (new_image_topic != image_topic_ || new_cloud_topic != cloud_topic_);
  bool sync_changed = (new_sync_mode != sync_mode_ ||
                       new_latest_pair_time_threshold != latest_pair_time_threshold_);
  bool threshold_changed = (new_latest_pair_time_threshold != latest_pair_time_threshold_);
  bool use_latest_before = use_latest_sync_;
  bool should_use_latest = (new_sync_mode == "latest" || new_sync_mode == "latest_pair");
  if (should_use_latest != use_latest_before) {
    sync_changed = true;
  }

  image_topic_ = new_image_topic;
  cloud_topic_ = new_cloud_topic;
  sync_mode_ = new_sync_mode;
  latest_pair_time_threshold_ = new_latest_pair_time_threshold;
  use_latest_sync_ = should_use_latest;
  if (threshold_changed) {
    arrival_gap_warned_.store(false, std::memory_order_release);
  }

  if (topics_changed || sync_changed) {
    if (use_latest_sync_) {
      setupLatestSync();
    } else {
      setupApproximateSync();
    }
  }

  if (new_fused_topic != fused_topic_ || new_fused_qos != fused_image_qos_reliability_) {
    fused_topic_ = new_fused_topic;
    fused_image_qos_reliability_ = new_fused_qos;
    recreateFusedImagePublisher();
  }

  if (new_info_topic != info_topic_) {
    info_topic_ = new_info_topic;
    validation_info_pub_ = node_->create_publisher<std_msgs::msg::String>(info_topic_, 10);
  }

  if (new_calib_file != calibration_file_) {
    calibration_file_ = new_calib_file;
    loadCalibrationParameters();
  }

  int old_render_rate = render_rate_hz_;
  render_rate_hz_ = new_render_rate_hz;
  projection_cache_ttl_sec_ = new_projection_cache_ttl_sec;
  metrics_compute_interval_ = new_metrics_compute_interval;

  if (use_latest_sync_ && old_render_rate != render_rate_hz_) {
    render_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(1000 / render_rate_hz_),
        std::bind(&CalibrationValidator::renderLatestFrame, this));
  }

  updated_config.reset_to_defaults = reset_requested;
  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    config_ = updated_config;
  }

  if (reset_requested) {
    reset_defaults_requested_.store(true, std::memory_order_release);
  }

  auto equals_double = [](double a, double b) {
    return std::abs(a - b) < 1e-9;
  };
  RuntimeConfig cfg_snapshot;
  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    cfg_snapshot = config_;
  }
  for (const auto& param : parameters) {
    const auto& name = param.get_name();
    bool applied = true;
    if (name == "image_topic") {
      applied = (image_topic_ == param.as_string());
    } else if (name == "cloud_topic") {
      applied = (cloud_topic_ == param.as_string());
    } else if (name == "fused_topic") {
      applied = (fused_topic_ == param.as_string());
    } else if (name == "fused_image_qos_reliability") {
      std::string qos = param.as_string();
      std::transform(qos.begin(), qos.end(), qos.begin(),
                     [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
      applied = (fused_image_qos_reliability_ == qos || fused_image_qos_reliability_ == "best_effort");
    } else if (name == "info_topic") {
      applied = (info_topic_ == param.as_string());
    } else if (name == "sync_mode") {
      std::string mode = param.as_string();
      std::transform(mode.begin(), mode.end(), mode.begin(),
                     [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
      if (mode != "approximate" && mode != "latest" && mode != "latest_pair") {
        applied = true;
      } else {
        applied = (sync_mode_ == mode);
      }
    } else if (name == "latest_pair_time_threshold") {
      applied = equals_double(latest_pair_time_threshold_, param.as_double());
    } else if (name == "calibration_file") {
      applied = (calibration_file_ == param.as_string());
    } else if (name == "render_rate_hz") {
      applied = (render_rate_hz_ == clampRenderRateHz(param.as_int()));
    } else if (name == "projection_cache_ttl_sec") {
      applied = equals_double(projection_cache_ttl_sec_,
                              std::max(kMinCacheTtlSec, param.as_double()));
    } else if (name == "metrics_compute_interval") {
      applied = (metrics_compute_interval_ ==
                 std::max(1, static_cast<int>(param.as_int())));
    } else if (name == "point_size") {
      applied = (cfg_snapshot.point_size == param.as_int());
    } else if (name == "min_depth") {
      applied = equals_double(cfg_snapshot.min_depth, param.as_double());
    } else if (name == "max_depth") {
      applied = equals_double(cfg_snapshot.max_depth, param.as_double());
    } else if (name == "show_statistics") {
      applied = (cfg_snapshot.show_statistics == param.as_bool());
    } else if (name == "show_edge_overlay") {
      applied = (cfg_snapshot.show_edge_overlay == param.as_bool());
    } else if (name == "show_depth_colorbar") {
      applied = (cfg_snapshot.show_depth_colorbar == param.as_bool());
    } else if (name == "enable_downsampling") {
      applied = (cfg_snapshot.enable_downsampling == param.as_bool());
    } else if (name == "max_points") {
      applied = (cfg_snapshot.max_points == param.as_int());
    } else if (name == "enable_accumulation") {
      applied = (cfg_snapshot.enable_accumulation == param.as_bool());
    } else if (name == "accumulation_time_sec") {
      applied = equals_double(cfg_snapshot.accumulation_time_sec, param.as_double());
    } else if (name == "accumulation_frames") {
      applied = (cfg_snapshot.accumulation_frames == param.as_int());
    } else if (name == "accumulation_max_points") {
      applied = (cfg_snapshot.accumulation_max_points == param.as_int());
    } else if (name == "filter_by_distance") {
      applied = (cfg_snapshot.filter_by_distance == param.as_bool());
    } else if (name == "min_distance") {
      applied = equals_double(cfg_snapshot.min_distance, param.as_double());
    } else if (name == "max_distance") {
      applied = equals_double(cfg_snapshot.max_distance, param.as_double());
    } else if (name == "enable_metrics") {
      applied = (cfg_snapshot.enable_metrics == param.as_bool());
    } else if (name == "edge_threshold") {
      applied = (cfg_snapshot.edge_threshold == param.as_int());
    } else if (name == "edge_distance_threshold") {
      applied = equals_double(cfg_snapshot.edge_distance_threshold, param.as_double());
    } else if (name == "min_points_for_nmi") {
      applied = (cfg_snapshot.min_points_for_nmi == param.as_int());
    } else if (name == "reset_to_defaults") {
      applied = (reset_requested == param.as_bool());
    }

    if (applied) {
      RCLCPP_INFO(node_->get_logger(), "[CalibrationValidator] Parameter applied: %s = %s",
                  name.c_str(), param.value_to_string().c_str());
    } else {
      RCLCPP_WARN(node_->get_logger(), "[CalibrationValidator] Parameter apply check failed: %s = %s",
                  name.c_str(), param.value_to_string().c_str());
    }
  }

  return result;
}

void CalibrationValidator::setupApproximateSync() {
  render_timer_.reset();
  image_raw_sub_.reset();
  cloud_raw_sub_.reset();

  image_sub_ = std::make_shared<message_filters::Subscriber<ImageMsg>>();
  cloud_sub_ = std::make_shared<message_filters::Subscriber<CloudMsg>>();

#if __has_include(<message_filters/subscriber.h>)
  image_sub_->subscribe(node_.get(), image_topic_, rmw_qos_profile_sensor_data);
  cloud_sub_->subscribe(node_.get(), cloud_topic_, rmw_qos_profile_sensor_data);
#else
  image_sub_->subscribe(node_.get(), image_topic_, rclcpp::SensorDataQoS());
  cloud_sub_->subscribe(node_.get(), cloud_topic_, rclcpp::SensorDataQoS());
#endif

  int queue_size = node_->get_parameter("queue_size").as_int();
  sync_ = std::make_shared<Synchronizer>(SyncPolicy(queue_size), *image_sub_, *cloud_sub_);
  sync_->registerCallback(std::bind(&CalibrationValidator::syncCallback, this,
                                    std::placeholders::_1, std::placeholders::_2));
}

void CalibrationValidator::setupLatestSync() {
  sync_.reset();
  image_sub_.reset();
  cloud_sub_.reset();

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_image_msg_.reset();
    latest_image_bgr_.release();
    has_latest_image_ = false;
  }
  {
    std::lock_guard<std::mutex> lock(projection_cache_mutex_);
    has_projection_cache_ = false;
    cloud_frame_counter_ = 0;
  }

  auto qos = rclcpp::SensorDataQoS();
  image_raw_sub_ = node_->create_subscription<ImageMsg>(
      image_topic_, qos,
      std::bind(&CalibrationValidator::imageLatestCallback, this, std::placeholders::_1));
  cloud_raw_sub_ = node_->create_subscription<CloudMsg>(
      cloud_topic_, qos,
      std::bind(&CalibrationValidator::cloudLatestCallback, this, std::placeholders::_1));

  render_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(1000 / render_rate_hz_),
      std::bind(&CalibrationValidator::renderLatestFrame, this));
}

bool CalibrationValidator::loadCalibrationParameters() {
  std::string calib_file = calibration_file_;

  std::smatch match;
  std::regex find_pkg_regex(R"(\$\(find\s+([^)\s]+)\))");
  if (std::regex_search(calib_file, match, find_pkg_regex) && match.size() >= 2) {
    const std::string package_name = match[1];
    try {
      const std::string package_path = ament_index_cpp::get_package_share_directory(package_name);
      calib_file = std::regex_replace(calib_file, find_pkg_regex, package_path);
    } catch (const std::exception& e) {
      RCLCPP_WARN(node_->get_logger(),
                  "[CalibrationValidator] Failed to resolve $(find %s): %s",
                  package_name.c_str(), e.what());
    }
  }

  cv::FileStorage fs(calib_file, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    RCLCPP_ERROR(node_->get_logger(), "[CalibrationValidator] Cannot open calibration file: %s",
                 calib_file.c_str());
    calibration_loaded_ = false;
    return false;
  }

  fs["K_0"] >> K_matrix_;
  fs["C_0"] >> D_coeffs_;
  fs["E_0"] >> E_matrix_;
  fs.release();

  if (K_matrix_.rows != 3 || K_matrix_.cols != 3) {
    RCLCPP_ERROR(node_->get_logger(), "[CalibrationValidator] Invalid camera matrix size: %dx%d",
                 K_matrix_.rows, K_matrix_.cols);
    calibration_loaded_ = false;
    return false;
  }

  if (E_matrix_.rows != 4 || E_matrix_.cols != 4) {
    RCLCPP_ERROR(node_->get_logger(), "[CalibrationValidator] Invalid extrinsic matrix size: %dx%d",
                 E_matrix_.rows, E_matrix_.cols);
    calibration_loaded_ = false;
    return false;
  }

  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      lidar_to_camera_(i, j) = static_cast<float>(E_matrix_.at<double>(i, j));
    }
  }

  fx_ = K_matrix_.at<double>(0, 0);
  fy_ = K_matrix_.at<double>(1, 1);
  cx_ = K_matrix_.at<double>(0, 2);
  cy_ = K_matrix_.at<double>(1, 2);

  if (fx_ <= 0 || fy_ <= 0 || cx_ <= 0 || cy_ <= 0) {
    RCLCPP_ERROR(node_->get_logger(), "[CalibrationValidator] Invalid camera intrinsic parameters");
    calibration_loaded_ = false;
    return false;
  }

  if (!D_coeffs_.empty() && D_coeffs_.rows >= 1 && D_coeffs_.cols >= 4) {
    has_distortion_ = true;
    k1_ = D_coeffs_.at<double>(0, 0);
    k2_ = D_coeffs_.at<double>(0, 1);
    p1_ = D_coeffs_.at<double>(0, 2);
    p2_ = D_coeffs_.at<double>(0, 3);
    k3_ = (D_coeffs_.cols > 4) ? D_coeffs_.at<double>(0, 4) : 0.0;
  } else {
    has_distortion_ = false;
    k1_ = k2_ = k3_ = p1_ = p2_ = 0.0;
  }

  cv::Mat r = E_matrix_(cv::Rect(0, 0, 3, 3));
  double det = cv::determinant(r);
  if (std::abs(det - 1.0) > 0.1) {
    RCLCPP_WARN(node_->get_logger(),
                "[CalibrationValidator] Rotation matrix determinant = %.3f (should be ~1.0)", det);
  }

  calibration_loaded_ = true;
  RCLCPP_INFO(node_->get_logger(),
              "[CalibrationValidator] Calibration loaded from: %s", calib_file.c_str());
  return true;
}

rclcpp::QoS CalibrationValidator::buildFusedImageQos() const {
  std::string reliability = fused_image_qos_reliability_;
  std::transform(reliability.begin(), reliability.end(), reliability.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

  rclcpp::QoS qos(rclcpp::KeepLast(10));
  if (reliability == "reliable") {
    qos.reliable();
  } else {
    qos.best_effort();
  }
  qos.durability_volatile();
  return qos;
}

void CalibrationValidator::recreateFusedImagePublisher() {
  std::string reliability = fused_image_qos_reliability_;
  std::transform(reliability.begin(), reliability.end(), reliability.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (reliability != "best_effort" && reliability != "reliable") {
    RCLCPP_WARN(node_->get_logger(),
                "[CalibrationValidator] Invalid fused_image_qos_reliability='%s', fallback to 'best_effort'",
                fused_image_qos_reliability_.c_str());
    reliability = "best_effort";
  }
  fused_image_qos_reliability_ = reliability;

  fused_image_pub_ = node_->create_publisher<ImageMsg>(fused_topic_, buildFusedImageQos());
  RCLCPP_INFO(node_->get_logger(),
              "[CalibrationValidator] Fused image QoS reliability: %s",
              fused_image_qos_reliability_.c_str());
}

void CalibrationValidator::syncCallback(const ImageMsg::ConstSharedPtr& image_msg,
                                        const CloudMsg::ConstSharedPtr& cloud_msg) {
  imageHeartbeat();
  cloudHeartbeat();
  (void)processDataPair(image_msg, cloud_msg);
}

void CalibrationValidator::imageLatestCallback(const ImageMsg::ConstSharedPtr& image_msg) {
  imageHeartbeat();

  try {
    cv_bridge::CvImagePtr cv_image =
        cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = cv_image->image.clone();

    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_image_msg_ = image_msg;
    latest_image_header_ = image_msg->header;
    latest_image_arrival_ = node_->now();
    latest_image_bgr_ = image;
    has_latest_image_ = true;
  } catch (const std::exception& e) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "[CalibrationValidator] Failed to convert image in latest callback: %s", e.what());
  }
}

void CalibrationValidator::cloudLatestCallback(const CloudMsg::ConstSharedPtr& cloud_msg) {
  cloudHeartbeat();
  (void)updateProjectionCacheFromCloud(cloud_msg);
}

void CalibrationValidator::renderLatestFrame() {
  applyDeferredResetToDefaults();

  if (!use_latest_sync_) {
    return;
  }

  cv::Mat image;
  std_msgs::msg::Header image_header;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!has_latest_image_ || latest_image_bgr_.empty()) {
      return;
    }
    image = latest_image_bgr_.clone();
    image_header = latest_image_header_;
  }

  RuntimeConfig current_config;
  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    current_config = config_;
  }

  std::vector<cv::Point2f> points;
  std::vector<cv::Scalar> colors;
  ValidationMetrics metrics;
  bool has_cache = false;
  {
    std::lock_guard<std::mutex> lock(projection_cache_mutex_);
    has_cache = has_projection_cache_;
    if (has_cache) {
      points = last_projection_points_;
      colors = last_projection_colors_;
      metrics = last_projection_metrics_;
    }
  }

  if (has_cache) {
    for (size_t i = 0; i < points.size(); ++i) {
      cv::circle(image, points[i], current_config.point_size, colors[i], -1);
    }
  } else {
    cv::putText(image, "Waiting for point cloud...", cv::Point(20, 40),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
  }
  if (current_config.show_statistics) {
    drawStatistics(image, metrics);
  }
  if (current_config.show_edge_overlay) {
    drawEdgeOverlay(image);
  }
  if (current_config.show_depth_colorbar) {
    drawDepthColorbar(image, static_cast<float>(current_config.min_depth),
                      static_cast<float>(current_config.max_depth));
  }

  if (show_window_) {
    if (!window_initialized_) {
      cv::namedWindow("LiDAR-Camera Calibration Validator", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
      cv::resizeWindow("LiDAR-Camera Calibration Validator", 1200, 800);
      window_initialized_ = true;
    }
    cv::imshow("LiDAR-Camera Calibration Validator", image);
    cv::waitKey(1);
  }

  auto fused_msg = cv_bridge::CvImage(image_header, "bgr8", image).toImageMsg();
  fused_image_pub_->publish(*fused_msg);
  publishValidationInfo(metrics);
}

bool CalibrationValidator::updateProjectionCacheFromCloud(const CloudMsg::ConstSharedPtr& cloud_msg) {
  applyDeferredResetToDefaults();

  if (!use_latest_sync_) {
    return false;
  }
  if (!calibration_loaded_) {
    return false;
  }

  cv::Mat image_for_metrics;
  int width = 0;
  int height = 0;
  rclcpp::Time image_arrival = node_->now();
  builtin_interfaces::msg::Time image_header_stamp;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!has_latest_image_ || latest_image_bgr_.empty()) {
      return false;
    }
    width = latest_image_bgr_.cols;
    height = latest_image_bgr_.rows;
    image_arrival = latest_image_arrival_;
    image_header_stamp = latest_image_header_.stamp;
    image_for_metrics = latest_image_bgr_.clone();
  }

  rclcpp::Time cloud_arrival = node_->now();
  double arrival_gap = std::abs((image_arrival - cloud_arrival).seconds());

  double stamp_gap = -1.0;
  if (rclcpp::Time(image_header_stamp).nanoseconds() != 0 &&
      rclcpp::Time(cloud_msg->header.stamp).nanoseconds() != 0) {
    rclcpp::Time image_stamp(image_header_stamp);
    rclcpp::Time cloud_stamp(cloud_msg->header.stamp);
    stamp_gap = std::abs((image_stamp - cloud_stamp).seconds());
  }

  if (latest_pair_time_threshold_ > 0.0 &&
      arrival_gap > latest_pair_time_threshold_ &&
      !arrival_gap_warned_.exchange(true, std::memory_order_acq_rel)) {
    RCLCPP_WARN(node_->get_logger(),
                "[CalibrationValidator] image-cloud arrival gap %.3fs (stamp gap %.3fs, threshold %.3fs)",
                arrival_gap, stamp_gap, latest_pair_time_threshold_);
  }

  bool expected = false;
  if (!is_processing_.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
    return false;
  }

  auto start_time = std::chrono::high_resolution_clock::now();
  bool processed = false;
  try {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    if (!cloud->empty()) {
      RuntimeConfig current_config;
      {
        std::lock_guard<std::mutex> lock(config_mutex_);
        current_config = config_;
      }

      rclcpp::Time cloud_stamp = cloud_msg->header.stamp;
      if (cloud_stamp.nanoseconds() == 0) {
        cloud_stamp = node_->now();
      }

      cloud = accumulatePointCloud(cloud, cloud_stamp, current_config);
      if (current_config.enable_downsampling &&
          static_cast<int>(cloud->size()) > current_config.max_points) {
        cloud = downsamplePointCloud(cloud, current_config.max_points);
      }

      std::vector<cv::Point2f> points;
      std::vector<float> depths;
      std::vector<cv::Scalar> colors;
      projectCloudPoints(cloud, width, height,
                         static_cast<float>(current_config.min_distance),
                         static_cast<float>(current_config.max_distance),
                         static_cast<float>(current_config.min_depth),
                         static_cast<float>(current_config.max_depth),
                         points, depths, colors);

      ValidationMetrics metrics;
      metrics.valid_projections = static_cast<int>(points.size());
      metrics.total_points = static_cast<int>(cloud->size());
      metrics.projection_ratio =
          metrics.total_points > 0 ? static_cast<double>(metrics.valid_projections) / metrics.total_points : 0.0;
      if (!depths.empty()) {
        auto minmax = std::minmax_element(depths.begin(), depths.end());
        metrics.min_depth = *minmax.first;
        metrics.max_depth = *minmax.second;
        metrics.mean_depth = std::accumulate(depths.begin(), depths.end(), 0.0) / depths.size();
      }

      cloud_frame_counter_++;
      bool should_compute_metrics = current_config.enable_metrics &&
                                    (cloud_frame_counter_ % metrics_compute_interval_ == 0);
      if (should_compute_metrics && !points.empty()) {
        ValidationMetrics detailed_metrics =
            calculateValidationMetrics(image_for_metrics, points, depths);
        metrics.edge_overlap_score = detailed_metrics.edge_overlap_score;
        metrics.depth_gray_abs_pearson = detailed_metrics.depth_gray_abs_pearson;
      } else {
        std::lock_guard<std::mutex> lock(projection_cache_mutex_);
        metrics.edge_overlap_score = last_projection_metrics_.edge_overlap_score;
        metrics.depth_gray_abs_pearson = last_projection_metrics_.depth_gray_abs_pearson;
      }

      auto end_time = std::chrono::high_resolution_clock::now();
      metrics.processing_time_ms =
          std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000.0;

      {
        std::lock_guard<std::mutex> lock(projection_cache_mutex_);
        last_projection_points_ = std::move(points);
        last_projection_depths_ = std::move(depths);
        last_projection_colors_ = std::move(colors);
        last_projection_metrics_ = metrics;
        last_projection_stamp_ = node_->now();
        has_projection_cache_ = true;
      }
      updatePerformanceStats();
      processed = true;
    }
  } catch (const std::exception& e) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "[CalibrationValidator] updateProjectionCacheFromCloud failed: %s", e.what());
  }

  is_processing_.store(false, std::memory_order_release);
  return processed;
}

bool CalibrationValidator::processDataPair(const ImageMsg::ConstSharedPtr& image_msg,
                                           const CloudMsg::ConstSharedPtr& cloud_msg) {
  applyDeferredResetToDefaults();

  data_received_ = true;
  last_data_time_ = node_->now();

  if (!calibration_loaded_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                         "[CalibrationValidator] Calibration not loaded, skipping frame");
    return false;
  }

  bool expected = false;
  if (!is_processing_.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "[CalibrationValidator] Skipping frame - previous frame still processing");
    return false;
  }

  bool processed = false;
  auto start_time = std::chrono::high_resolution_clock::now();

  try {
    cv_bridge::CvImagePtr cv_image =
        cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = cv_image->image.clone();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    if (!cloud->empty()) {
      RuntimeConfig current_config;
      {
        std::lock_guard<std::mutex> lock(config_mutex_);
        current_config = config_;
      }

      rclcpp::Time cloud_stamp = cloud_msg->header.stamp;
      if (cloud_stamp.nanoseconds() == 0) {
        cloud_stamp = node_->now();
      }

      cloud = accumulatePointCloud(cloud, cloud_stamp, current_config);

      if (current_config.enable_downsampling &&
          static_cast<int>(cloud->size()) > current_config.max_points) {
        cloud = downsamplePointCloud(cloud, current_config.max_points);
      }

      ValidationMetrics metrics;
      cv::Mat fused_image = projectPointsToImageFast(image, cloud, metrics);

      auto end_time = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
      metrics.processing_time_ms = duration.count() / 1000.0;

      if (!fused_image.empty()) {
        if (show_window_) {
          if (!window_initialized_) {
            cv::namedWindow("LiDAR-Camera Calibration Validator", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
            cv::resizeWindow("LiDAR-Camera Calibration Validator", 1200, 800);
            window_initialized_ = true;
          }
          cv::imshow("LiDAR-Camera Calibration Validator", fused_image);
          cv::waitKey(1);
        }

        auto fused_msg = cv_bridge::CvImage(image_msg->header, "bgr8", fused_image).toImageMsg();
        fused_image_pub_->publish(*fused_msg);

        publishValidationInfo(metrics);
      }

      updatePerformanceStats();
      processed = true;
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "[CalibrationValidator] Error in processing callback: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(node_->get_logger(), "[CalibrationValidator] Unknown error in processing callback");
  }

  is_processing_.store(false, std::memory_order_release);
  return processed;
}

cv::Scalar CalibrationValidator::depthToColor(float depth, float min_depth, float max_depth) {
  static const std::vector<cv::Scalar> kDepthColorLut = []() {
    std::vector<cv::Scalar> lut(256);
    for (int i = 0; i < 256; ++i) {
      float normalized_depth = static_cast<float>(i) / 255.0f;
      float hue = 240.0f * normalized_depth;
      cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(hue, 255, 255));
      cv::Mat bgr;
      cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
      cv::Vec3b color = bgr.at<cv::Vec3b>(0, 0);
      lut[i] = cv::Scalar(color[0], color[1], color[2]);
    }
    return lut;
  }();

  if (max_depth <= min_depth) {
    return kDepthColorLut.front();
  }

  depth = std::max(min_depth, std::min(depth, max_depth));
  float normalized_depth = (max_depth - depth) / (max_depth - min_depth);
  int idx = static_cast<int>(std::round(normalized_depth * 255.0f));
  idx = std::max(0, std::min(255, idx));
  return kDepthColorLut[static_cast<size_t>(idx)];
}

cv::Point2f CalibrationValidator::applyDistortionCorrection(float u_norm, float v_norm) {
  if (!has_distortion_) {
    return cv::Point2f(u_norm * fx_ + cx_, v_norm * fy_ + cy_);
  }

  double r2 = u_norm * u_norm + v_norm * v_norm;
  double r4 = r2 * r2;
  double r6 = r4 * r2;
  double radial_distortion = 1.0 + k1_ * r2 + k2_ * r4 + k3_ * r6;

  double tangential_u = 2.0 * p1_ * u_norm * v_norm + p2_ * (r2 + 2.0 * u_norm * u_norm);
  double tangential_v = p1_ * (r2 + 2.0 * v_norm * v_norm) + 2.0 * p2_ * u_norm * v_norm;

  double u_distorted = u_norm * radial_distortion + tangential_u;
  double v_distorted = v_norm * radial_distortion + tangential_v;

  return cv::Point2f(u_distorted * fx_ + cx_, v_distorted * fy_ + cy_);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CalibrationValidator::downsamplePointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int max_points) {
  if (static_cast<int>(cloud->size()) <= max_points) {
    return cloud;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::RandomSample<pcl::PointXYZ> random_sample;
  random_sample.setInputCloud(cloud);
  random_sample.setSample(max_points);
  random_sample.filter(*downsampled);
  return downsampled;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CalibrationValidator::accumulatePointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const rclcpp::Time& stamp,
    const RuntimeConfig& config) {
  if (!config.enable_accumulation) {
    std::lock_guard<std::mutex> lock(accumulation_mutex_);
    cloud_buffer_.clear();
    return cloud;
  }

  {
    std::lock_guard<std::mutex> lock(accumulation_mutex_);
    cloud_buffer_.push_back({cloud, stamp});

    double time_window = std::max(0.0, config.accumulation_time_sec);
    while (!cloud_buffer_.empty()) {
      bool removed = false;

      if (time_window > 0.0) {
        double age = (cloud_buffer_.back().stamp - cloud_buffer_.front().stamp).seconds();
        if (age > time_window) {
          cloud_buffer_.pop_front();
          removed = true;
        }
      }

      if (config.accumulation_frames > 0 &&
          static_cast<int>(cloud_buffer_.size()) > config.accumulation_frames) {
        cloud_buffer_.pop_front();
        removed = true;
      }

      if (!removed) {
        break;
      }
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr merged(new pcl::PointCloud<pcl::PointXYZ>);
  {
    std::lock_guard<std::mutex> lock(accumulation_mutex_);
    for (const auto& entry : cloud_buffer_) {
      *merged += *entry.cloud;
    }
  }

  if (config.accumulation_max_points > 0 &&
      static_cast<int>(merged->size()) > config.accumulation_max_points) {
    merged = downsamplePointCloud(merged, config.accumulation_max_points);
  }

  return merged;
}

ValidationMetrics CalibrationValidator::calculateValidationMetrics(
    const cv::Mat& image,
    const std::vector<cv::Point2f>& projected_points,
    const std::vector<float>& depths) {
  ValidationMetrics metrics;
  if (projected_points.empty()) {
    return metrics;
  }

  RuntimeConfig current_config;
  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    current_config = config_;
  }

  if (projected_points.size() >= 10) {
    metrics.edge_overlap_score = calculateEdgeOverlapScore(image, projected_points);
  }

  if (static_cast<int>(projected_points.size()) >= current_config.min_points_for_nmi) {
    metrics.depth_gray_abs_pearson =
        calculateDepthGrayAbsPearson(image, projected_points, depths);
  }

  return metrics;
}

double CalibrationValidator::calculateEdgeOverlapScore(
    const cv::Mat& image, const std::vector<cv::Point2f>& projected_points) {
  RuntimeConfig current_config;
  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    current_config = config_;
  }

  cv::Mat gray;
  cv::Mat edges;
  cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  cv::Canny(gray, edges, current_config.edge_threshold, current_config.edge_threshold * 2);

  cv::Mat dist_transform;
  cv::distanceTransform(~edges, dist_transform, cv::DIST_L2, 3);

  int close_to_edge = 0;
  for (const auto& point : projected_points) {
    int x = static_cast<int>(point.x);
    int y = static_cast<int>(point.y);

    if (x >= 0 && x < dist_transform.cols && y >= 0 && y < dist_transform.rows) {
      double distance = dist_transform.at<float>(y, x);
      if (distance <= current_config.edge_distance_threshold) {
        close_to_edge++;
      }
    }
  }

  return projected_points.empty() ? 0.0
                                  : static_cast<double>(close_to_edge) / projected_points.size();
}

double CalibrationValidator::calculateDepthGrayAbsPearson(
    const cv::Mat& image,
    const std::vector<cv::Point2f>& projected_points,
    const std::vector<float>& depths) {
  if (projected_points.size() < 50 || depths.size() != projected_points.size()) {
    return 0.0;
  }

  cv::Mat gray;
  cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

  std::vector<double> intensities;
  std::vector<double> depth_values;

  for (size_t i = 0; i < projected_points.size(); ++i) {
    int x = static_cast<int>(projected_points[i].x);
    int y = static_cast<int>(projected_points[i].y);
    if (x >= 0 && x < gray.cols && y >= 0 && y < gray.rows) {
      intensities.push_back(gray.at<uchar>(y, x));
      depth_values.push_back(depths[i]);
    }
  }

  if (intensities.size() < 50) {
    return 0.0;
  }

  double mean_intensity = 0.0;
  double mean_depth = 0.0;
  for (size_t i = 0; i < intensities.size(); ++i) {
    mean_intensity += intensities[i];
    mean_depth += depth_values[i];
  }
  mean_intensity /= intensities.size();
  mean_depth /= depth_values.size();

  double numerator = 0.0;
  double denom_intensity = 0.0;
  double denom_depth = 0.0;
  for (size_t i = 0; i < intensities.size(); ++i) {
    double diff_intensity = intensities[i] - mean_intensity;
    double diff_depth = depth_values[i] - mean_depth;
    numerator += diff_intensity * diff_depth;
    denom_intensity += diff_intensity * diff_intensity;
    denom_depth += diff_depth * diff_depth;
  }

  if (denom_intensity == 0.0 || denom_depth == 0.0) {
    return 0.0;
  }

  double correlation = numerator / std::sqrt(denom_intensity * denom_depth);
  return std::abs(correlation);
}

void CalibrationValidator::drawStatistics(cv::Mat& image, const ValidationMetrics& metrics) {
  int y_offset = 30;
  cv::Scalar text_color(0, 255, 0);
  cv::Scalar bg_color(0, 0, 0);
  double font_scale = 0.7;
  int thickness = 2;
  int line_height = 25;

  cv::Rect bg_rect(5, 5, 450, 200);
  cv::Mat overlay;
  image.copyTo(overlay);
  cv::rectangle(overlay, bg_rect, bg_color, -1);
  cv::addWeighted(image, 0.7, overlay, 0.3, 0, image);

  cv::putText(image,
              cv::format("Projection: %d/%d points (%.1f%%)", metrics.valid_projections,
                         metrics.total_points, metrics.projection_ratio * 100.0),
              cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, font_scale, text_color, thickness);
  y_offset += line_height;

  cv::putText(image, cv::format("Edge Overlap Score: %.3f", metrics.edge_overlap_score),
              cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, font_scale, text_color, thickness);
  y_offset += line_height;

  cv::putText(image, cv::format("|corr(gray,depth)|: %.3f", metrics.depth_gray_abs_pearson),
              cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, font_scale, text_color, thickness);
  y_offset += line_height;

  if (metrics.total_points > 0) {
    cv::putText(image, cv::format("Depth Range: %.1f - %.1f m", metrics.min_depth, metrics.max_depth),
                cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, font_scale, text_color, thickness);
    y_offset += line_height;

    cv::putText(image, cv::format("Mean Depth: %.1f m", metrics.mean_depth),
                cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, font_scale, text_color, thickness);
    y_offset += line_height;
  }

  cv::putText(image, cv::format("Processing: %.1f ms (%.1f FPS)", metrics.processing_time_ms, processing_fps_),
              cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, font_scale, text_color, thickness);
  y_offset += line_height;

  std::string quality_text;
  cv::Scalar quality_color = text_color;

  quality_text = "Auxiliary metrics only; see GT pixel error separately";
  cv::putText(image, quality_text, cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX,
              font_scale, quality_color, thickness);
}

void CalibrationValidator::drawEdgeOverlay(cv::Mat& image) {
  RuntimeConfig current_config;
  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    current_config = config_;
  }

  cv::Mat gray;
  cv::Mat edges;
  cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  cv::Canny(gray, edges, current_config.edge_threshold, current_config.edge_threshold * 2);

  std::vector<cv::Mat> channels;
  cv::split(image, channels);
  cv::bitwise_or(channels[1], edges, channels[1]);
  cv::merge(channels, image);
}

void CalibrationValidator::drawDepthColorbar(cv::Mat& image, float min_depth, float max_depth) {
  int legend_width = 250;
  int legend_height = 25;
  int legend_x = image.cols - legend_width - 25;
  int legend_y = 25;

  cv::rectangle(image, cv::Point(legend_x - 8, legend_y - 8),
                cv::Point(legend_x + legend_width + 8, legend_y + legend_height + 30),
                cv::Scalar(0, 0, 0), -1);

  for (int i = 0; i < legend_width; ++i) {
    float depth_ratio = static_cast<float>(i) / legend_width;
    float depth = max_depth - depth_ratio * (max_depth - min_depth);
    cv::Scalar color = depthToColor(depth, min_depth, max_depth);
    cv::line(image, cv::Point(legend_x + i, legend_y),
             cv::Point(legend_x + i, legend_y + legend_height), color, 1);
  }

  cv::putText(image, cv::format("%.1fm", min_depth),
              cv::Point(legend_x + legend_width - 35, legend_y + legend_height + 20),
              cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 255, 255), 1);

  cv::putText(image, cv::format("%.1fm", max_depth),
              cv::Point(legend_x, legend_y + legend_height + 20),
              cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 255, 255), 1);

  cv::putText(image, "Depth", cv::Point(legend_x + legend_width / 2 - 25, legend_y - 8),
              cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(255, 255, 255), 1);
}

void CalibrationValidator::imageHeartbeat() {
  data_received_ = true;
  last_data_time_ = node_->now();
}

void CalibrationValidator::cloudHeartbeat() {
  data_received_ = true;
  last_data_time_ = node_->now();
}

void CalibrationValidator::publishValidationInfo(const ValidationMetrics& metrics) {
  std::ostringstream json_stream;
  json_stream << "{"
              << "\"timestamp\":" << node_->now().seconds() << ","
              << "\"projection_ratio\":" << metrics.projection_ratio << ","
              << "\"edge_overlap_score\":" << metrics.edge_overlap_score << ","
              << "\"normalized_mutual_info\":" << metrics.depth_gray_abs_pearson << ","
              << "\"depth_gray_abs_pearson\":" << metrics.depth_gray_abs_pearson << ","
              << "\"valid_projections\":" << metrics.valid_projections << ","
              << "\"total_points\":" << metrics.total_points << ","
              << "\"processing_time_ms\":" << metrics.processing_time_ms << ","
              << "\"min_depth\":" << metrics.min_depth << ","
              << "\"max_depth\":" << metrics.max_depth << ","
              << "\"mean_depth\":" << metrics.mean_depth << ","
              << "\"processing_fps\":" << processing_fps_ << "}";

  std_msgs::msg::String info_msg;
  info_msg.data = json_stream.str();
  validation_info_pub_->publish(info_msg);
}

void CalibrationValidator::updatePerformanceStats() {
  frame_count_++;

  auto current_time = std::chrono::steady_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_process_time_);

  if (duration.count() >= 1000) {
    processing_fps_ = static_cast<double>(frame_count_) / (duration.count() / 1000.0);
    frame_count_ = 0;
    last_process_time_ = current_time;
  }
}

void CalibrationValidator::applyDeferredResetToDefaults() {
  if (!reset_defaults_requested_.exchange(false, std::memory_order_acq_rel)) {
    return;
  }

  std::vector<rclcpp::Parameter> defaults = {
      rclcpp::Parameter("point_size", 3),
      rclcpp::Parameter("min_depth", 0.5),
      rclcpp::Parameter("max_depth", 50.0),
      rclcpp::Parameter("show_statistics", true),
      rclcpp::Parameter("show_edge_overlay", false),
      rclcpp::Parameter("show_depth_colorbar", true),
      rclcpp::Parameter("enable_downsampling", false),
      rclcpp::Parameter("max_points", 1000000),
      rclcpp::Parameter("enable_accumulation", false),
      rclcpp::Parameter("accumulation_time_sec", 0.1),
      rclcpp::Parameter("accumulation_frames", 3),
      rclcpp::Parameter("accumulation_max_points", 200000),
      rclcpp::Parameter("filter_by_distance", true),
      rclcpp::Parameter("min_distance", 0.3),
      rclcpp::Parameter("max_distance", 100.0),
      rclcpp::Parameter("enable_metrics", true),
      rclcpp::Parameter("edge_threshold", 50),
      rclcpp::Parameter("edge_distance_threshold", 5.0),
      rclcpp::Parameter("min_points_for_nmi", 100),
      rclcpp::Parameter("render_rate_hz", 30),
      rclcpp::Parameter("projection_cache_ttl_sec", 0.1),
      rclcpp::Parameter("metrics_compute_interval", 3),
      rclcpp::Parameter("reset_to_defaults", false)};

  try {
    node_->set_parameters(defaults);
  } catch (const std::exception& e) {
    RCLCPP_WARN(node_->get_logger(),
                "[CalibrationValidator] Failed to apply reset_to_defaults: %s", e.what());
  }
}

cv::Mat CalibrationValidator::projectPointsToImageFast(
    const cv::Mat& image,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    ValidationMetrics& metrics) {
  if (!cloud || cloud->empty()) {
    return image.clone();
  }

  cv::Mat result = image.clone();
  int width = image.cols;
  int height = image.rows;

  RuntimeConfig current_config;
  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    current_config = config_;
  }

  projected_points_cache_.clear();
  depths_cache_.clear();
  colors_cache_.clear();

  projectCloudPoints(cloud, width, height,
                     static_cast<float>(current_config.min_distance),
                     static_cast<float>(current_config.max_distance),
                     static_cast<float>(current_config.min_depth),
                     static_cast<float>(current_config.max_depth),
                     projected_points_cache_, depths_cache_, colors_cache_);

  for (size_t i = 0; i < projected_points_cache_.size(); ++i) {
    cv::circle(result, projected_points_cache_[i], current_config.point_size, colors_cache_[i], -1);
  }

  metrics.valid_projections = static_cast<int>(projected_points_cache_.size());
  metrics.total_points = static_cast<int>(cloud->size());
  metrics.projection_ratio =
      metrics.total_points > 0 ? static_cast<double>(metrics.valid_projections) / metrics.total_points : 0.0;

  if (!depths_cache_.empty()) {
    auto minmax = std::minmax_element(depths_cache_.begin(), depths_cache_.end());
    metrics.min_depth = *minmax.first;
    metrics.max_depth = *minmax.second;

    double sum = std::accumulate(depths_cache_.begin(), depths_cache_.end(), 0.0);
    metrics.mean_depth = sum / depths_cache_.size();
  }

  if (current_config.enable_metrics && !projected_points_cache_.empty()) {
    ValidationMetrics detailed_metrics =
        calculateValidationMetrics(image, projected_points_cache_, depths_cache_);
    metrics.edge_overlap_score = detailed_metrics.edge_overlap_score;
    metrics.depth_gray_abs_pearson = detailed_metrics.depth_gray_abs_pearson;
    metrics.mean_edge_distance = detailed_metrics.mean_edge_distance;
  }

  if (current_config.show_statistics) {
    drawStatistics(result, metrics);
  }

  if (current_config.show_edge_overlay) {
    drawEdgeOverlay(result);
  }

  if (current_config.show_depth_colorbar) {
    drawDepthColorbar(result, static_cast<float>(current_config.min_depth),
                      static_cast<float>(current_config.max_depth));
  }

  return result;
}

void CalibrationValidator::projectCloudPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                              int width,
                                              int height,
                                              float min_distance,
                                              float max_distance,
                                              float min_depth,
                                              float max_depth,
                                              std::vector<cv::Point2f>& points,
                                              std::vector<float>& depths,
                                              std::vector<cv::Scalar>& colors) {
  if (!cloud || cloud->empty()) {
    return;
  }

  const size_t cloud_size = cloud->size();
  points.reserve(cloud_size);
  depths.reserve(cloud_size);
  colors.reserve(cloud_size);

  if (!use_parallel_processing_ || static_cast<int>(cloud_size) <= PARALLEL_PROCESSING_THRESHOLD) {
    processPointBatch(cloud, 0, static_cast<int>(cloud_size), width, height, min_distance, max_distance,
                      points, depths, colors);
    return;
  }

  struct LocalBuffers {
    std::vector<cv::Point2f> points;
    std::vector<float> depths;
    std::vector<cv::Scalar> colors;
  };

  tbb::enumerable_thread_specific<LocalBuffers> tls;
  tbb::parallel_for(tbb::blocked_range<size_t>(0, cloud_size, 4096),
                    [&](const tbb::blocked_range<size_t>& range) {
                      auto& local = tls.local();
                      if (local.points.empty()) {
                        local.points.reserve(range.size());
                        local.depths.reserve(range.size());
                        local.colors.reserve(range.size());
                      }

                      for (size_t i = range.begin(); i != range.end(); ++i) {
                        const auto& point = cloud->points[i];
                        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
                          continue;
                        }

                        float distance =
                            std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
                        if (distance < min_distance || distance > max_distance) {
                          continue;
                        }

                        Eigen::Vector4f lidar_point(point.x, point.y, point.z, 1.0f);
                        Eigen::Vector4f camera_point = lidar_to_camera_ * lidar_point;
                        if (camera_point.z() <= 0.01f) {
                          continue;
                        }

                        float u_norm = camera_point.x() / camera_point.z();
                        float v_norm = camera_point.y() / camera_point.z();
                        cv::Point2f corrected_point = applyDistortionCorrection(u_norm, v_norm);
                        int u_int = static_cast<int>(std::round(corrected_point.x));
                        int v_int = static_cast<int>(std::round(corrected_point.y));

                        if (u_int >= 0 && u_int < width && v_int >= 0 && v_int < height) {
                          local.points.emplace_back(corrected_point.x, corrected_point.y);
                          local.depths.push_back(camera_point.z());
                          local.colors.push_back(depthToColor(camera_point.z(), min_depth, max_depth));
                        }
                      }
                    });

  size_t total_points = 0;
  for (const auto& local : tls) {
    total_points += local.points.size();
  }
  points.reserve(total_points);
  depths.reserve(total_points);
  colors.reserve(total_points);

  for (const auto& local : tls) {
    points.insert(points.end(), local.points.begin(), local.points.end());
    depths.insert(depths.end(), local.depths.begin(), local.depths.end());
    colors.insert(colors.end(), local.colors.begin(), local.colors.end());
  }
}

void CalibrationValidator::processPointBatch(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                             int start_idx,
                                             int end_idx,
                                             int width,
                                             int height,
                                             float min_distance,
                                             float max_distance,
                                             std::vector<cv::Point2f>& points,
                                             std::vector<float>& depths,
                                             std::vector<cv::Scalar>& colors) {
  int batch_size = end_idx - start_idx;
  points.reserve(batch_size);
  depths.reserve(batch_size);
  colors.reserve(batch_size);

  float min_depth;
  float max_depth;
  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    min_depth = static_cast<float>(config_.min_depth);
    max_depth = static_cast<float>(config_.max_depth);
  }

  for (int i = start_idx; i < end_idx; ++i) {
    const auto& point = cloud->points[static_cast<size_t>(i)];

    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }

    float distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    if (distance < min_distance || distance > max_distance) {
      continue;
    }

    Eigen::Vector4f lidar_point(point.x, point.y, point.z, 1.0f);
    Eigen::Vector4f camera_point = lidar_to_camera_ * lidar_point;

    if (camera_point.z() <= 0.01f) {
      continue;
    }

    float u_norm = camera_point.x() / camera_point.z();
    float v_norm = camera_point.y() / camera_point.z();

    cv::Point2f corrected_point = applyDistortionCorrection(u_norm, v_norm);
    int u_int = static_cast<int>(std::round(corrected_point.x));
    int v_int = static_cast<int>(std::round(corrected_point.y));

    if (u_int >= 0 && u_int < width && v_int >= 0 && v_int < height) {
      points.emplace_back(corrected_point.x, corrected_point.y);
      depths.push_back(camera_point.z());
      colors.push_back(depthToColor(camera_point.z(), min_depth, max_depth));
    }
  }
}

}  // namespace lidar_cam_validator
