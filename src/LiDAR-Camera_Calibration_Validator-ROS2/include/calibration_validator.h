// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025 Yao Yuzhuo (yaoyuzhuo6@gmail.com)

#ifndef CALIBRATION_VALIDATOR_H
#define CALIBRATION_VALIDATOR_H

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#if __has_include(<message_filters/subscriber.h>)
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#else
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/synchronizer.hpp>
#endif

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/random_sample.h>

#include <Eigen/Dense>

#include <memory>
#include <mutex>
#include <thread>
#include <chrono>
#include <algorithm>
#include <deque>
#include <atomic>
#include <sstream>
#include <vector>
#include <string>

#define PARALLEL_PROCESSING_THRESHOLD 30000

namespace lidar_cam_validator {

struct ValidationMetrics {
  double edge_overlap_score = 0.0;
  double depth_gray_abs_pearson = 0.0;
  double projection_ratio = 0.0;
  double mean_edge_distance = 0.0;

  int valid_projections = 0;
  int total_points = 0;

  double processing_time_ms = 0.0;
  double min_depth = 0.0;
  double max_depth = 0.0;
  double mean_depth = 0.0;
};

struct RuntimeConfig {
  int point_size = 3;
  double min_depth = 0.5;
  double max_depth = 50.0;
  bool show_statistics = true;
  bool show_edge_overlay = false;
  bool show_depth_colorbar = true;

  bool enable_downsampling = false;
  int max_points = 1000000;

  bool enable_accumulation = false;
  double accumulation_time_sec = 0.1;
  int accumulation_frames = 3;
  int accumulation_max_points = 200000;

  bool filter_by_distance = true;
  double min_distance = 0.3;
  double max_distance = 100.0;

  bool enable_metrics = true;
  int edge_threshold = 50;
  double edge_distance_threshold = 5.0;
  int min_points_for_nmi = 100;

  bool reset_to_defaults = false;
};

class CalibrationValidator {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit CalibrationValidator(const rclcpp::Node::SharedPtr& node);
  ~CalibrationValidator();

  bool hasDataReceived() const { return data_received_; }
  rclcpp::Time getLastDataTime() const { return last_data_time_; }

private:
  using ImageMsg = sensor_msgs::msg::Image;
  using CloudMsg = sensor_msgs::msg::PointCloud2;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<ImageMsg, CloudMsg>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  rclcpp::Node::SharedPtr node_;

  std::shared_ptr<message_filters::Subscriber<ImageMsg>> image_sub_;
  std::shared_ptr<message_filters::Subscriber<CloudMsg>> cloud_sub_;
  std::shared_ptr<Synchronizer> sync_;

  rclcpp::Subscription<ImageMsg>::SharedPtr image_raw_sub_;
  rclcpp::Subscription<CloudMsg>::SharedPtr cloud_raw_sub_;

  rclcpp::Publisher<ImageMsg>::SharedPtr fused_image_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr validation_info_pub_;

  RuntimeConfig config_;

  cv::Mat K_matrix_;
  cv::Mat D_coeffs_;
  cv::Mat E_matrix_;
  Eigen::Matrix4f lidar_to_camera_;
  bool calibration_loaded_ = false;

  double fx_ = 0.0;
  double fy_ = 0.0;
  double cx_ = 0.0;
  double cy_ = 0.0;
  double k1_ = 0.0;
  double k2_ = 0.0;
  double k3_ = 0.0;
  double p1_ = 0.0;
  double p2_ = 0.0;
  bool has_distortion_ = false;

  std::vector<cv::Point2f> projected_points_cache_;
  std::vector<float> depths_cache_;
  std::vector<cv::Scalar> colors_cache_;

  bool use_parallel_processing_ = true;
  std::chrono::steady_clock::time_point last_process_time_;
  double processing_fps_ = 0.0;
  int frame_count_ = 0;

  std::string image_topic_;
  std::string cloud_topic_;
  std::string fused_topic_;
  std::string fused_image_qos_reliability_;
  std::string info_topic_;
  std::string calibration_file_;

  rclcpp::TimerBase::SharedPtr render_timer_;
  bool window_initialized_ = false;
  bool show_window_ = true;
  bool data_received_ = false;
  rclcpp::Time last_data_time_;

  std::mutex config_mutex_;
  std::mutex data_mutex_;
  std::mutex accumulation_mutex_;
  std::mutex projection_cache_mutex_;

  ImageMsg::ConstSharedPtr latest_image_msg_;
  rclcpp::Time latest_image_arrival_;
  std_msgs::msg::Header latest_image_header_;
  cv::Mat latest_image_bgr_;
  bool has_latest_image_ = false;

  std::vector<cv::Point2f> last_projection_points_;
  std::vector<float> last_projection_depths_;
  std::vector<cv::Scalar> last_projection_colors_;
  ValidationMetrics last_projection_metrics_;
  rclcpp::Time last_projection_stamp_;
  bool has_projection_cache_ = false;
  int cloud_frame_counter_ = 0;
  int render_rate_hz_ = 30;
  double projection_cache_ttl_sec_ = 0.1;
  int metrics_compute_interval_ = 3;

  bool use_latest_sync_ = false;
  std::string sync_mode_;
  double latest_pair_time_threshold_ = 0.5;

  struct BufferedCloud {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    rclcpp::Time stamp;
  };
  std::deque<BufferedCloud> cloud_buffer_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
  std::atomic<bool> is_processing_{false};
  std::atomic<bool> reset_defaults_requested_{false};
  std::atomic<bool> arrival_gap_warned_{false};

  void declareAndLoadParameters();
  void updateRuntimeConfigFromNode();
  rcl_interfaces::msg::SetParametersResult onParameterUpdate(
      const std::vector<rclcpp::Parameter>& parameters);

  void setupApproximateSync();
  void setupLatestSync();
  void renderLatestFrame();
  bool processDataPair(const ImageMsg::ConstSharedPtr& image_msg,
                       const CloudMsg::ConstSharedPtr& cloud_msg);
  bool updateProjectionCacheFromCloud(const CloudMsg::ConstSharedPtr& cloud_msg);

  void syncCallback(const ImageMsg::ConstSharedPtr& image_msg,
                    const CloudMsg::ConstSharedPtr& cloud_msg);
  void imageLatestCallback(const ImageMsg::ConstSharedPtr& image_msg);
  void cloudLatestCallback(const CloudMsg::ConstSharedPtr& cloud_msg);

  cv::Mat projectPointsToImageFast(const cv::Mat& image,
                                   const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                   ValidationMetrics& metrics);
  void projectCloudPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                          int width,
                          int height,
                          float min_distance,
                          float max_distance,
                          float min_depth,
                          float max_depth,
                          std::vector<cv::Point2f>& points,
                          std::vector<float>& depths,
                          std::vector<cv::Scalar>& colors);
  void processPointBatch(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                         int start_idx,
                         int end_idx,
                         int width,
                         int height,
                         float min_distance,
                         float max_distance,
                         std::vector<cv::Point2f>& points,
                         std::vector<float>& depths,
                         std::vector<cv::Scalar>& colors);

  cv::Scalar depthToColor(float depth, float min_depth, float max_depth);
  cv::Point2f applyDistortionCorrection(float u_norm, float v_norm);

  pcl::PointCloud<pcl::PointXYZ>::Ptr downsamplePointCloud(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int max_points);
  pcl::PointCloud<pcl::PointXYZ>::Ptr accumulatePointCloud(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
      const rclcpp::Time& stamp,
      const RuntimeConfig& config);

  ValidationMetrics calculateValidationMetrics(const cv::Mat& image,
                                               const std::vector<cv::Point2f>& projected_points,
                                               const std::vector<float>& depths);
  double calculateEdgeOverlapScore(const cv::Mat& image,
                                   const std::vector<cv::Point2f>& projected_points);
  double calculateDepthGrayAbsPearson(const cv::Mat& image,
                                       const std::vector<cv::Point2f>& projected_points,
                                       const std::vector<float>& depths);

  void drawStatistics(cv::Mat& image, const ValidationMetrics& metrics);
  void drawEdgeOverlay(cv::Mat& image);
  void drawDepthColorbar(cv::Mat& image, float min_depth, float max_depth);

  bool loadCalibrationParameters();
  rclcpp::QoS buildFusedImageQos() const;
  void recreateFusedImagePublisher();

  void imageHeartbeat();
  void cloudHeartbeat();

  void publishValidationInfo(const ValidationMetrics& metrics);
  void updatePerformanceStats();
  void applyDeferredResetToDefaults();
};

}  // namespace lidar_cam_validator

#endif  // CALIBRATION_VALIDATOR_H
