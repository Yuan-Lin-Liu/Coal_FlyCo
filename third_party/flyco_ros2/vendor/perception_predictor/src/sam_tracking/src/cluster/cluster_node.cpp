#include <rclcpp/rclcpp.hpp>
#include <flyco_ros2/logging.hpp>
#include <rclcpp/create_timer.hpp>
#include <flyco_ros2/parameters.hpp>
#include <message_filters/subscriber.h>
#include <image_transport/image_transport.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Authors      :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 *                   Guiyong Zheng <shverses at gmail dot com>, SAI, SYSU.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 *                   https://gy920.github.io/
 * Date         :    Jun. 2026
 * E-mail       :    cfengag at connect dot ust dot hk
 *                   shverses at gmail dot com
 * Description  :    This file is part of the FlyCo Perception public ROS runtime.
 * Copyright    :    Copyright (c) 2026 Chen Feng and Guiyong Zheng.
 * License      :    PolyForm Noncommercial License 1.0.0
 *                   <https://polyformproject.org/licenses/noncommercial/1.0.0/>
 * Project      :    FlyCo: Foundation Model-Empowered Drones for Autonomous 3D Structure Scanning in Open-World Environments
 * Website      :    https://hkust-aerial-robotics.github.io/FC-Planner/
 *⭐⭐⭐******************************************************************⭐⭐⭐*/

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>

#include <algorithm>
#include <boost/filesystem.hpp>
#include <cctype>
#include <sstream>
#include <string>

#include "sam_tracking/cluster/pc_cluster.hpp"
#include "sam_tracking/cluster/pc_utils.hpp"

class PointCloudClusterManager {
 public:
  explicit PointCloudClusterManager(const rclcpp::Node::SharedPtr& nh) : node_(nh) {
    loadParameters(nh);
    setupSubscribers(nh);  // 包含同步器的初始化逻辑
    setupPublishers(nh);
    initializeCluster();
    checkAndLoadPointClouds(nh);
    process_timer_ =
        rclcpp::create_timer(nh, nh->get_clock(), rclcpp::Duration::from_seconds(1.0 / process_rate_hz_), std::bind(&PointCloudClusterManager::processLatestSyncedDataTimer, this));
    color_timer_ =
        rclcpp::create_timer(nh, nh->get_clock(), rclcpp::Duration::from_seconds(5.0), std::bind(&PointCloudClusterManager::colorPCTimerCallback, this));
  }

  ~PointCloudClusterManager() { shutdown(); }

  void shutdown() {
    process_timer_->cancel();
    color_timer_->cancel();

    if (sync_cloud_pose_) {
      sync_cloud_pose_.reset();
    }
    if (cloud_sub_) {
      cloud_sub_->unsubscribe();
      cloud_sub_.reset();
    }
    if (pose_sub_) {
      pose_sub_->unsubscribe();
      pose_sub_.reset();
    }

    target_point_sub_.reset();
    stop_save_sub_.reset();

    all_pc_pub_.reset();
    target_pc_pub_.reset();
    clustered_target_pc_pub_.reset();
    accumulated_target_pc_pub_.reset();
    rgb_pc_pub_.reset();
    env_pc_pub_.reset();
    pose_pub_.reset();
  }

 private:
  // 初始化参数
  void loadParameters(const rclcpp::Node::SharedPtr&nh) {
    // 加载已有参数
    all_point_topic_ = flyco::parameter_or<std::string>(nh, "all_point_topic", "/cluster/pointcloud");
    target_point_topic_ = flyco::parameter_or<std::string>(nh, "target_point_topic", "/cluster/guide_pc");
    min_height_ = flyco::parameter_or<double>(nh, "min_height", 0.0);
    cluster_tolerance_ = flyco::parameter_or<double>(nh, "cluster_tolerance", 1.0);
    resolution_ = flyco::parameter_or<double>(nh, "resolution", 0.1);
    min_cluster_size_ = flyco::parameter_or<int>(nh, "min_cluster_size", 50);
    max_cluster_size_ = flyco::parameter_or<int>(nh, "max_cluster_size", 85000);
    max_points_ = flyco::parameter_or<int>(nh, "max_points", 1024);

    // 加载 set_filter_range 所需参数
    filter_min_x_ = flyco::parameter_or<double>(nh, "cluster_min_x", std::numeric_limits<double>::lowest());
    filter_max_x_ = flyco::parameter_or<double>(nh, "cluster_max_x", std::numeric_limits<double>::max());
    filter_min_y_ = flyco::parameter_or<double>(nh, "cluster_min_y", std::numeric_limits<double>::lowest());
    filter_max_y_ = flyco::parameter_or<double>(nh, "cluster_max_y", std::numeric_limits<double>::max());
    filter_min_z_ = flyco::parameter_or<double>(nh, "cluster_min_z", std::numeric_limits<double>::lowest());
    filter_max_z_ = flyco::parameter_or<double>(nh, "cluster_max_z", std::numeric_limits<double>::max());

    // std::cout << "cluster_max_y " << filter_max_y_ << std::endl;

    // 新增高度滤波相关参数
    filter_grid_size_ = flyco::parameter_or<double>(nh, "filter_grid_size", 2.0);  // 默认值 2.0
    grid_max_height_ = flyco::parameter_or<double>(nh, "grid_max_height", 5.0);  // 默认值 5.0
    grid_height_threshold_ = flyco::parameter_or<double>(nh, "grid_height_threshold", 1.0);  // 默认值 1.0
    incremental_ = flyco::parameter_or<bool>(nh, "incremental", false);  // 默认值 1.0
    ground_filter_mode_ = flyco::parameter_or<std::string>(nh, "ground_filter_mode", "off");
    bool legacy_ground_filter_enabled = false;
    legacy_ground_filter_enabled = flyco::parameter_or<bool>(nh, "ground_filter_enabled", false);
    if (legacy_ground_filter_enabled && ground_filter_mode_ == "off") {
      ground_filter_mode_ = "grid";
    }
    ground_filter_margin_ = flyco::parameter_or<double>(nh, "ground_filter_margin", 0.3);
    cluster_algorithm_ = flyco::parameter_or<std::string>(nh, "cluster_algorithm", "euclidean");
    dbscan_core_min_points_ = flyco::parameter_or<int>(nh, "dbscan_core_min_points", 1);
    selection_temporal_weight_ = flyco::parameter_or<double>(nh, "selection_temporal_weight", 0.25);
    selection_min_score_ = flyco::parameter_or<double>(nh, "selection_min_score", 0.05);
    selection_max_centroid_jump_ = flyco::parameter_or<double>(nh, "selection_max_centroid_jump", 8.0);
    target_group_mode_ = flyco::parameter_or<std::string>(nh, "target_group_mode", "balanced");
    applyTargetGroupMode(target_group_mode_);

    // Advanced overrides are intentionally hidden from the default public
    // config. Keep reading them so old experiment launch files stay valid.
    target_group_enabled_ = flyco::parameter_or<bool>(nh, "target_group_enabled", target_group_enabled_);
    target_group_max_aux_clusters_ = flyco::parameter_or<int>(nh, "target_group_max_aux_clusters", target_group_max_aux_clusters_);
    target_group_candidate_limit_ = flyco::parameter_or<int>(nh, "target_group_candidate_limit", target_group_candidate_limit_);
    target_group_min_match_count_ = flyco::parameter_or<int>(nh, "target_group_min_match_count", target_group_min_match_count_);
    target_group_min_guide_coverage_ = flyco::parameter_or<double>(nh, "target_group_min_guide_coverage", target_group_min_guide_coverage_);
    target_group_min_cluster_precision_ = flyco::parameter_or<double>(nh, "target_group_min_cluster_precision", target_group_min_cluster_precision_);
    target_group_max_centroid_distance_ = flyco::parameter_or<double>(nh, "target_group_max_centroid_distance", target_group_max_centroid_distance_);
    target_refine_mode_ = flyco::parameter_or<std::string>(nh, "target_refine_mode", "guide_radius");
    target_refine_radius_ = flyco::parameter_or<double>(nh, "target_refine_radius", 20.0);
    process_rate_hz_ = flyco::parameter_or<double>(nh, "process_rate_hz", 10.0);
    process_rate_hz_ = std::max(1.0, process_rate_hz_);
    sync_queue_size_ = flyco::parameter_or<int>(nh, "sync_queue_size", 5);
    sync_queue_size_ = std::max(1, sync_queue_size_);
  }

  void applyTargetGroupMode(const std::string &mode) {
    std::string normalized = mode;
    std::transform(normalized.begin(), normalized.end(), normalized.begin(),
                   [](unsigned char ch) {
                     return static_cast<char>(std::tolower(ch));
                   });

    target_group_enabled_ = true;
    target_group_max_aux_clusters_ = 3;
    target_group_candidate_limit_ = 12;
    target_group_min_match_count_ = 20;
    target_group_min_guide_coverage_ = 0.04;
    target_group_min_cluster_precision_ = 0.08;
    target_group_max_centroid_distance_ = 20.0;

    if (normalized == "off" || normalized == "disabled" ||
        normalized == "false") {
      target_group_enabled_ = false;
      target_group_max_aux_clusters_ = 0;
      target_group_candidate_limit_ = 5;
    } else if (normalized == "conservative") {
      target_group_max_aux_clusters_ = 2;
      target_group_candidate_limit_ = 8;
      target_group_min_match_count_ = 30;
      target_group_min_guide_coverage_ = 0.06;
      target_group_min_cluster_precision_ = 0.12;
      target_group_max_centroid_distance_ = 15.0;
    } else if (normalized == "aggressive") {
      target_group_max_aux_clusters_ = 4;
      target_group_candidate_limit_ = 16;
      target_group_min_match_count_ = 10;
      target_group_min_guide_coverage_ = 0.025;
      target_group_min_cluster_precision_ = 0.05;
      target_group_max_centroid_distance_ = 25.0;
    } else if (normalized != "balanced") {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("flyco.sam_tracking"), "Unknown target_group_mode='" << mode
                                                    << "', using balanced");
    }
  }

  void checkAndLoadPointClouds(const rclcpp::Node::SharedPtr&nh) {
    // 读取 inheritance_enable 参数
    bool inheritance_enable = false;
    inheritance_enable = flyco::parameter_or<bool>(nh, "inheritance_enable", false);

    // std::cout << "inheritance_enable  " << inheritance_enable << std::endl;
    // 如果 inheritance_enable 为 true，则加载点云文件
    if (inheritance_enable) {
      std::string inheritance_all_pc_path, inheritance_target_pc_path,
          inheritance_raw_all_point_path;

      inheritance_all_pc_path = flyco::parameter_or<std::string>(nh, "inheritance_all_pc_path", "");
      inheritance_target_pc_path = flyco::parameter_or<std::string>(nh, "inheritance_target_pc_path", "");
      inheritance_raw_all_point_path = flyco::parameter_or<std::string>(nh, "inheritance_raw_all_point_path", "");

      // 检查路径是否有效
      if (inheritance_all_pc_path.empty() ||
          inheritance_target_pc_path.empty() ||
          inheritance_raw_all_point_path.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("flyco.sam_tracking"), "Inheritance enabled, but one or more file paths are empty. "
            "Please check the configuration.");
        return;
      }

      // 调用 PointCloudCluster 的 load_pc_from_file 函数加载点云
      if (cluster_.load_pc_from_file(inheritance_all_pc_path,
                                     inheritance_raw_all_point_path,
                                     inheritance_target_pc_path)) {
        RCLCPP_INFO(rclcpp::get_logger("flyco.sam_tracking"), "Point clouds loaded successfully from the provided paths.");
      } else {
        RCLCPP_INFO(rclcpp::get_logger("flyco.sam_tracking"), "Point clouds loaded Failed from");
      }

    } else {
      RCLCPP_INFO(rclcpp::get_logger("flyco.sam_tracking"), "Inheritance is disabled. No point clouds loaded from file.");
    }
  }

  // 配置订阅器
  void setupSubscribers(const rclcpp::Node::SharedPtr&nh) {
    target_point_sub_ =
        nh->create_subscription<sensor_msgs::msg::PointCloud2>(target_point_topic_, rclcpp::QoS(1).reliable(), std::bind(&PointCloudClusterManager::targetPointCallback, this, std::placeholders::_1));

    // 使用 message_filters 配置同步器
    cloud_sub_ =
        std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
            nh, all_point_topic_, rclcpp::QoS(sync_queue_size_).reliable().get_rmw_qos_profile());

    // 订阅 /stop_save 话题
    stop_save_sub_ = nh->create_subscription<std_msgs::msg::Bool>("/stop_save", rclcpp::QoS(1).reliable(), std::bind(&PointCloudClusterManager::stopSaveCallback, this, std::placeholders::_1));

    pose_sub_ =
        std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(
            nh, "/cluster/pose", rclcpp::QoS(sync_queue_size_).reliable().get_rmw_qos_profile());

    sync_cloud_pose_ = std::make_shared<message_filters::Synchronizer<
        message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>>(
        message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>(sync_queue_size_),
        *cloud_sub_, *pose_sub_);

    sync_cloud_pose_->registerCallback(std::bind(
        &PointCloudClusterManager::allPointPoseCallback, this, std::placeholders::_1, std::placeholders::_2));
  }

  void stopSaveCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg) {
    if (msg->data) {
      // 保存点云到文件
      std::string all_pc_path, raw_all_pc_path, target_pc_path;

      // 从参数服务器中获取保存路径
      const auto& nh = node_;
      all_pc_path = flyco::parameter_or<std::string>(nh, "save_all_pc_path", "/tmp/all_point_cloud.pcd");
      raw_all_pc_path = flyco::parameter_or<std::string>(nh, "save_raw_all_pc_path", "/tmp/raw_all_point_cloud.pcd");
      target_pc_path = flyco::parameter_or<std::string>(nh, "save_target_pc_path", "/tmp/target_point_cloud.pcd");

      ensureParentDirectory(all_pc_path);
      ensureParentDirectory(raw_all_pc_path);
      ensureParentDirectory(target_pc_path);

      RCLCPP_INFO(rclcpp::get_logger("flyco.sam_tracking"), "Saving point clouds to files...");
      if (cluster_.save_pc_to_file(all_pc_path, raw_all_pc_path,
                                   target_pc_path)) {
        RCLCPP_INFO(rclcpp::get_logger("flyco.sam_tracking"), "Point clouds saved successfully.");
      } else {
        RCLCPP_INFO(rclcpp::get_logger("flyco.sam_tracking"), "Point clouds saved faied.");
      }
    }
  }

  void ensureParentDirectory(const std::string &file_path) {
    if (file_path.empty()) {
      return;
    }
    boost::filesystem::path parent =
        boost::filesystem::path(file_path).parent_path();
    if (parent.empty()) {
      return;
    }
    boost::system::error_code ec;
    boost::filesystem::create_directories(parent, ec);
    if (ec) {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("flyco.sam_tracking"), "Failed to create directory for path "
                      << file_path << ": " << ec.message());
    }
  }

  // 配置发布器
  void setupPublishers(const rclcpp::Node::SharedPtr&nh) {
    clustered_target_pc_pub_ =
        nh->create_publisher<sensor_msgs::msg::PointCloud2>("/cluster/target_pc", rclcpp::QoS(1).reliable());
    accumulated_target_pc_pub_ =
        nh->create_publisher<sensor_msgs::msg::PointCloud2>("/cluster/target_pc_accumulated", rclcpp::QoS(1).reliable());
    env_pc_pub_ = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/cluster/env_pc", rclcpp::QoS(1).reliable());
    pose_pub_ = nh->create_publisher<nav_msgs::msg::Odometry>("/cluster/target_pose", rclcpp::QoS(1).reliable());

    rgb_pc_pub_ =
        nh->create_publisher<sensor_msgs::msg::PointCloud2>("/cluster/debug_rgb_pc", rclcpp::QoS(1).reliable());

    // Debug 发布器
    all_pc_pub_ =
        nh->create_publisher<sensor_msgs::msg::PointCloud2>("/cluster/debug_all_pc", rclcpp::QoS(1).reliable());
    target_pc_pub_ =
        nh->create_publisher<sensor_msgs::msg::PointCloud2>("/cluster/debug_target_pc", rclcpp::QoS(1).reliable());
  }

  // 初始化聚类器
  void initializeCluster() {
    // 设置已有参数
    cluster_.set_min_height(min_height_);
    cluster_.set_cluster_params(cluster_tolerance_, min_cluster_size_,
                                max_cluster_size_);
    cluster_.set_resolution(resolution_);
    cluster_.set_max_points(max_points_);
    cluster_.set_filter_range(filter_min_x_, filter_max_x_, filter_min_y_,
                              filter_max_y_, filter_min_z_, filter_max_z_);
    cluster_.set_cluster_algorithm(cluster_algorithm_);
    cluster_.set_dbscan_core_min_points(dbscan_core_min_points_);
    cluster_.set_selection_params(selection_temporal_weight_,
                                  selection_min_score_,
                                  selection_max_centroid_jump_);
    cluster_.set_target_group_params(
        target_group_enabled_, target_group_max_aux_clusters_,
        target_group_candidate_limit_, target_group_min_match_count_,
        target_group_min_guide_coverage_, target_group_min_cluster_precision_,
        target_group_max_centroid_distance_);
    cluster_.set_target_refine_params(target_refine_mode_,
                                      target_refine_radius_);

    // 设置新增高度滤波相关参数
    cluster_.set_ground_filter_mode(ground_filter_mode_);
    cluster_.set_ground_filter_margin(ground_filter_margin_);
    cluster_.set_filter_grid_size(filter_grid_size_);
    cluster_.set_grid_max_height(grid_max_height_);
    cluster_.set_grid_height_threshold(grid_height_threshold_);
  }

  void allPointPoseCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg,
                            const nav_msgs::msg::Odometry::ConstSharedPtr &pose) {
    if (has_pending_synced_pair_) {
      ++dropped_synced_pair_count_;
    }
    pending_cloud_msg_ = msg;
    pending_pose_msg_ = pose;
    has_pending_synced_pair_ = true;
  }

  void processLatestSyncedDataTimer() {
    if (!has_pending_synced_pair_ || pending_cloud_msg_ == nullptr ||
        pending_pose_msg_ == nullptr) {
      return;
    }

    sensor_msgs::msg::PointCloud2::ConstSharedPtr msg = pending_cloud_msg_;
    nav_msgs::msg::Odometry::ConstSharedPtr pose = pending_pose_msg_;
    pending_cloud_msg_.reset();
    pending_pose_msg_.reset();
    has_pending_synced_pair_ = false;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud);
    cluster_.update_cluster(cloud);
    rclcpp::Time cloud_time = msg->header.stamp;

    RCLCPP_INFO_STREAM_THROTTLE(rclcpp::get_logger("flyco.sam_tracking"), flyco::steady_logging_clock(), static_cast<int64_t>((1.0) * 1000), "Cluster backend=" << cluster_.cluster_algorithm()
                                << " input_points="
                                << cluster_.last_cluster_input_size()
                                << " clusters=" << cluster_.last_cluster_count()
                                << " guide_points="
                                << cluster_.last_target_guide_size()
                                << " best_cluster_points="
                                << cluster_.last_best_cluster_size()
                                << " dropped_synced_pairs="
                                << dropped_synced_pair_count_
                                << " runtime_ms="
                                << cluster_.last_cluster_runtime_ms());

    // 发布聚类点云和调试点云
    publishClusteredPointClouds(cloud_time);
    publishDebugPointClouds(cloud_time);

    // 初始化姿态消息
    nav_msgs::msg::Odometry::SharedPtr pub_pose(new nav_msgs::msg::Odometry(*pose));
    pub_pose->header.frame_id = "world";

    // 发布姿态消息
    pose_pub_->publish(*pub_pose);
  }

  // 目标点云的回调
  void targetPointCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud);
    cluster_.update_target(cloud);
  }

  // 定时发布彩色点云
  void colorPCTimerCallback() {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_pc(
        new pcl::PointCloud<pcl::PointXYZRGB>());
    if (cluster_.get_color_pc_debug(rgb_pc)) {
      publishPointCloud<pcl::PointXYZRGB>(rgb_pc, rgb_pc_pub_, "world",
                                          node_->now());
    }
  }

  // 发布聚类点云
  void publishClusteredPointClouds(const rclcpp::Time &stamp) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_pc(
        new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr env_pc(
        new pcl::PointCloud<pcl::PointXYZ>());
    const bool current_ok =
        cluster_.get_cluster_pointcloud(target_pc, env_pc, incremental_);
    if (current_ok) {
      auto logCloudStats = [](const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
        if (cloud == nullptr || cloud->empty()) {
          return std::string("empty");
        }
        std::vector<float> z_values;
        z_values.reserve(cloud->size());
        for (const auto &point : cloud->points) {
          if (std::isfinite(point.z)) {
            z_values.push_back(point.z);
          }
        }
        if (z_values.empty()) {
          return std::string("no_finite_z");
        }
        auto percentile = [&z_values](double q) {
          q = std::min(1.0, std::max(0.0, q));
          std::vector<float> sorted = z_values;
          std::sort(sorted.begin(), sorted.end());
          const double pos = q * static_cast<double>(sorted.size() - 1);
          const auto lower = static_cast<std::size_t>(std::floor(pos));
          const auto upper = static_cast<std::size_t>(std::ceil(pos));
          if (lower == upper) {
            return static_cast<double>(sorted[lower]);
          }
          const double ratio = pos - static_cast<double>(lower);
          return static_cast<double>(sorted[lower]) * (1.0 - ratio) +
                 static_cast<double>(sorted[upper]) * ratio;
        };
        const auto minmax =
            std::minmax_element(z_values.begin(), z_values.end());
        const auto min_it = minmax.first;
        const auto max_it = minmax.second;
        std::ostringstream oss;
        oss.setf(std::ios::fixed);
        oss.precision(3);
        oss << "n=" << z_values.size() << " z[min/p1/p5/med/p95/max]="
            << *min_it << "/" << percentile(0.01) << "/" << percentile(0.05)
            << "/" << percentile(0.50) << "/" << percentile(0.95) << "/"
            << *max_it;
        return oss.str();
      };
      RCLCPP_INFO_STREAM_THROTTLE(rclcpp::get_logger("flyco.sam_tracking"), flyco::steady_logging_clock(), static_cast<int64_t>((1.0) * 1000), "cluster output stats: target{" << logCloudStats(target_pc)
                                               << "} env{"
                                               << logCloudStats(env_pc) << "}");
      publishPointCloud<pcl::PointXYZ>(target_pc, clustered_target_pc_pub_,
                                       "world", stamp);
      publishPointCloud<pcl::PointXYZ>(env_pc, env_pc_pub_, "world", stamp);
    } else {
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("flyco.sam_tracking"),
                           flyco::steady_logging_clock(), 5000,
                           "Failed to get clustered point clouds.");
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_target_pc(
        new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_env_pc(
        new pcl::PointCloud<pcl::PointXYZ>());
    const bool accumulated_ok = cluster_.get_cluster_pointcloud(
        accumulated_target_pc, accumulated_env_pc, false);
    if (accumulated_ok) {
      publishPointCloud<pcl::PointXYZ>(accumulated_target_pc,
                                       accumulated_target_pc_pub_, "world",
                                       stamp);
    } else if (!current_ok) {
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("flyco.sam_tracking"),
                           flyco::steady_logging_clock(), 5000,
                           "Failed to get accumulated clustered target point cloud.");
    }
  }

  // 发布调试点云
  void publishDebugPointClouds(const rclcpp::Time &stamp) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr all_pc(
        new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_pc(
        new pcl::PointCloud<pcl::PointXYZ>());
    if (cluster_.get_pc_debug(all_pc, target_pc)) {
      publishPointCloud<pcl::PointXYZ>(all_pc, all_pc_pub_, "world", stamp);
      publishPointCloud<pcl::PointXYZ>(target_pc, target_pc_pub_, "world",
                                       stamp);
    } else {
      RCLCPP_WARN(rclcpp::get_logger("flyco.sam_tracking"), "Failed to get point clouds.");
    }
  }

  // 辅助方法：发布点云
  template <typename PointT>
  void publishPointCloud(const typename pcl::PointCloud<PointT>::Ptr &cloud,
                         rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr&pub, const std::string &frame_id,
                         const rclcpp::Time &stamp) {
    if (cloud->empty()) {
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("flyco.sam_tracking"),
                           flyco::steady_logging_clock(), 5000,
                           "Attempted to publish an empty point cloud.");
      return;
    }

    cloud->width = cloud->size();
    cloud->height = 1;
    cloud->is_dense = true;

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header.frame_id = frame_id;
    output.header.stamp = stamp;
    pub->publish(output);
  }

 private:
  // ROS 相关
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr target_point_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_save_sub_;  // 新增订阅器
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr all_pc_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr target_pc_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clustered_target_pc_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr accumulated_target_pc_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rgb_pc_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr env_pc_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub_;
  rclcpp::TimerBase::SharedPtr process_timer_;
  rclcpp::TimerBase::SharedPtr color_timer_;

  // 参数
  std::string all_point_topic_, target_point_topic_;
  double min_height_, cluster_tolerance_, resolution_;
  int min_cluster_size_, max_cluster_size_, max_points_;

  double filter_min_x_, filter_max_x_;
  double filter_min_y_, filter_max_y_;
  double filter_min_z_, filter_max_z_;

  double filter_grid_size_; /**< The grid size for height filtering. */
  double grid_max_height_;  /**< The maximum height for height filtering. */
  double grid_height_threshold_; /**< The threshold for height filtering. */

  bool incremental_;
  std::string ground_filter_mode_;
  double ground_filter_margin_;
  std::string cluster_algorithm_;
  int dbscan_core_min_points_;
  double selection_temporal_weight_;
  double selection_min_score_;
  double selection_max_centroid_jump_;
  std::string target_group_mode_;
  bool target_group_enabled_;
  int target_group_max_aux_clusters_;
  int target_group_candidate_limit_;
  int target_group_min_match_count_;
  double target_group_min_guide_coverage_;
  double target_group_min_cluster_precision_;
  double target_group_max_centroid_distance_;
  std::string target_refine_mode_;
  double target_refine_radius_;
  double process_rate_hz_;
  int sync_queue_size_;

  std::shared_ptr<message_filters::Synchronizer<
      message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2,
                                                      nav_msgs::msg::Odometry>>>
      sync_cloud_pose_;

  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>
      cloud_sub_;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> pose_sub_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pending_cloud_msg_;
  nav_msgs::msg::Odometry::ConstSharedPtr pending_pose_msg_;
  bool has_pending_synced_pair_ = false;
  std::size_t dropped_synced_pair_count_ = 0;

  // 聚类器
  PointCloudCluster cluster_;
  rclcpp::Node::SharedPtr node_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "sam_tracking_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  {
    PointCloudClusterManager processor(node);
    rclcpp::spin(node);
  }
  node.reset();
  if (rclcpp::ok()) rclcpp::shutdown();
  return 0;
}
