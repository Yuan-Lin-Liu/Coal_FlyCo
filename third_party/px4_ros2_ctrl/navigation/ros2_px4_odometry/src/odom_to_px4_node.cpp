#include <rclcpp/rclcpp.hpp>
#include <px4_ros2/navigation/experimental/local_position_measurement_interface.hpp>
#include <px4_ros2/utils/frame_conversion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>
#include <cstddef>
#include <string>
#include <stdexcept>
#include <vector>

using namespace std::chrono_literals; // NOLINT

static px4_ros2::PoseFrame poseFrame(const rclcpp::Node & node)
{
  const auto value = node.get_parameter("pose_frame").as_string();
  if (value == "NED") {return px4_ros2::PoseFrame::LocalNED;}
  if (value == "FRD") {return px4_ros2::PoseFrame::LocalFRD;}
  throw std::invalid_argument("pose_frame must be NED or FRD");
}

/**
 * @class LocalNavigation
 * @brief A wrapper class for converting external pose data to PX4 local position measurements.
 *
 * This class supports multiple ROS message types and handles coordinate
 * frame conversions (e.g., ENU to NED) and application of extrinsic transformations.
 */
class LocalNavigation : public px4_ros2::LocalPositionMeasurementInterface
{
public:
  /**
   * @brief Constructor initializing the navigation interface and default extrinsics.
   * @param node Reference to the ROS node.
   */
  explicit LocalNavigation(rclcpp::Node & node)
  : LocalPositionMeasurementInterface(node, poseFrame(node),
      node.get_parameter("publish_velocity").as_bool() ?
      (poseFrame(node) == px4_ros2::PoseFrame::LocalNED ?
      px4_ros2::VelocityFrame::LocalNED : px4_ros2::VelocityFrame::LocalFRD) :
      px4_ros2::VelocityFrame::Unknown,
      node.get_parameter("topic_namespace_prefix").as_string())
  {
    // Initialize extrinsic parameters to identity
    extrinsic_translation_ = Eigen::Vector3d::Zero();
    extrinsic_rotation_ = Eigen::Quaterniond::Identity();
    use_pose_covariance_ = node.get_parameter("use_pose_covariance").as_bool();
    publish_velocity_ = node.get_parameter("publish_velocity").as_bool();
    default_variances_ = node.get_parameter("pose_variances").as_double_array();
    if (default_variances_.size() != 6 || !areValuesFinite(default_variances_)) {
      throw std::invalid_argument("pose_variances must contain six finite positive values");
    }
    for (const auto value : default_variances_) {
      if (value <= 0.) {throw std::invalid_argument("pose_variances must be positive");}
    }
    const double rate = node.get_parameter("publish_rate").as_double();
    if (!std::isfinite(rate) || rate < 0.) {throw std::invalid_argument("publish_rate must be nonnegative");}
    min_interval_ns_ = rate > 0. ? static_cast<int64_t>(1.e9 / rate) : 0;
    const auto output_topic = node.get_parameter("odometry_output_topic").as_string();
    if (!output_topic.empty()) {
      odom_pub_ = node.create_publisher<nav_msgs::msg::Odometry>(output_topic, 1);
      tf_pub_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);
    }
  }

  /**
   * @brief Process nav_msgs::msg::Odometry messages and update local position.
   * @param msg The received odometry message.
   */
  void updateLocalPosition(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    px4_ros2::LocalPositionMeasurement local_position_measurement {};

    if (!isTimestampValid(rclcpp::Time(msg->header.stamp), "odometry")) {
      return;
    }

    if (use_pose_covariance_ && !isPoseCovarianceValid(msg->pose.covariance, "odometry")) {
      return;
    }

    local_position_measurement.timestamp_sample = msg->header.stamp;

    if (!populatePoseMeasurement(msg->pose.pose, "odometry", local_position_measurement)) {
      return;
    }

    if (use_pose_covariance_) {
      assignPoseCovariance(msg->pose.covariance, local_position_measurement);
    } else {
      assignDefaultPoseVariance(local_position_measurement);
    }
    if (publish_velocity_) {populateVelocityMeasurement(msg->twist, local_position_measurement);}
    if (publishMeasurement(local_position_measurement, "odometry") && odom_pub_) {
      publishRosOdometry(*msg);
    }
  }

  /**
   * @brief Process geometry_msgs::msg::PoseStamped messages and update local position.
   * @param msg The received pose message.
   */
  void updateLocalPosition(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    px4_ros2::LocalPositionMeasurement local_position_measurement {};

    if (!isTimestampValid(rclcpp::Time(msg->header.stamp), "pose_stamped")) {
      return;
    }

    local_position_measurement.timestamp_sample = msg->header.stamp;

    if (!populatePoseMeasurement(msg->pose, "pose_stamped", local_position_measurement)) {
      return;
    }

    assignDefaultPoseVariance(local_position_measurement);
    publishMeasurement(local_position_measurement, "pose_stamped");
  }

  /**
   * @brief Process geometry_msgs::msg::PoseWithCovarianceStamped messages and update local position.
   * @param msg The received pose with covariance message.
   */
  void updateLocalPosition(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    px4_ros2::LocalPositionMeasurement local_position_measurement {};

    if (!isTimestampValid(rclcpp::Time(msg->header.stamp), "pose_with_covariance_stamped")) {
      return;
    }

    if (use_pose_covariance_ && !isPoseCovarianceValid(msg->pose.covariance, "pose_with_covariance_stamped")) {
      return;
    }

    local_position_measurement.timestamp_sample = msg->header.stamp;

    if (!populatePoseMeasurement(
        msg->pose.pose, "pose_with_covariance_stamped", local_position_measurement))
    {
      return;
    }

    if (use_pose_covariance_) {
      assignPoseCovariance(msg->pose.covariance, local_position_measurement);
    } else {
      assignDefaultPoseVariance(local_position_measurement);
    }
    publishMeasurement(local_position_measurement, "pose_with_covariance_stamped");
  }

  /**
   * @brief Set extrinsic transformation from mocap frame to robot body frame.
   * @param translation Translation vector [x, y, z] from mocap origin to robot body origin.
   * @param rotation_matrix Row-major 3x3 rotation matrix as a 9-element vector.
   */
  void setExtrinsicParameters(
    const std::vector<double> & translation,
    const std::vector<double> & rotation_matrix)
  {
    if (translation.size() != 3) {
      RCLCPP_ERROR(_node.get_logger(), "Translation vector must have 3 elements [x, y, z]");
      return;
    }
    if (rotation_matrix.size() != 9) {
      RCLCPP_ERROR(_node.get_logger(), "Rotation matrix must have 9 elements [r11, ..., r33]");
      return;
    }
    if (!areValuesFinite(translation)) {
      RCLCPP_ERROR(_node.get_logger(), "Extrinsic translation contains non-finite values");
      return;
    }
    if (!areValuesFinite(rotation_matrix)) {
      RCLCPP_ERROR(_node.get_logger(), "Extrinsic rotation matrix contains non-finite values");
      return;
    }

    extrinsic_translation_ = Eigen::Vector3d(translation[0], translation[1], translation[2]);

    Eigen::Matrix3d rotation_mat;
    rotation_mat << rotation_matrix[0], rotation_matrix[1], rotation_matrix[2],
      rotation_matrix[3], rotation_matrix[4], rotation_matrix[5],
      rotation_matrix[6], rotation_matrix[7], rotation_matrix[8];

    extrinsic_rotation_ = Eigen::Quaterniond(rotation_mat);
    if (!isQuaternionValid(extrinsic_rotation_)) {
      RCLCPP_ERROR(_node.get_logger(), "Extrinsic rotation matrix produced an invalid quaternion");
      extrinsic_rotation_ = Eigen::Quaterniond::Identity();
      return;
    }
    extrinsic_rotation_.normalize();
  }

private:
  static constexpr double kQuaternionNormEpsilon = 1.0e-6;
  static constexpr int kWarningThrottleMs = 1000;

  static bool areValuesFinite(const std::vector<double> & values)
  {
    for (const double value : values) {
      if (!std::isfinite(value)) {
        return false;
      }
    }

    return true;
  }

  static bool isQuaternionValid(const Eigen::Quaterniond & q)
  {
    return q.coeffs().allFinite() && std::isfinite(q.norm()) &&
           q.norm() > kQuaternionNormEpsilon;
  }

  static bool isQuaternionValid(const Eigen::Quaternionf & q)
  {
    return q.coeffs().allFinite() && std::isfinite(q.norm()) &&
           q.norm() > static_cast<float>(kQuaternionNormEpsilon);
  }

  static bool isQuaternionValid(const geometry_msgs::msg::Quaternion & q)
  {
    const double norm_sq = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
    return std::isfinite(q.w) && std::isfinite(q.x) && std::isfinite(q.y) &&
           std::isfinite(q.z) && std::isfinite(norm_sq) &&
           norm_sq > kQuaternionNormEpsilon * kQuaternionNormEpsilon;
  }

  static bool isVectorFinite(const geometry_msgs::msg::Point & v)
  {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
  }

  static bool isVectorFinite(const geometry_msgs::msg::Vector3 & v)
  {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
  }

  template<typename CovarianceT>
  static bool isCovarianceFinite(const CovarianceT & covariance)
  {
    for (const double value : covariance) {
      if (!std::isfinite(value)) {
        return false;
      }
    }

    return true;
  }

  void warnDrop(const char * source, const char * reason) const
  {
    RCLCPP_WARN_THROTTLE(
      _node.get_logger(), *_node.get_clock(), kWarningThrottleMs,
      "Dropping %s measurement: %s", source, reason);
  }

  void warnInvalidVariance(const char * source, const char * field_name, const double value) const
  {
    RCLCPP_WARN_THROTTLE(
      _node.get_logger(), *_node.get_clock(), kWarningThrottleMs,
      "Dropping %s measurement: %s variance must be finite and > 0 (value: %.6g)",
      source, field_name, value);
  }

  bool isTimestampValid(const rclcpp::Time & timestamp, const char * source) const
  {
    if (timestamp.nanoseconds() == 0) {
      warnDrop(source, "timestamp is empty");
      return false;
    }

    return true;
  }

  bool isPoseValueValid(const geometry_msgs::msg::Pose & pose, const char * source) const
  {
    if (!isVectorFinite(pose.position)) {
      warnDrop(source, "position contains non-finite values");
      return false;
    }

    if (!isQuaternionValid(pose.orientation)) {
      warnDrop(source, "orientation quaternion is non-finite or has near-zero norm");
      return false;
    }

    return true;
  }

  template<typename CovarianceT>
  bool isPositiveVariance(
    const CovarianceT & covariance, const std::size_t index, const char * source,
    const char * field_name) const
  {
    const double value = covariance[index];

    if (!std::isfinite(value) || value <= 0.0) {
      warnInvalidVariance(source, field_name, value);
      return false;
    }

    return true;
  }

  template<typename CovarianceT>
  bool isPoseCovarianceValid(const CovarianceT & covariance, const char * source) const
  {
    if (!isCovarianceFinite(covariance)) {
      warnDrop(source, "pose covariance contains non-finite values");
      return false;
    }

    return isPositiveVariance(covariance, 0, source, "position x") &&
           isPositiveVariance(covariance, 7, source, "position y") &&
           isPositiveVariance(covariance, 14, source, "position z") &&
           isPositiveVariance(covariance, 21, source, "roll") &&
           isPositiveVariance(covariance, 28, source, "pitch") &&
           isPositiveVariance(covariance, 35, source, "yaw");
  }

  template<typename CovarianceT>
  bool isTwistCovarianceValid(const CovarianceT & covariance, const char * source) const
  {
    if (!isCovarianceFinite(covariance)) {
      warnDrop(source, "twist covariance contains non-finite values");
      return false;
    }

    return isPositiveVariance(covariance, 0, source, "velocity x") &&
           isPositiveVariance(covariance, 7, source, "velocity y") &&
           isPositiveVariance(covariance, 14, source, "velocity z");
  }

  bool populatePoseMeasurement(
    const geometry_msgs::msg::Pose & pose, const char * source,
    px4_ros2::LocalPositionMeasurement & local_position_measurement) const
  {
    if (!isPoseValueValid(pose, source)) {
      return false;
    }

    const Eigen::Vector3d input_position(
      pose.position.x, pose.position.y, pose.position.z);
    Eigen::Quaterniond input_orientation(
      pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    input_orientation.normalize();

    const Eigen::Vector3d transformed_position = extrinsic_rotation_ * input_position +
      extrinsic_translation_;
    Eigen::Quaterniond transformed_orientation = extrinsic_rotation_ * input_orientation;

    if (!transformed_position.allFinite() || !isQuaternionValid(transformed_orientation)) {
      warnDrop(source, "transformed pose contains invalid values");
      return false;
    }

    transformed_orientation.normalize();

    // PX4 expects NED frame. Input ROS odometry/pose is treated as ENU.
    local_position_measurement.position_xy = Eigen::Vector2f(
      static_cast<float>(transformed_position.y()),
      static_cast<float>(transformed_position.x()));
    local_position_measurement.position_z = static_cast<float>(-transformed_position.z());

    Eigen::Quaternionf q_px4 = px4_ros2::attitudeEnuToNed(
      Eigen::Quaternionf(
        static_cast<float>(transformed_orientation.w()),
        static_cast<float>(transformed_orientation.x()),
        static_cast<float>(transformed_orientation.y()),
        static_cast<float>(transformed_orientation.z()))
    );

    if (!isQuaternionValid(q_px4)) {
      warnDrop(source, "converted PX4 attitude quaternion is invalid");
      return false;
    }

    q_px4.normalize();
    local_position_measurement.attitude_quaternion = q_px4;

    return true;
  }

  template<typename CovarianceT>
  void assignPoseCovariance(
    const CovarianceT & covariance,
    px4_ros2::LocalPositionMeasurement & local_position_measurement) const
  {
    local_position_measurement.position_xy_variance = Eigen::Vector2f(
      static_cast<float>(covariance[7]),
      static_cast<float>(covariance[0]));
    local_position_measurement.position_z_variance = static_cast<float>(covariance[14]);
    local_position_measurement.attitude_variance = Eigen::Vector3f(
      static_cast<float>(covariance[21]),
      static_cast<float>(covariance[28]),
      static_cast<float>(covariance[35]));
  }

  void assignDefaultPoseVariance(
    px4_ros2::LocalPositionMeasurement & local_position_measurement) const
  {
    local_position_measurement.position_xy_variance = Eigen::Vector2f(
      default_variances_[1], default_variances_[0]);
    local_position_measurement.position_z_variance = default_variances_[2];
    local_position_measurement.attitude_variance = Eigen::Vector3f(
      default_variances_[3], default_variances_[4], default_variances_[5]);
  }

  bool populateVelocityMeasurement(
    const geometry_msgs::msg::TwistWithCovariance & twist,
    px4_ros2::LocalPositionMeasurement & local_position_measurement) const
  {
    const char * velocity_source = "odometry velocity";

    if (!isVectorFinite(twist.twist.linear)) {
      warnDrop(velocity_source, "linear velocity contains non-finite values");
      return false;
    }

    if (!isTwistCovarianceValid(twist.covariance, velocity_source)) {
      return false;
    }

    const Eigen::Vector3d input_velocity(
      twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z);
    const Eigen::Vector3d transformed_velocity = extrinsic_rotation_ * input_velocity;

    if (!transformed_velocity.allFinite()) {
      warnDrop(velocity_source, "transformed linear velocity contains invalid values");
      return false;
    }

    local_position_measurement.velocity_xy = Eigen::Vector2f(
      static_cast<float>(transformed_velocity.y()),
      static_cast<float>(transformed_velocity.x()));
    local_position_measurement.velocity_z = static_cast<float>(-transformed_velocity.z());
    local_position_measurement.velocity_xy_variance = Eigen::Vector2f(
      static_cast<float>(twist.covariance[7]),
      static_cast<float>(twist.covariance[0]));
    local_position_measurement.velocity_z_variance = static_cast<float>(twist.covariance[14]);

    return true;
  }

  bool publishMeasurement(
    const px4_ros2::LocalPositionMeasurement & local_position_measurement,
    const char * source)
  {
    const auto sample_ns = local_position_measurement.timestamp_sample.nanoseconds();
    if (min_interval_ns_ > 0 && last_sample_ns_ > 0 && sample_ns - last_sample_ns_ < min_interval_ns_) {
      return false;
    }
    try {
      update(local_position_measurement);
      last_sample_ns_ = sample_ns;
      RCLCPP_DEBUG(
        _node.get_logger(),
        "Successfully sent %s update to navigation interface.", source);
    } catch (const px4_ros2::NavigationInterfaceInvalidArgument & e) {
      RCLCPP_ERROR_THROTTLE(
        _node.get_logger(),
        *_node.get_clock(), kWarningThrottleMs, "Exception caught while sending %s: %s",
        source, e.what());
      return false;
    }
    return true;
  }

  void publishRosOdometry(const nav_msgs::msg::Odometry & input)
  {
    auto output = input;
    output.header.frame_id = _node.get_parameter("output_frame").as_string();
    output.child_frame_id = _node.get_parameter("output_child_frame").as_string();
    const auto & p = input.pose.pose.position;
    const Eigen::Vector3d position = extrinsic_rotation_ * Eigen::Vector3d(p.x, p.y, p.z) + extrinsic_translation_;
    const auto & q = input.pose.pose.orientation;
    const Eigen::Quaterniond orientation = (extrinsic_rotation_ * Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized()).normalized();
    output.pose.pose.position.x = position.x();
    output.pose.pose.position.y = position.y();
    output.pose.pose.position.z = position.z();
    output.pose.pose.orientation.w = orientation.w();
    output.pose.pose.orientation.x = orientation.x();
    output.pose.pose.orientation.y = orientation.y();
    output.pose.pose.orientation.z = orientation.z();
    if (!use_pose_covariance_) {
      output.pose.covariance.fill(0.);
      for (std::size_t i = 0; i < 6; ++i) {output.pose.covariance[i * 7] = default_variances_[i];}
    }
    odom_pub_->publish(output);
    geometry_msgs::msg::TransformStamped tf;
    tf.header = output.header;
    tf.child_frame_id = output.child_frame_id;
    tf.transform.translation.x = position.x();
    tf.transform.translation.y = position.y();
    tf.transform.translation.z = position.z();
    tf.transform.rotation = output.pose.pose.orientation;
    tf_pub_->sendTransform(tf);
  }

  Eigen::Vector3d extrinsic_translation_;
  Eigen::Quaterniond extrinsic_rotation_;
  bool use_pose_covariance_{true};
  bool publish_velocity_{true};
  std::vector<double> default_variances_;
  int64_t min_interval_ns_{0};
  int64_t last_sample_ns_{0};
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_pub_;

};

/**
 * @class GenericOdometryNode
 * @brief ROS2 node for converting various pose/odometry messages into PX4 local position updates.
 *
 * This node subscribes to selected message types, applies extrinsic transformations,
 * and pushes position data to PX4 via the LocalNavigation interface.
 */
class GenericOdometryNode : public rclcpp::Node
{
public:
  GenericOdometryNode()
  : Node("generic_odometry_node")
  {
    // Declare parameters
    this->declare_parameter("message_type", "odometry");
    this->declare_parameter("odometry_topic", "/Odometry");
    this->declare_parameter("input_frame", "");
    this->declare_parameter("pose_frame", "NED");
    this->declare_parameter("topic_namespace_prefix", "");
    this->declare_parameter("use_pose_covariance", true);
    this->declare_parameter("publish_velocity", true);
    this->declare_parameter("pose_variances", std::vector<double>{1.e-9, 1.e-9, 1.e-9, 1.e-4, 1.e-4, 1.e-4});
    this->declare_parameter("publish_rate", 0.);
    this->declare_parameter("odometry_output_topic", "");
    this->declare_parameter("output_frame", "world");
    this->declare_parameter("output_child_frame", "base_link");
    // Declare extrinsic parameters
    this->declare_parameter("extrinsic_translation", std::vector<double>{0.0, 0.0, 0.0});
    this->declare_parameter(
      "extrinsic_rotation_matrix", std::vector<double>{
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0
    });

    _interface = std::make_unique<LocalNavigation>(*this);

    // Read and set extrinsics
    auto translation = this->get_parameter("extrinsic_translation").as_double_array();
    auto rotation_matrix = this->get_parameter("extrinsic_rotation_matrix").as_double_array();
    _interface->setExtrinsicParameters(translation, rotation_matrix);

    if (!_interface->doRegister()) {
      throw std::runtime_error("Registration failed");
    }

    // Set up QoS
    const rclcpp::QoS qos(rclcpp::QoS(1)
      .best_effort()
      .keep_last(1)
      .durability_volatile());

    // Subscribe to appropriate topic based on message type
    std::string message_type = this->get_parameter("message_type").as_string();

    if (message_type == "odometry") {
      odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        this->get_parameter("odometry_topic").as_string(),
        qos,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          const auto frame = this->get_parameter("input_frame").as_string();
          if (!frame.empty() && msg->header.frame_id != frame) {return;}
          _interface->updateLocalPosition(msg);
        });
      RCLCPP_INFO(
        this->get_logger(), "Subscribed to odometry topic: %s",
        this->get_parameter("odometry_topic").as_string().c_str());
    } else if (message_type == "pose_stamped") {
      pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        this->get_parameter("odometry_topic").as_string(),
        qos,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          _interface->updateLocalPosition(msg);
        });
      RCLCPP_INFO(
        this->get_logger(), "Subscribed to pose topic: %s",
        this->get_parameter("odometry_topic").as_string().c_str());
    } else if (message_type == "pose_with_covariance_stamped") {
      pose_with_covariance_sub_ =
        this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        this->get_parameter("odometry_topic").as_string(),
        qos,
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
          _interface->updateLocalPosition(msg);
        });
      RCLCPP_INFO(
        this->get_logger(), "Subscribed to pose with covariance topic: %s",
        this->get_parameter("odometry_topic").as_string().c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Unsupported message type: %s", message_type.c_str());
      RCLCPP_INFO(
        this->get_logger(),
        "Supported message types: odometry, pose_stamped, pose_with_covariance_stamped");
    }
  }

private:
  std::unique_ptr<LocalNavigation> _interface;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    pose_with_covariance_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GenericOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
