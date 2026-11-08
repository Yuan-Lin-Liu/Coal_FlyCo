// Mode lifecycle follows px4_ros2_ctrl/modes: NodeWithMode + ModeBase.
// Use the executor variant to retain FlyCo's existing PX4 Takeoff/return sequence.
#include <rclcpp/rclcpp.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/utils/message_version.hpp>
#include <px4_ros2/third_party/nlohmann/json.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <cmath>
#include <optional>

static std::string topicNamespace(rclcpp::Node & node)
{
  auto prefix = node.declare_parameter<std::string>("px4_topic_prefix", "/fmu");
  while (!prefix.empty() && prefix.back() == '/') {prefix.pop_back();}
  if (prefix.size() < 4 || prefix.substr(prefix.size() - 4) != "/fmu") {
    throw std::invalid_argument("px4_topic_prefix must end in /fmu");
  }
  return prefix.substr(0, prefix.size() - 3);
}

class FlyCoMode : public px4_ros2::ModeBase
{
public:
  explicit FlyCoMode(rclcpp::Node & node)
  : ModeBase(node, Settings("FlyCo").activateEvenWhileDisarmed(false), topicNamespace(node))
  {
    // Retain the user's Position-equivalent position requirement. The library
    // derives attitude, angular-rate and altitude requirements from this type.
    setpoint_ = std::make_shared<px4_ros2::TrajectorySetpointType>(*this, true);
    modeRequirements().local_position_relaxed = true;
    setSetpointUpdateRate(50.f);
    registration_pub_ = node.create_publisher<std_msgs::msg::String>(
      "/flyco/mode/registration", rclcpp::QoS(1).transient_local());
    local_sub_ = node.create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      topicNamespacePrefix() + "fmu/out/vehicle_local_position" +
      px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleLocalPosition>(),
      rclcpp::SensorDataQoS(), [this](px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr msg) {
        if (std::isfinite(msg->x) && std::isfinite(msg->y) && std::isfinite(msg->z) && std::isfinite(msg->heading)) {
          local_ = *msg;
        }
      });
    preview_sub_ = node.create_subscription<px4_msgs::msg::TrajectorySetpoint>(
      "/flyco/control/preview", rclcpp::QoS(1),
      [this](px4_msgs::msg::TrajectorySetpoint::ConstSharedPtr msg) {
        if (!isActive() || !std::isfinite(msg->yaw)) {return;}
        for (int i = 0; i < 3; ++i) {
          if (!std::isfinite(msg->position[i]) || std::isinf(msg->velocity[i]) || std::isinf(msg->acceleration[i])) {return;}
        }
        if (std::isinf(msg->yawspeed)) {return;}
        command_ = *msg;
        command_rx_ = std::chrono::steady_clock::now();
      });
    controller_sub_ = node.create_subscription<std_msgs::msg::String>(
      "/flyco/control/status", rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        const auto status = nlohmann::json::parse(msg->data, nullptr, false);
        controller_enabled_ = status.is_object() && status.contains("enabled") &&
          status["enabled"].is_boolean() && status["enabled"].get<bool>();
        if (!controller_enabled_) {command_.reset();}
      });
  }

  void publishRegistration(int executor_id)
  {
    std_msgs::msg::String msg;
    msg.data = nlohmann::json{{"mode_id", id()}, {"executor_id", executor_id}}.dump();
    registration_pub_->publish(msg);
  }

  void onActivate() override
  {
    command_.reset();
    following_ = false;
    hold_valid_ = false;
    captureHold();
  }

  void onDeactivate() override
  {
    command_.reset();
    following_ = hold_valid_ = false;
  }

  // No custom checkArmingAndRunConditions override: use the template's default
  // health reply and let PX4 evaluate the declared mode requirements.
  void updateSetpoint(float) override
  {
    if (!hold_valid_) {captureHold();}
    if (!hold_valid_) {return;}
    const bool have_target = controller_enabled_ && command_ &&
      std::chrono::steady_clock::now() - command_rx_ < std::chrono::milliseconds(500);
    px4_ros2::TrajectorySetpoint target;
    if (have_target) {
      const auto & msg = *command_;
      target.withPosition(Eigen::Vector3f(msg.position[0], msg.position[1], msg.position[2]));
      target.withVelocity(Eigen::Vector3f(msg.velocity[0], msg.velocity[1], msg.velocity[2]));
      target.withAcceleration(Eigen::Vector3f(msg.acceleration[0], msg.acceleration[1], msg.acceleration[2]));
      target.withYaw(msg.yaw).withYawRate(msg.yawspeed);
      following_ = true;
    } else {
      // Without a planner target, remain in the template's position-hold state.
      if (following_) {captureHold();}
      following_ = false;
      target.withPosition(hold_position_).withYaw(hold_yaw_);
    }
    setpoint_->update(target);
  }

private:
  void captureHold()
  {
    if (!local_) {return;}
    hold_position_ = Eigen::Vector3f(local_->x, local_->y, local_->z);
    hold_yaw_ = local_->heading;
    hold_valid_ = true;
  }

  std::shared_ptr<px4_ros2::TrajectorySetpointType> setpoint_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr registration_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_sub_;
  rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr preview_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr controller_sub_;
  std::optional<px4_msgs::msg::VehicleLocalPosition> local_;
  std::optional<px4_msgs::msg::TrajectorySetpoint> command_;
  std::chrono::steady_clock::time_point command_rx_{};
  Eigen::Vector3f hold_position_{Eigen::Vector3f::Zero()};
  float hold_yaw_{0.f};
  bool hold_valid_{false}, following_{false}, controller_enabled_{false};
};

class FlyCoExecutor : public px4_ros2::ModeExecutorBase
{
public:
  explicit FlyCoExecutor(FlyCoMode & mode) : ModeExecutorBase(Settings{}, mode), mode_(mode) {}
  bool doRegister()
  {
    if (!ModeExecutorBase::doRegister()) {return false;}
    mode_.publishRegistration(id());
    return true;
  }
  void onActivate() override {}
  void onDeactivate(DeactivateReason) override {}
private:
  FlyCoMode & mode_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  using Node = px4_ros2::NodeWithModeExecutor<FlyCoExecutor, FlyCoMode>;
  rclcpp::spin(std::make_shared<Node>("flyco_mode"));
  rclcpp::shutdown();
  return 0;
}
