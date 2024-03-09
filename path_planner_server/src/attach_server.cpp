#include "path_planner_server/attach_server.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "rmw/qos_profiles.h"
#include "rmw/types.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/detail/empty__struct.hpp"
#include "std_msgs/msg/detail/string__struct.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <tf2_eigen/tf2_eigen.h>

#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <string>

namespace path_planner_server {

AttachServer::AttachServer(const std::string &node_name,
                           const rclcpp::NodeOptions &options)
    : Node{node_name, options},
      group_1_{this->create_callback_group(
          rclcpp::CallbackGroupType::MutuallyExclusive)},
      group_2_{this->create_callback_group(
          rclcpp::CallbackGroupType::MutuallyExclusive)},
      subscription_{this->create_subscription<LaserScan>(
          kScanTopicName, 1,
          std::bind(&AttachServer::subscription_cb, this,
                    std::placeholders::_1),
          [this]() {
            rclcpp::SubscriptionOptions options;
            options.callback_group = group_1_;
            return options;
          }())},
      publisher_{this->create_publisher<Twist>(kCmdTopicName, 1)},
      tf_buffer_{std::make_unique<tf2_ros::Buffer>(this->get_clock())},
      tf_listener_{
          std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_)},
      tf_broadcaster_{std::make_shared<tf2_ros::TransformBroadcaster>(this)},
      service_{this->create_service<Trigger>(
          kServiceName,
          std::bind(&AttachServer::service_cb, this, std::placeholders::_1,
                    std::placeholders::_2),
          rmw_qos_profile_services_default, group_2_)} {
  rcl_interfaces::msg::ParameterDescriptor
      elevator_up_string_param_description{};
  elevator_up_string_param_description.description =
      "If true, publish message of type std_msgs::msg::String to /elevator_up."
      "If false, publish std_msgs::msg::Empty";
  this->declare_parameter<bool>(kElevatorUpStringParamName, false,
                                elevator_up_string_param_description);
  bool string_publisher{};
  this->get_parameter(kElevatorUpStringParamName, string_publisher);

  rclcpp::QoS qos{1};
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

  if (string_publisher) {
    elevator_up_string_publisher_ =
        this->create_publisher<std_msgs::msg::String>(kElevatorUpTopicName,
                                                      qos);
  } else {
    elevator_up_empty_publisher_ =
        this->create_publisher<std_msgs::msg::Empty>(kElevatorUpTopicName, qos);
  }
  RCLCPP_INFO(this->get_logger(),
              "Started %s service server. Will publish %s to the %s topic.",
              kServiceName,
              string_publisher ? "std_msgs::msg::String"
                               : "std_msgs::msg::Empty",
              kElevatorUpTopicName);
}

void AttachServer::subscription_cb(const std::shared_ptr<const LaserScan> msg) {
  if (publish_mode_ == CenterPublishMode::Off) {
    return;
  }

  const auto angleToIndex{[msg](float angle) {
    return static_cast<size_t>(
        std::ceil((angle - msg->angle_min) / msg->angle_increment));
  }};
  const auto i0{0};
  const auto i1{angleToIndex(0.0)};
  const auto i2{msg->ranges.size() - 1};
  const auto compLegCenter{
      [msg](size_t i_0,
            size_t i_f) -> std::optional<std::pair<double, double>> {
        size_t num_points{0};
        double x{0.0};
        double y{0.0};
        double angle{};
        for (auto i{i_0}; i < i_f; ++i) {
          if (msg->range_min <= msg->ranges[i] &&
              msg->ranges[i] <= msg->range_max &&
              msg->intensities[i] >= kIntensityThreshold) {
            angle = msg->angle_min + i * msg->angle_increment;
            x += msg->ranges[i] * std::cos(angle);
            y += msg->ranges[i] * std::sin(angle);
            ++num_points;
          }
        }
        if (num_points) {
          return {{x / num_points, y / num_points}};
        } else {
          return std::nullopt;
        }
      }};

  const auto center_left{compLegCenter(i0, i1)};
  const auto center_right{compLegCenter(i1, i2)};
  if (!center_left) {
    RCLCPP_DEBUG(this->get_logger(),
                 "[subscription_cb(): Left leg not detected, returning.]");
  }
  if (!center_right) {
    RCLCPP_DEBUG(this->get_logger(),
                 "[subscription_cb(): Right leg not detected, returning.]");
  }

  Eigen::Isometry3d pub_to_center;
  if (center_left && center_right) {
    std::optional<Eigen::Isometry3d> pub_to_scan{
        getTransform(kPublishFrame, kScanFrame)};
    if (!pub_to_scan) {
      return;
    }
    Eigen::Isometry3d scan_to_center = Eigen::Isometry3d::Identity();
    scan_to_center.translation() =
        Eigen::Vector3d{(center_left->first + center_right->first) / 2,
                        (center_left->second + center_right->second) / 2, 0};
    Eigen::Vector3d y{center_left->first - center_right->first,
                      center_left->second - center_right->second, 0};
    y.normalize();
    Eigen::Vector3d z{pub_to_scan->linear()(2, 0), pub_to_scan->linear()(2, 1),
                      pub_to_scan->linear()(2, 2)};
    Eigen::Vector3d x{y.cross(z)};
    scan_to_center.linear() =
        (Eigen::Matrix3d() << x, y, z).finished().transpose();

    pub_to_center = pub_to_scan.value() * scan_to_center;
    pub_to_center_ = pub_to_center;
  } else if (pub_to_center_) {
    RCLCPP_DEBUG(this->get_logger(),
                 "subscription_cb(): Publishing cached transform.");
    pub_to_center = *pub_to_center_;
  } else {
    return;
  }

  geometry_msgs::msg::TransformStamped pub_to_center_msg{
      tf2::eigenToTransform(pub_to_center)};
  pub_to_center_msg.header.stamp = this->get_clock()->now();
  pub_to_center_msg.header.frame_id = kPublishFrame;
  pub_to_center_msg.child_frame_id = kCartFrame;
  tf_broadcaster_->sendTransform(pub_to_center_msg);
  /*if (!center_published_) {
    tf_buffer_->setTransform(pub_to_center_msg, "default_authority", true);
  }*/
  center_published_ = true;

  if (publish_mode_ == CenterPublishMode::Once) {
    publish_mode_ = CenterPublishMode::Off;
  }
}

void AttachServer::service_cb(const std::shared_ptr<Trigger::Request> /*req*/,
                              const std::shared_ptr<Trigger::Response> res) {
  RCLCPP_INFO(this->get_logger(), "%s service called", kServiceName);

  res->success = false;

  // Publish center frame.

  center_published_ = false;
  publish_mode_ = CenterPublishMode::On;
  while (!center_published_) { /*spin*/
  }

  // Perform the movement if requested.

  const auto get_fist_goal{[this]()
                               -> std::optional<std::pair<double, double>> {
    const auto base_to_cart{getTransform(kBaseLinkFrame, kCartFrame)};
    if (!base_to_cart) {
      return std::nullopt;
    }
    return {{base_to_cart->translation()[0], base_to_cart->translation()[1]}};
  }};
  const auto get_second_goal{
      [this]() -> std::optional<std::pair<double, double>> {
        const auto base_to_cart = getTransform(kBaseLinkFrame, kCartFrame);
        if (!base_to_cart) {
          return std::nullopt;
        }
        Eigen::Isometry3d cart_to_goal = Eigen::Isometry3d::Identity();
        cart_to_goal.translate(Eigen::Vector3d{0.3, 0.0, 0.0});
        const Eigen::Isometry3d base_to_goal{*base_to_cart * cart_to_goal};
        return {{base_to_goal.translation()[0], base_to_goal.translation()[1]}};
      }};

  // Move to center frame

  RCLCPP_INFO(this->get_logger(), "Moving towards the center point...");
  if (!moveToGoal(get_fist_goal)) {
    RCLCPP_WARN(this->get_logger(), "Moving towards the center point failed.");
    return;
  }

  // Move 30cm more

  RCLCPP_INFO(this->get_logger(), "Moving 30cm more...");
  if (!moveToGoal(get_second_goal)) {
    RCLCPP_WARN(this->get_logger(), "The 30cm forward movement failed.");
    return;
  }

  // Load shelf.

  RCLCPP_INFO(this->get_logger(), "Attaching shelf...");
  if (elevator_up_string_publisher_) {
    elevator_up_string_publisher_->publish(std_msgs::msg::String{});
  } else if (elevator_up_empty_publisher_) {
    elevator_up_empty_publisher_->publish(std_msgs::msg::Empty{});
  }
  rclcpp::sleep_for(std::chrono::seconds(kLoadShelfDelayDuration));

  RCLCPP_INFO(this->get_logger(), "Done.");
  publish_mode_ = CenterPublishMode::Off;
  res->success = true;
}

std::optional<Eigen::Isometry3d>
AttachServer::getTransform(const std::string &source_frame,
                           const std::string &dest_frame) {
  geometry_msgs::msg::TransformStamped t{};
  try {
    t = tf_buffer_->lookupTransform(
        source_frame, dest_frame,
        rclcpp::Time(0, 0, this->get_clock()->get_clock_type()));
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(),
                "getTransform(): Could not get transform from %s to %s: %s",
                source_frame.c_str(), dest_frame.c_str(), ex.what());

    return std::nullopt;
  }
  return tf2::transformToEigen(t);
}

bool AttachServer::moveToGoal(
    std::function<std::optional<std::pair<double, double>>()> getGoal) {
  const auto t_start{std::chrono::steady_clock::now()};
  std::optional<std::pair<double, double>> goal{};
  double error_distance{};
  double error_yaw{};

  while (true) {
    goal = getGoal();

    if (goal) {
      error_distance =
          std::sqrt(std::pow(goal->first, 2) + std::pow(goal->second, 2));
      error_yaw = std::atan2(goal->second, goal->first);
      if (error_distance < kGoalPosTol) {
        publisher_->publish(Twist{});
        return true;
      }
      Twist msg{};
      msg.angular.z = kKpYaw * error_yaw;
      msg.linear.x = kKpDdistance * error_distance;
      publisher_->publish(msg);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "moveToGoal(): Could not get transform to goal.");
      publisher_->publish(Twist{});
    }

    if (std::chrono::steady_clock::now() - t_start > kMotionTimeout) {
      RCLCPP_WARN(this->get_logger(), "[move_goal()] Timed out.");
      publisher_->publish(Twist{});
      return false;
    }
    std::this_thread::sleep_for(kTimerPeriod);
  }
}

} // namespace path_planner_server

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node{std::make_shared<path_planner_server::AttachServer>()};
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
}
