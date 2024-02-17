#ifndef PATH_PLANNER_SERVER__ATTACH_SERVER_HPP_
#define PATH_PLANNER_SERVER__ATTACH_SERVER_HPP_

#include "geometry_msgs/msg/twist.hpp"
#include "path_planner_server/srv/go_to_loading.hpp"
#include "rclcpp/callback_group.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/detail/string__struct.hpp"
#include "std_msgs/msg/string.hpp"

#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/subscription.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>

namespace path_planner_server {

using namespace std::chrono_literals;

using LaserScan = sensor_msgs::msg::LaserScan;
using GoToLoading = path_planner_server::srv::GoToLoading;
using Twist = geometry_msgs::msg::Twist;

enum class CenterPublishMode { Off, On, Once };

class AttachServer : public rclcpp::Node {
public:
  explicit AttachServer(
      const std::string &node_name = kNodeName,
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions{});

private:
  constexpr static char kNodeName[]{"approach_service_server"};
  constexpr static char kServiceName[]{"approach_shelf"};
  constexpr static char kScanTopicName[]{"/scan"};
  constexpr static char kCmdTopicName[]{
      "diffbot_base_controller/cmd_vel_unstamped"};
  constexpr static char kElevatorUpTopicName[]{"elevator_up"};

  constexpr static char kOdomFrame[]{"odom"};
  constexpr static char kBaseLinkFrame[]{"robot_base_link"};
  constexpr static char kScanFrame[]{"robot_front_laser_base_link"};
  constexpr static char kCartFrame[]{"cart_frame"};
  constexpr static char kPublishFrame[]{"odom"};

  constexpr static double kAngleLeftMin{-1.5708}; // [rad]
  constexpr static double kAngleRightMax{1.5708}; // [rad]
  constexpr static auto kTimerPeriod{100ms};
  constexpr static double kIntensityThreshold{8000};
  constexpr static double kMinLegsAngularDistance{0.5};
  constexpr static double kKpYaw{1.0};
  constexpr static double kKpDdistance{0.5};
  constexpr static double kGoalPosTol{8e-2};
  constexpr static auto kMotionTimeout{10s};

  void subscription_cb(const std::shared_ptr<const LaserScan> msg);
  void service_cb(const std::shared_ptr<GoToLoading::Request> req,
                  const std::shared_ptr<GoToLoading::Response> res);

  std::optional<Eigen::Isometry3d> getTransform(const std::string &source_frame,
                                                const std::string &dest_frame);
  bool
  moveToGoal(std::function<std::optional<std::pair<double, double>>()> getGoal);

  rclcpp::CallbackGroup::SharedPtr group_1_;
  rclcpp::CallbackGroup::SharedPtr group_2_;
  rclcpp::Subscription<LaserScan>::SharedPtr subscription_{};
  rclcpp::Publisher<Twist>::SharedPtr publisher_{};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr elevator_up_publisher_{};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_{};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{};
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Service<GoToLoading>::SharedPtr service_{};

  std::atomic<CenterPublishMode> publish_mode_{CenterPublishMode::Off};
  std::atomic<bool> center_published_{false};
  std::optional<Eigen::Isometry3d> pub_to_center_{std::nullopt};
};

} // namespace path_planner_server

#endif // PATH_PLANNER_SERVER__ATTACH_SERVER_HPP_