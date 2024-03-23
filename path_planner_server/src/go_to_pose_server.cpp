#include "rclcpp/logging.hpp"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <path_planner_server/action/go_to_pose.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <stdexcept>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

constexpr char kNodeName[]{"GoToPoseServer"};
constexpr char kActionName[]{"go_to_pose"};
constexpr char kOdometryTopicName[]{"/odom"};
constexpr char kVelocityCommandTopicName[]{"cmd_vel"};
constexpr static char kMapFrame[]{"map"};
constexpr static char kOdomFrame[]{"robot_odom"};

class GoToPose : public rclcpp::Node {
public:
  using GoToPoseAction = path_planner_server::action::GoToPose;
  using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPoseAction>;

  GoToPose()
      : Node{kNodeName}, tf_buffer_{std::make_unique<tf2_ros::Buffer>(
                             this->get_clock())},
        tf_listener_{
            std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_)},
        subscription_{this->create_subscription<nav_msgs::msg::Odometry>(
            kOdometryTopicName, 1,
            std::bind(&GoToPose::subscription_cb, this,
                      std::placeholders::_1))},
        publisher_{this->create_publisher<geometry_msgs::msg::Twist>(
            kVelocityCommandTopicName, 1)},

        action_server_{rclcpp_action::create_server<GoToPoseAction>(
            this, kActionName,
            std::bind(&GoToPose::handle_goal, this, std::placeholders::_1,
                      std::placeholders::_2),
            std::bind(&GoToPose::handle_cancel, this, std::placeholders::_1),
            std::bind(&GoToPose::handle_accepted, this,
                      std::placeholders::_1))} {
    RCLCPP_INFO(this->get_logger(), "Started %s action server.", kActionName);
  }

private:
  enum class State : uint8_t {
    inactive,
    starting,
    preempted,
    moving_to_goal,
    rotating_to_goal,
  };

  static_assert(std::atomic<State>::is_always_lock_free);

  // math constants
  constexpr static double kPi{3.1416};

  // settings
  constexpr static double kGoalPosTol{1e-1};
  constexpr static double kGoalThetaTol{1e-1};
  constexpr static double kLoopRate{5};      // Hz
  constexpr static double kLinearSpeed{0.2}; // m/s

  // subscription
  void subscription_cb(const nav_msgs::msg::Odometry::SharedPtr msg);

  // action
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const GoToPoseAction::Goal> goal);
  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleGoToPose> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleGoToPose> goal_handle);
  void execute(const std::shared_ptr<GoalHandleGoToPose> goal_handle);

  // tf
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_{};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_{};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_{};
  rclcpp_action::Server<GoToPoseAction>::SharedPtr action_server_;

  Eigen::Isometry2d desired_pos_; // in map frame
  Eigen::Isometry2d current_pos_; // in base link frame

  // thread synchronization for action preemption
  std::atomic<State> state_{State::inactive};
  mutable std::mutex preempt_lock_;
  mutable std::mutex odometry_lock_;
  std::condition_variable cv_;
};

void GoToPose::subscription_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
  tf2::Quaternion q{msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w};
  double yaw{};
  double pitch{};
  double roll{};

  tf2::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);

  std::lock_guard<std::mutex> lock{odometry_lock_};
  current_pos_ = Eigen::Translation2d{msg->pose.pose.position.x,
                                      msg->pose.pose.position.y} *
                 Eigen::Rotation2D{yaw};
}

rclcpp_action::GoalResponse
GoToPose::handle_goal(const rclcpp_action::GoalUUID & /*uuid*/,
                      std::shared_ptr<const GoToPoseAction::Goal> goal) {
  RCLCPP_INFO(this->get_logger(), "Received goal request.");
  if (state_.load() != State::inactive) {
    state_.store(State::preempted);
    std::unique_lock lock{preempt_lock_};
    cv_.wait(lock, [this] { return this->state_.load() == State::inactive; });
  }
  state_.store(State::starting);

  desired_pos_ = Eigen::Translation2d{goal->goal_pos.x, goal->goal_pos.y} *
                 Eigen::Rotation2Dd{goal->goal_pos.theta};
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GoToPose::handle_cancel(
    const std::shared_ptr<GoalHandleGoToPose> /*goal_handle*/) {
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal.");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void GoToPose::handle_accepted(
    const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
  using namespace std::placeholders;

  // this needs to return quickly to avoid blocking the executor, so spin up
  // a new thread
  std::thread{std::bind(&GoToPose::execute, this, _1), goal_handle}.detach();
}

void GoToPose::execute(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Executing goal");
  auto state_expected{State::starting};
  state_.compare_exchange_strong(state_expected, State::moving_to_goal);

  const auto set_to_inactive{[this] {
    publisher_->publish(geometry_msgs::msg::Twist{});
    std::lock_guard<std::mutex> lock{preempt_lock_};
    state_.store(State::inactive); // allow new goal
    cv_.notify_one();
  }};

  auto feedback{std::make_shared<GoToPoseAction::Feedback>()};
  auto result{std::make_shared<GoToPoseAction::Result>()};
  geometry_msgs::msg::Twist twist{};
  rclcpp::Rate loop_rate(kLoopRate);
  double dx{};
  double dy{};
  double ds{};
  double theta_des{};
  double dtheta_forward{}, dtheta_backward{};
  decltype(current_pos_) current_pos;
  std::optional<Eigen::Isometry2d> map_to_base{};

  while (rclcpp::ok()) {

    auto state{state_.load()};

    if (!goal_handle->is_canceling() && state != State::preempted) {
      try {
        const auto t{tf_buffer_->lookupTransform(
            kMapFrame, kOdomFrame,
            rclcpp::Time(0, 0, this->get_clock()->get_clock_type()))};
        Eigen::Isometry3d map_to_base_3d = tf2::transformToEigen(t);
        map_to_base = Eigen::Isometry2d{};
        map_to_base->linear() = map_to_base_3d.rotation().block<2, 2>(0, 0);
        map_to_base->translation() = map_to_base_3d.translation().head<2>();
      } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(),
                    "getTransform(): Could not get transform from %s to %s: %s",
                    kMapFrame, kOdomFrame, ex.what());
        if (!map_to_base) {
          continue;
        }
      }

      {
        std::lock_guard<std::mutex> lock{odometry_lock_};
        current_pos = map_to_base.value() * current_pos_;
      }

      feedback->current_pos.x = current_pos.translation()(0);
      feedback->current_pos.y = current_pos.translation()(1);
      feedback->current_pos.theta =
          Eigen::Rotation2Dd(current_pos.linear()).angle();
      RCLCPP_DEBUG(this->get_logger(), "Publish feedback");
      goal_handle->publish_feedback(feedback);
    }

    if (state == State::preempted) {
      result->status = false;
      goal_handle->abort(result);
      set_to_inactive();
      RCLCPP_INFO(this->get_logger(), "Goal preempted by new goal.");
      return;
    }
    if (goal_handle->is_canceling()) {
      result->status = false;
      goal_handle->canceled(result);
      set_to_inactive();
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return;
    }
    if (state == State::moving_to_goal) {
      dx = desired_pos_.translation()(0) - current_pos.translation()(0);
      dy = desired_pos_.translation()(1) - current_pos.translation()(1);
      ds = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
      theta_des = std::atan2(dy, dx);
      dtheta_forward = std::atan2(
          std::sin(theta_des -
                   Eigen::Rotation2Dd(current_pos.linear()).angle()),
          std::cos(theta_des -
                   Eigen::Rotation2Dd(current_pos.linear()).angle()));
      dtheta_backward = std::atan2(
          std::sin(theta_des + kPi -
                   Eigen::Rotation2Dd(current_pos.linear()).angle()),
          std::cos(theta_des + kPi -
                   Eigen::Rotation2Dd(current_pos.linear()).angle()));
      if (ds > kGoalPosTol) {
        bool move_forward{std::abs(dtheta_forward) <=
                          std::abs(dtheta_backward)};
        auto dtheta{move_forward ? dtheta_forward : dtheta_backward};
        if (std::abs(dtheta) <= kGoalThetaTol) {
          twist.linear.x = move_forward ? kLinearSpeed : -kLinearSpeed;
          twist.angular.z = 0.0;
        } else {
          twist.linear.x = 0.0;
          twist.angular.z = dtheta / 2;
        }
      } else {
        state_expected = State::moving_to_goal;
        if (state_.compare_exchange_strong(state_expected,
                                           State::rotating_to_goal)) {
          state = State::rotating_to_goal;
        }
      }
    }
    if (state == State::rotating_to_goal) {
      theta_des = Eigen::Rotation2Dd(desired_pos_.linear()).angle();
      dtheta_forward = std::atan2(
          std::sin(theta_des -
                   Eigen::Rotation2Dd(current_pos.linear()).angle()),
          std::cos(theta_des -
                   Eigen::Rotation2Dd(current_pos.linear()).angle()));
      if (std::abs(dtheta_forward) <= kGoalThetaTol) {
        result->status = true;
        goal_handle->succeed(result);
        set_to_inactive();
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        return;
      }
      twist.linear.x = 0.0;
      twist.angular.z = dtheta_forward / 2;
    }
    publisher_->publish(twist);
    loop_rate.sleep();
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<GoToPose>());

  rclcpp::shutdown();
}