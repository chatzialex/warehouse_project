#include "rclcpp/logging.hpp"
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <path_planner_server/action/go_to_pose.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <stdexcept>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

constexpr char kNodeName[]{"GoToPoseServer"};
constexpr char kActionName[]{"go_to_pose"};
constexpr char kOdometryTopicName[]{"/odom"};
constexpr char kVelocityCommandTopicName[]{"cmd_vel"};

class GoToPose : public rclcpp::Node {
public:
  using GoToPoseAction = path_planner_server::action::GoToPose;
  using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPoseAction>;

  GoToPose()
      : Node{kNodeName},
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
                      std::placeholders::_1))} {}

private:
  enum class State : uint8_t {
    inactive,
    starting,
    preempted,
    cancelling,
    moving_to_goal,
    rotating_to_goal,
  };

  static_assert(std::atomic<State>::is_always_lock_free);

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

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_{};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_{};
  rclcpp_action::Server<GoToPoseAction>::SharedPtr action_server_;

  geometry_msgs::msg::Pose2D desired_pos_{};
  geometry_msgs::msg::Pose2D current_pos_{};

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
  current_pos_.x = msg->pose.pose.position.x;
  current_pos_.y = msg->pose.pose.position.y;
  current_pos_.theta = yaw;
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
  desired_pos_ = goal->goal_pos;
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
  auto feedback{std::make_shared<GoToPoseAction::Feedback>()};
  auto result{std::make_shared<GoToPoseAction::Result>()};
  geometry_msgs::msg::Twist twist{};
  rclcpp::Rate loop_rate(kLoopRate);
  double dx{};
  double dy{};
  double ds{};
  double theta_des{};
  double dtheta{};
  decltype(current_pos_) current_pos;

  while (rclcpp::ok()) {
    {
      std::lock_guard<std::mutex> lock{odometry_lock_};
      current_pos = current_pos_;
    }
    feedback->current_pos = current_pos;
    RCLCPP_DEBUG(this->get_logger(), "Publish feedback");
    goal_handle->publish_feedback(feedback);
    auto state{state_.load()};
    if (goal_handle->is_canceling()) {
      state = State::cancelling;
    }
    switch (state) {
    case State::preempted:
      result->status = false;
      goal_handle->abort(result);
      publisher_->publish(geometry_msgs::msg::Twist{});
      RCLCPP_INFO(this->get_logger(), "Goal preempted by new goal.");
      {
        std::lock_guard<std::mutex> lock{preempt_lock_};
        state_.store(State::inactive); // allow new goal
        cv_.notify_one();
      }
      return;
    case State::cancelling:
      result->status = false;
      goal_handle->canceled(result);
      publisher_->publish(geometry_msgs::msg::Twist{});
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      {
        std::lock_guard<std::mutex> lock{preempt_lock_};
        state_.store(State::inactive); // allow new goal
        cv_.notify_one();
      }
      return;
    case State::moving_to_goal:
      dx = desired_pos_.x - current_pos.x;
      dy = desired_pos_.y - current_pos.y;
      ds = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
      theta_des = std::atan2(dy, dx);
      dtheta = std::atan2(std::sin(theta_des - current_pos.theta),
                          std::cos(theta_des - current_pos.theta));
      if (ds > kGoalPosTol) {
        if (std::abs(dtheta) <= kGoalThetaTol) {
          twist.linear.x = kLinearSpeed;
          twist.angular.z = 0.0;
          break;
        } else {
          twist.linear.x = 0.0;
          twist.angular.z = dtheta / 2;
          break;
        }
      }
      state_expected = State::moving_to_goal;
      if (!state_.compare_exchange_strong(state_expected,
                                          State::rotating_to_goal)) {
        break;
      }
      [[fallthrough]];
    case State::rotating_to_goal:
      theta_des = desired_pos_.theta;
      dtheta = std::atan2(std::sin(theta_des - current_pos.theta),
                          std::cos(theta_des - current_pos.theta));
      if (std::abs(dtheta) <= kGoalThetaTol) {
        result->status = true;
        goal_handle->succeed(result);
        publisher_->publish(geometry_msgs::msg::Twist{});
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        {
          std::lock_guard<std::mutex> lock{preempt_lock_};
          state_.store(State::inactive); // allow new goal
          cv_.notify_one();
        }
        return;
      }
      twist.linear.x = 0.0;
      twist.angular.z = dtheta / 2;
      break;
    default:
      throw std::logic_error("We should never be here!");
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