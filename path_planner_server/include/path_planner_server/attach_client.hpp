#ifndef PATH_PLANNER_SERVER__ATTACH_CLIENT_HPP_
#define PATH_PLANNER_SERVER__ATTACH_CLIENT_HPP_

#include "path_planner_server/srv/go_to_loading.hpp"

#include "rclcpp/client.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"

namespace path_planner_server {

using namespace std::chrono_literals;

class AttachClient : public rclcpp::Node {
public:
  explicit AttachClient(
      const std::string &node_name = kNodeName,
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions{});

private:
  constexpr static char kNodeName[]{"attach_client"};
  constexpr static char kServiceName[]{"approach_shelf"};
  constexpr static auto kTimerPeriod{1s};
  constexpr static auto kServiceWaitTime{1s};
  constexpr static auto kFutureWaitTime{1s};

  void timer_cb();

  rclcpp::Client<path_planner_server::srv::GoToLoading>::SharedPtr client_{};
  rclcpp::TimerBase::SharedPtr timer_{};
};

} // namespace path_planner_server

#endif // PATH_PLANNER_SERVER__ATTACH_CLIENT_HPP_