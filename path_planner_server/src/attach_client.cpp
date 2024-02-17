#include "path_planner_server/attach_client.hpp"
#include "rclcpp/utilities.hpp"
#include <memory>

namespace path_planner_server {

AttachClient::AttachClient(const std::string &node_name,
                           const rclcpp::NodeOptions &options)
    : Node{node_name, options},
      client_{this->create_client<path_planner_server::srv::GoToLoading>(
          kServiceName)},
      timer_{this->create_wall_timer(kTimerPeriod,
                                     [this] { return timer_cb(); })} {}

void AttachClient::timer_cb() {
  while (!client_->wait_for_service(kServiceWaitTime)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
          this->get_logger(),
          "Client interrupted while waiting for service. Terminating...");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Service Unavailable.");
    return;
  }

  timer_->cancel();

  auto request{
      std::make_shared<path_planner_server::srv::GoToLoading::Request>()};
  request->attach_to_shelf = true;

  auto response_received_callback =
      [this](rclcpp::Client<path_planner_server::srv::GoToLoading>::SharedFuture
                 future) {
        auto status = future.wait_for(1s);
        if (status == std::future_status::ready) {
          if (future.get()->complete) {
            RCLCPP_INFO(this->get_logger(), "%s succeeded.", kServiceName);
          } else {
            RCLCPP_INFO(this->get_logger(), "%s failed.", kServiceName);
          }
        } else {
          RCLCPP_INFO(this->get_logger(), "%s in progress...", kServiceName);
        }
      };
  auto future_result =
      client_->async_send_request(request, response_received_callback);
}

} // namespace path_planner_server

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node{std::make_shared<path_planner_server::AttachClient>()};
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
}