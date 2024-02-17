from copy import deepcopy

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty


class MoveShelfToShip(Node):
    goals = [
        {"position": {"x": 0.0, "y": 0.0}, "orientation": {"z": 0.0, "w": 1.0}},  # initial position
        {"position": {"x": 5.5, "y": 0.0}, "orientation": {"z": -0.707, "w": 0.707}},  # loading position
        {"position": {"x": 2.5, "y": 1.3}, "orientation": {"z": 0.707, "w": 0.707}}  # shipping position
    ]

    def __init__(self):
        super().__init__('move_shelf_to_ship')

        self.publisher_ = self.create_publisher(Empty, 'elevator_down', 1)
        self.navigator_ = BasicNavigator()

        self.route = []
        waypoint = PoseStamped()
        waypoint.header.frame_id = 'map'
        for pt in self.goals:
            waypoint.pose.position.x = pt["position"]["x"]
            waypoint.pose.position.y = pt["position"]["y"]
            waypoint.pose.orientation.z = pt["orientation"]["z"]
            waypoint.pose.orientation.w = pt["orientation"]["w"]
            self.route.append(deepcopy(waypoint))

        self.route[0].header.stamp = self.navigator_.get_clock().now().to_msg()
        self.navigator_.setInitialPose(self.route[0])
        self.navigator_.waitUntilNav2Active()

        self.timer = self.create_timer(0.2, self.timer_cb)

    def timer_cb(self):
        self.timer.cancel()

        for pt in self.route[1:]:
            pt.header.stamp = self.navigator_.get_clock().now().to_msg()
        self.navigator_.followWaypoints(self.route[1:])

        # Do something during your route (e.x. AI to analyze stock information or upload to the cloud)
        # Print the current waypoint ID for the demonstration
        i = 0
        while not self.navigator_.isTaskComplete():
            i = i + 1
            feedback = self.navigator_.getFeedback()
            if feedback and i % 5 == 0:
                self.get_logger().info(
                    f'Executing current waypoint {feedback.current_waypoint + 1}/{len(self.route)-1}'
                )

        result = self.navigator_.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Route completed! Returning to start...')
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Route cancelled. Returning to start...')
        elif result == TaskResult.FAILED:
            self.get_logger().warn('Route failed! Returning to start...')

        # go back to start
        self.route[0].header.stamp = self.navigator_.get_clock().now().to_msg()
        self.navigator_.goToPose(self.route[0])
        while not self.navigator_.isTaskComplete():
            pass

        exit(0)

def main():
    rclpy.init()
    node = MoveShelfToShip()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()