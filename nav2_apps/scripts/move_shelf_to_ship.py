from copy import deepcopy

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
from std_srvs.srv import Trigger


class MoveShelfToShip(Node):
    goals = {
        'initial_position': {'position': {'x': 0.0, 'y': 0.0}, 'orientation': {'z': 0.0, 'w': 1.0}},
        'loading position': {'position': {'x': 5.5, 'y': 0.0}, 'orientation': {'z': -0.707, 'w': 0.707}},
        'shipping_position': {'position': {'x': 2.5, 'y': 1.3}, 'orientation': {'z': 0.707, 'w': 0.707}}
    }

    def __init__(self):
        super().__init__('move_shelf_to_ship')

        self.publisher_ = self.create_publisher(
            Empty,
            'elevator_down',
            QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                depth=1
            )
        )

        self.client_ = self.create_client(Trigger, 'approach_shelf')
        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.navigator_ = BasicNavigator()
        self.route = {}
        waypoint = PoseStamped()
        waypoint.header.frame_id = 'map'
        for name, pt in self.goals.items():
            waypoint.pose.position.x = pt['position']['x']
            waypoint.pose.position.y = pt['position']['y']
            waypoint.pose.orientation.z = pt['orientation']['z']
            waypoint.pose.orientation.w = pt['orientation']['w']
            self.route[name] = deepcopy(waypoint)
        self.route['initial_position'].header.stamp = self.navigator_.get_clock().now().to_msg()
        self.navigator_.setInitialPose(self.route['initial_position'])
        self.navigator_.waitUntilNav2Active()

        self.timer = self.create_timer(0.2, self.timerCb)

    def timerCb(self):
        self.timer.cancel()

        success = (self.goToPose('loading position')
                   and self.attachToShelf()
                   and self.goToPose('shipping_position')
        )

        self.goToPose("initial_position")

    def goToPose(self, pose_name):
        self.route[pose_name].header.stamp = self.navigator_.get_clock().now().to_msg()
        self.navigator_.goToPose(self.route[pose_name])

        i = 0
        while not self.navigator_.isTaskComplete():
            i = i + 1
            feedback = self.navigator_.getFeedback()
            if feedback and i % 20 == 0:
                self.get_logger().info(f'Estimated time of arrival at {pose_name}: '
                    f'{Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9} '
                    'seconds')

        result = self.navigator_.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f'Arrived at {pose_name}.')
            return True
        elif result == TaskResult.CANCELED:
            self.get_logger().info(f'Route to {pose_name} was cancelled.')
            return False
        elif result == TaskResult.FAILED:
            self.get_logger().warn(f'Failed to arrive to {pose_name}.')
            return False

    def attachToShelf(self):
        self.get_logger().info('Attempting to attach shelf.')
        future = self.client_.call_async()
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info('Successfully attached to shelf.')
        else:
            self._logger().warn('Failed to attach to shelf.')

def main():
    rclpy.init()
    node = MoveShelfToShip()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()