from copy import deepcopy

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32
from std_msgs.msg import Empty
from std_srvs.srv import Trigger


class MoveShelfToShipNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        self.elevator_down_publisher_ = self.create_publisher(
            Empty,
            'elevator_down',
            QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                depth=1
            )
        )

        self.local_footprint_publisher_ = self.create_publisher(
            Polygon,
            'local_costmap/footprint',
             QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                depth=1
            )
        )

        self.global_footprint_publisher_ = self.create_publisher(
            Polygon,
            'global_costmap/footprint',
             QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                depth=1
            )
        )

        self.client_ = self.create_client(Trigger, 'approach_shelf')
        while not self.client_.wait_for_service(timeout_sec=1.0):
            print('Service not available, waiting again...')

class MoveShelfToShip():
    goals = {
        'initial_position': {'position': {'x': 0.0, 'y': 0.0}, 'orientation': {'z': 0.0, 'w': 1.0}},
        'loading position': {'position': {'x': 5.5, 'y': 0.0}, 'orientation': {'z': -0.707, 'w': 0.707}},
        'shipping_position': {'position': {'x': 2.5, 'y': 1.3}, 'orientation': {'z': 0.707, 'w': 0.707}}
    }

    footprints = {
        'robot': [[0.25, 0.25], [0.25, -0.25], [-0.25, -0.25], [-0.25, 0.25]],
        'robot_with_box': [[0.50, 0.50], [0.50, -0.50], [-0.50, -0.50], [-0.50, 0.50]]
    }

    def __init__(self):
        self.node_ = MoveShelfToShipNode('move_shelf_to_ship_node')

        self.navigator_ = BasicNavigator()
        self.route_ = {}
        waypoint = PoseStamped()
        waypoint.header.frame_id = 'map'
        for name, pt in self.goals.items():
            waypoint.pose.position.x = pt['position']['x']
            waypoint.pose.position.y = pt['position']['y']
            waypoint.pose.orientation.z = pt['orientation']['z']
            waypoint.pose.orientation.w = pt['orientation']['w']
            self.route_[name] = deepcopy(waypoint)
        self.route_['initial_position'].header.stamp = self.navigator_.get_clock().now().to_msg()
        self.navigator_.setInitialPose(self.route_['initial_position'])
        self.navigator_.waitUntilNav2Active()

    def perform_motion(self):
        success = (self.goToPose('loading position')
                   and self.attachToShelf()
                   and self.updateFootprint('robot_with_box')
                   and self.goToPose('shipping_position')
                   and self.elevatorDown()
                   and self.updateFootprint('robot')
        )

        self.goToPose("initial_position")


    def goToPose(self, pose_name):
        self.route_[pose_name].header.stamp = self.navigator_.get_clock().now().to_msg()
        self.navigator_.goToPose(self.route_[pose_name])

        i = 0
        while not self.navigator_.isTaskComplete():
            i = i + 1
            feedback = self.navigator_.getFeedback()
            if feedback and i % 20 == 0:
                print(f'Estimated time of arrival at {pose_name}: '
                    f'{Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9} '
                    'seconds')

        result = self.navigator_.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f'Arrived at {pose_name}.')
            return True
        elif result == TaskResult.CANCELED:
            print(f'Route to {pose_name} was cancelled.')
            return False
        elif result == TaskResult.FAILED:
            print(f'Failed to arrive to {pose_name}.')
            return False

    def attachToShelf(self):
        print('Attempting to attach shelf.')
        request = Trigger.Request()
        future = self.node_.client_.call_async(request)
        rclpy.spin_until_future_complete(self.node_, future)
        if future.result().success:
            print('Successfully attached to shelf.')
        else:
            print('Failed to attach to shelf.')
        return future.result().success

    def updateFootprint(self, footprint_name):
        polygon = Polygon()
        point = Point32()
        for pt in self.footprints[footprint_name]:
            point.x = pt[0]
            point.y = pt[1]
            polygon.points.append(deepcopy(point))

        self.node_.local_footprint_publisher_.publish(polygon)
        self.node_.global_footprint_publisher_.publish(polygon)
        rclpy.spin_once(self.node_)

        return True

    def elevatorDown(self):
        print('Lowering elevator.')
        self.node_.elevator_down_publisher_.publish(Empty())
        rclpy.spin_once(self.node_)
        return True


def main():
    rclpy.init()

    move_shelf_to_ship = MoveShelfToShip()
    move_shelf_to_ship.perform_motion()

    rclpy.shutdown()

if __name__ == '__main__':
    main()