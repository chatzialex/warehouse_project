from copy import deepcopy
import argparse
import types
import time

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from path_planner_server.action import GoToPose

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32
from std_msgs.msg import Empty
from std_msgs.msg import String
from std_srvs.srv import Trigger


class MoveShelfToShipNode(Node):
    def __init__(self, node_name, real_robot):
        super().__init__(node_name)

        if real_robot:
            self.elevator_down_publisher_ = self.create_publisher(
                String,
                'elevator_down',
                QoSProfile(
                    reliability=QoSReliabilityPolicy.RELIABLE,
                    depth=1
                )
            )
        else:
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

        self._action_client = ActionClient(self, GoToPose, 'go_to_pose')
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            print('Action not available, waiting again...')

    def send_goal(self, goal_pose):
        goal_msg = GoToPose.Goal()
        goal_msg.goal_pos = goal_pose
        self.send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedbackCallback)
        self.send_goal_future.add_done_callback(self.goalResponseCallback)

    def goalResponseCallback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.getResultCallback)

    def getResultCallback(self, future):
        result = future.result().result
        if result.status:
            self.get_logger().info('Action succeded.')
        else:
            self.get_logger().warning('Action failed.')

    def feedbackCallback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            'Received current pose: x={0}, y={1}, theta={2}'.format(
                feedback.current_pos.x, feedback.current_pos.y, feedback.current_pos.theta
            )
        )


class MoveShelfToShip():
    footprints = {
        'robot': [[0.25, 0.25], [0.25, -0.25], [-0.25, -0.25], [-0.25, 0.25]],
        'robot_with_box': [[0.50, 0.40], [0.50, -0.40], [-0.50, -0.40], [-0.50, 0.40]]
    }

    def __init__(self, real_robot):
        if (real_robot):
            self.goals = {
                'initial_position': {'position': {'x': 0.216, 'y': 0.0}, 'orientation': {'z': 0.285, 'w': 0.959}},
                'loading position': {'position': {'x': 4.463, 'y': 0.970}, 'orientation': {'z': -0.614, 'w': 0.789}},
                'shipping_position': {'position': {'x': 1.811, 'y': 1.534}, 'orientation': {'z': 0.737, 'w': 0.676}}
            }

            self.goals_custom = {
                'post_loading_position': {'position': {'x': 4.0, 'y': 0.4}, 'orientation': {'theta': 3.14}}
            }
        else:
            self.goals = {
                'initial_position': {'position': {'x': 0.0, 'y': 0.0}, 'orientation': {'z': 0.0, 'w': 1.0}},
                'loading position': {'position': {'x': 5.5, 'y': 0.0}, 'orientation': {'z': -0.707, 'w': 0.707}},
                'shipping_position': {'position': {'x': 2.5, 'y': 1.3}, 'orientation': {'z': 0.707, 'w': 0.707}}
            }

            self.goals_custom = {
                'post_loading_position': {'position': {'x': 5.0, 'y': -0.5}, 'orientation': {'theta': 3.14}}
            }

        self.node_ = MoveShelfToShipNode(node_name='move_shelf_to_ship_node', real_robot=real_robot)

        def elevatorDown(self):
            if real_robot:
                self.node_.elevator_down_publisher_.publish(String())
            else:
                self.node_.elevator_down_publisher_.publish(Empty())
            rclpy.spin_once(self.node_)
            time.sleep(3)
            return True
        self.elevatorDown = types.MethodType(elevatorDown, self)

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
        success = (self.elevatorDown()
                   and self.goToPose('loading position')
                   and self.attachToShelf()
                   and self.updateFootprint('robot_with_box')
                   and self.goToPoseCustom('post_loading_position')
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

    def goToPoseCustom(self, pose_name):
        goal_pose = Pose2D()
        goal_pose.x = self.goals_custom[pose_name]['position']['x']
        goal_pose.y = self.goals_custom[pose_name]['position']['y']
        goal_pose.theta = self.goals_custom[pose_name]['orientation']['theta']

        self.node_.send_goal(goal_pose=goal_pose)

        rclpy.spin_until_future_complete(self.node_, self.node_.send_goal_future)
        if (not self.node_.send_goal_future.result().accepted):
            return False

        rclpy.spin_once(self.node_)
        rclpy.spin_once(self.node_)
        rclpy.spin_until_future_complete(self.node_, self.node_.get_result_future)
        rclpy.spin_once(self.node_)
        rclpy.spin_once(self.node_)
        return self.node_.get_result_future.result().result.status



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


def main():
    parser=argparse.ArgumentParser(description="Move shelf to ship.")
    parser.add_argument('--real_robot', action='store_true', help='Use the goals specified for the real robot.')
    args=parser.parse_args()

    rclpy.init()

    move_shelf_to_ship = MoveShelfToShip(real_robot=args.real_robot)
    move_shelf_to_ship.perform_motion()

    rclpy.shutdown()

if __name__ == '__main__':
    main()