from copy import deepcopy

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

goals = {
    "init_position": {"position": {"x": 0.0, "y": 0.0}, "orientation": {"z": 0.0, "w": 1.0}},
    "loading_position": {"position": {"x": 5.5, "y": 0.0}, "orientation": {"z": -0.707, "w": 0.707}},
    "shipping_position": {"position": {"x": 2.5, "y": 1.3}, "orientation": {"z": 0.707, "w": 0.707}}
}

def main():
    rclpy.init()
    navigator = BasicNavigator()

    # Set the initial pose.
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = goals["init_position"]["position"]["x"]
    initial_pose.pose.position.y = goals["init_position"]["position"]["y"]
    initial_pose.pose.orientation.z = goals["init_position"]["orientation"]["z"]
    initial_pose.pose.orientation.w = goals["init_position"]["orientation"]["w"]
    print("pos.x=" + str(goals["init_position"]["position"]["x"]))
    print("pos.y=" + str(goals["init_position"]["position"]["y"]))
    print("ori.z=" + str(goals["init_position"]["orientation"]["z"]))
    print("ori.w=" + str(goals["init_position"]["orientation"]["w"]))
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to activate fully. 
    navigator.waitUntilNav2Active()

    # Send the route.
    route = []
    route_pose = PoseStamped()
    route_pose.header.frame_id = 'map'
    route_pose.header.stamp = navigator.get_clock().now().to_msg()
    route_pose.pose.orientation.z = 1.0
    route_pose.pose.orientation.w = 0.0
    for pt in [goals["loading_position"], goals["shipping_position"]]:
        route_pose.pose.position.x = pt["position"]["x"]
        route_pose.pose.position.y = pt["position"]["y"]
        route_pose.pose.orientation.z = pt["orientation"]["z"]
        route_pose.pose.orientation.w = pt["orientation"]["w"]
        route.append(deepcopy(route_pose))
    nav_start = navigator.get_clock().now()
    navigator.followWaypoints(route)

    # Do something during your route (e.x. AI to analyze stock information or upload to the cloud)
    # Print the current waypoint ID for the demonstration
    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Executing current waypoint: ' +
                  str(feedback.current_waypoint + 1) + '/' + str(len(route)))

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Route completed! Returning to start...')
    elif result == TaskResult.CANCELED:
        print('Route cancelled. Returning to start...')
    elif result == TaskResult.FAILED:
        print('Route failed! Returning to start...')

    # go back to start
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    navigator.goToPose(initial_pose)
    while not navigator.isTaskComplete():
        pass

    exit(0)

if __name__ == '__main__':
    main()