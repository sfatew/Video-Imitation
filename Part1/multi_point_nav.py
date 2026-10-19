#!/usr/bin/env python3
"""
Multi-point navigation for ROSOrin Pro (Gazebo + Nav2).

Sends the robot through a list of waypoints using Nav2's FollowWaypoints.

Prerequisites:
  1. Gazebo simulation + Nav2 navigation are running with your saved map.
  2. Initial pose has been set in RViz ("2D Pose Estimate").

Run:
  python3 multi_point_nav.py

Get waypoint coordinates:
  In RViz, use the "Publish Point" tool and click on the map while running:
    ros2 topic echo /clicked_point
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# (x, y, yaw in degrees) in the map frame -- replace with points from YOUR map
WAYPOINTS = [
    (1.0, 0.0, 0.0),
    (1.0, 1.0, 90.0),
    (0.0, 1.0, 180.0),
    (0.0, 0.0, 0.0),
]


def make_pose(nav: BasicNavigator, x: float, y: float, yaw_deg: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    yaw = math.radians(yaw_deg)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def main() -> None:
    rclpy.init()
    nav = BasicNavigator()

    print("Waiting for Nav2 to become active...")
    # If Hiwonder's navigation does not use AMCL, change the localizer argument
    nav.waitUntilNav2Active(localizer="amcl")

    poses = [make_pose(nav, *wp) for wp in WAYPOINTS]
    print(f"Following {len(poses)} waypoints")
    nav.followWaypoints(poses)

    last_reported = -1
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        if feedback and feedback.current_waypoint != last_reported:
            last_reported = feedback.current_waypoint
            x, y, yaw = WAYPOINTS[last_reported]
            print(f"-> Waypoint {last_reported + 1}/{len(poses)}: ({x:.2f}, {y:.2f}, {yaw:.0f} deg)")

    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        print("All waypoints reached.")
    elif result == TaskResult.CANCELED:
        print("Navigation was canceled.")
    else:
        print("Navigation failed (some waypoints may have been missed).")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
