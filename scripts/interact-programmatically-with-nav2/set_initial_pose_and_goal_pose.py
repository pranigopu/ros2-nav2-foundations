#!/usr/bin/env python3

# NOTE: The above indicates a CLI command, indicating the following code must be run as an executable

# TEST SCRIPT FOR NAV2 SIMPLE API COMMANDER

# Import rclpy, the canonical Python API for interacting with ROS 2:
import rclpy # NOTE: rcl => "ROS Client Library"
# Import a class that defines a basic interface to Nav2 robot navigation:
from nav2_simple_commander.robot_navigator import BasicNavigator
# Import the pose with time stamp geometry message to enable specifying poses:
from geometry_msgs.msg import PoseStamped
import tf_transformations as tft

#================================================
# Initialisation:
rclpy.init()
nav = BasicNavigator()

#================================================
# Set initial pose:
initial_pose = PoseStamped()
initial_pose.header.frame_id =  "map"
initial_pose.header.stamp = nav.get_clock().now().to_msg()
initial_pose.pose.position.x = 0.0
initial_pose.pose.position.y = 0.0
initial_pose.pose.position.z = 0.0

qx, qy, qz, qw = tft.quaternion_from_euler(0.0, 0.0, 0.0)
initial_pose.pose.orientation.x = qx
initial_pose.pose.orientation.y = qy
initial_pose.pose.orientation.z = qz
initial_pose.pose.orientation.w = qw

nav.setInitialPose(initial_pose)

#================================================
# Wait for Nav2 to be active before sending the initial pose on the appropriate topic:
nav.waitUntilNav2Active()

#================================================
# Set goal pose:
goal_pose = PoseStamped()
goal_pose.header.frame_id =  "map"
goal_pose.header.stamp = nav.get_clock().now().to_msg()
goal_pose.pose.position.x = -9.0
goal_pose.pose.position.y = 2.0
goal_pose.pose.position.z = 0.0

qx, qy, qz, qw = tft.quaternion_from_euler(0.0, 0.0, -1.57)
# NOTE: 1.57 radians is approximately 90 degrees
goal_pose.pose.orientation.x = qx
goal_pose.pose.orientation.y = qy
goal_pose.pose.orientation.z = qz
goal_pose.pose.orientation.w = qw

nav.goToPose(goal_pose)

'''
KEY POINTS:
- The above method will only set the goal pose
- It will not wait until navigation to this goal is complete
- Hence, we use the subsequent loop
'''

#================================================
# Wait for Nav2 to complete task
# NOTE: # Task Complete => Successful/Aborted/Failed

while not nav.isTaskComplete():
    feedback = nav.getFeedback()
    # NOTE: The above gets PoseStamped message of current pose of navigating agent
    print(feedback)

# Get the pending action result message:
print(nav.getResult())

#================================================
# Shutdown

rclpy.shutdown()
