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
# Function to create new PoseStamped instances based on given data:
def new_pose_stamped(header_frame_id:str, x:float, y:float, z:float, roll:float, pitch:float, yaw:float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id =  header_frame_id
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = float(z)
    # NOTE: If position.x is not assigned to a floating point value (even if it is an integer), then we get the assertion error:
    # AssertionError: The 'x' field must be of type 'float'
    # The same applies for position.y and position.z
    qx, qy, qz, qw = tft.quaternion_from_euler(roll, pitch, yaw)
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose

#================================================
# Initialisation:
rclpy.init()
nav = BasicNavigator()

#================================================
# Set initial pose:
initial_pose = new_pose_stamped("map", 0, 0, 0, 0, 0, 0)
nav.setInitialPose(initial_pose)

#================================================
# Wait for Nav2 to be active before sending the initial pose on the appropriate topic:
nav.waitUntilNav2Active()

#================================================
# Set waypoints:
waypoints = [
    new_pose_stamped("map", -9, 2, 0, 0, 0, 1.57),
    new_pose_stamped("map", -5, 5, 0, 0, 0, -1.57),
    new_pose_stamped("map", 0, 0, 0, 0, 0, 0)
]

for waypoint in waypoints:
    nav.goToPose(waypoint)

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
        # print(feedback)

    # Get the pending action result message:
    print(nav.getResult())

#================================================
# Shutdown:
rclpy.shutdown()
