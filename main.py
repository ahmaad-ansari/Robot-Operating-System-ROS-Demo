#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import sys, select, termios, tty

# Define global variables
msg = Twist()
pose = Pose()

# Define keyboard control function
def keyboard_control():
    print("Use arrow keys to move the turtle. Press 'q' to exit.")
    # Set up keyboard input
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    # Start loop
    while not rospy.is_shutdown():
        # Check for keyboard input
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            key = sys.stdin.read(1)
            # Move turtle based on keyboard input
            if key == '\x1b[A':
                msg.linear.x = 2.0
            elif key == '\x1b[B':
                msg.linear.x = -2.0
            elif key == '\x1b[C':
                msg.angular.z = -2.0
            elif key == '\x1b[D':
                msg.angular.z = 2.0
            elif key == 'q':
                break
            else:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        # Publish movement commands
        pub.publish(msg)
        # Sleep for a short time
        rospy.sleep(0.1)
    # Reset keyboard settings
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

# Define pose callback function
def pose_callback(data):
    global pose
    pose = data

# Define main function
if __name__ == '__main__':
    try:
        # Initialize node
        rospy.init_node('turtle_control', anonymous=True)
        # Set up publisher for twist commands
        pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        # Set up subscriber for turtle pose
        rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
        # Call keyboard control function
        keyboard_control()
    except rospy.ROSInterruptException:
        pass
