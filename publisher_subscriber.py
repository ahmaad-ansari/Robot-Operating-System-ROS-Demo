#!/usr/bin/env python

# Import the necessary ROS libraries and message types
import rospy
from std_msgs.msg import String

# Define the publisher function
def publisher():
    # Create a Publisher object that publishes messages on the 'chatter' topic of type String
    pub = rospy.Publisher('chatter', String, queue_size=10)
    # Initialize the node with a unique name, and set it to be anonymous (if multiple nodes with the same name are launched, ROS will append numbers to make them unique)
    rospy.init_node('publisher', anonymous=True)
    # Set the rate at which the publisher publishes messages
    rate = rospy.Rate(10) # 10hz
    # Loop until the node is shutdown
    while not rospy.is_shutdown():
        # Create a string message to publish
        hello_str = "Hello ROS, I am the publisher!"
        # Log the message to the console
        rospy.loginfo(hello_str)
        # Publish the message on the 'chatter' topic
        pub.publish(hello_str)
        # Wait for the specified amount of time before publishing the next message
        rate.sleep()

# Define the subscriber function
def subscriber():
    # Initialize the node with a unique name, and set it to be anonymous
    rospy.init_node('subscriber', anonymous=True)
    # Create a Subscriber object that subscribes to the 'chatter' topic of type String, and calls the 'callback' function every time a message is received
    rospy.Subscriber('chatter', String, callback)
    # Spin until the node is shutdown
    rospy.spin()

# Define the callback function that is called when a message is received
def callback(data):
    # Log the message to the console, along with the ID of the node that called the function
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

# Main function that executes when the program is run
if __name__ == '__main__':
    try:
        # Launch the publisher and subscriber nodes
        publisher()
        subscriber()
    except rospy.ROSInterruptException:
        # Catch any exceptions that occur when the node is interrupted
        pass
