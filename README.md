# ROS Publisher and Subscriber Example
This code demonstrates a simple example of a publisher and subscriber in the Robot Operating System (ROS). The `publisher()` function creates a ROS node that publishes messages to the 'chatter' topic. The `subscriber()` function creates a ROS node that subscribes to the 'chatter' topic and calls the `callback()` function every time a message is received.

## Requirements
+ ROS installed on the system
+ `std_msgs` package installed in ROS

## Running the Code
1. Clone the repository to your local machine
2. Navigate to the directory containing the code in a terminal
3. Run the following command to make the code executable:
```bash
chmod +x publisher_subscriber.py
```
4. Run the code with the following command:
```bash
rosrun <package_name> publisher_subscriber.py
```
Make sure to replace `<package_name>` with the name of the ROS package you are using.

## Details
The `publisher()` function creates a ROS node called 'publisher', which publishes messages of type `std_msgs.msg.String` to the 'chatter' topic at a rate of 10 messages per second. The message being published is "Hello ROS, I am the publisher!".

The `subscriber()` function creates a ROS node called 'subscriber', which subscribes to the 'chatter' topic and calls the `callback()` function every time a message is received.

The `callback()` function logs the received message to the console, along with the ID of the node that called the function.

The `publisher()` and `subscriber()` functions are launched in the `try` block of the main function, which is executed when the code is run. If a `rospy.ROSInterruptException` is raised, it is caught by the `except` block.

### [Video Submission](https://drive.google.com/file/d/1qUBXyrE9aYIv1PNQJRhhyQ4ISbpnLh7O/view?usp=share_link)
