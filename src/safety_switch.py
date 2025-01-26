#!/usr/bin/env python
# Using joystick input, LiDAR data, and velocity commands to determine safe and autonomous robot movement.
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from sensor_msgs.msg import LaserScan
import math

# Global variables
enable = False # Indicates whether the joystick button is pressed.
can_go_forward = False  # Indicates whether the robot can safely move forward.
can_go_backward = False  # Indicates whether the robot can safely move backward.

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) # Publishes Twist messages to control robot velocity.

def callback_listener(data):
    """
    Callback function for the velocity topic.
    It checks if the robot is allowed to move based on joystick input (enable) and obstacle status (can_go_forward and can_go_backward).
    """
    if (data.linear.x > 0) :  # Check if the robot is commanded to move forward.
        if not enable or not can_go_forward : 
    	    empty_twist = Twist() 
    	    pub.publish(empty_twist) # Publish an empty Twist message to stop the robot.
        else: 
    	    pub.publish(data)
    else:
        if not enable or not can_go_backward:
            empty_twist = Twist()
            pub.publish(empty_twist)
        else:
            pub.publish(data)

def callback_joy(data):
    """
    Callback function for the joystick topic.
    Updates the 'enable' variable based on joystick button status.
    """
    global enable
    if (data.axes[5] < 0.95 and not (data.axes[2]>-0.001 and data.axes[2]<0.001)):
    	enable = True
    else: 
    	enable = False

def callback_lidar(data):
    """
    Callback function for the LiDAR topic.
    Updates the 'can_go_forward' and 'can_go_backward' variables based on obstacle detection.
    """
    global can_go_forward
    global can_go_backward

    # Thresholds for defining the collision distance in (meters) based on robot's velocity
    x_ThreshFront_forward = 0.75
    x_ThreshBack_forward = 0
    x_ThreshFront_backward = 0
    x_ThreshBack_backward = -0.55
    y_ThreshLeft  = 0.40
    y_ThreshRight = -0.40

    # Temporary variables to determine if the path is clear.
    can_go_forward_temp = True
    can_go_backward_temp = True

    # Loop through all LiDAR scan data points.
    for i in range(len(data.ranges)):
        theta = data.angle_min + (i*data.angle_increment)
        x = math.cos(theta) * data.ranges[i]
        y = math.sin(theta) * data.ranges[i]
        
        isInDangerZoneX_forward = (x<x_ThreshFront_forward and x>x_ThreshBack_forward)
        isInDangerZoneX_backward = (x<x_ThreshFront_backward and x>x_ThreshBack_backward)
        isInDangerZoneY = (y<y_ThreshLeft and y>y_ThreshRight)
        
        if isInDangerZoneX_forward and isInDangerZoneY:
            can_go_forward_temp = False
        if isInDangerZoneX_backward and isInDangerZoneY:
            can_go_backward_temp = False
            
    can_go_forward = can_go_forward_temp
    can_go_backward = can_go_backward_temp

def safety_switch():
    """
    Initializes the ROS node and subscribes to relevant topics.
    Ensures the robot's movement is controlled by joystick input and obstacle detection.
    """
    rospy.init_node('safety_switch', anonymous=True)
    rospy.Subscriber("/controller/cmd_vel_smooth", Twist, callback_listener)
    rospy.Subscriber("/bluetooth_teleop/joy", Joy, callback_joy)
    rospy.Subscriber("/scan", LaserScan, callback_lidar)
    # Keeps the script running until the node is manually stopped.
    rospy.spin()

# Run the safety switch node when the script is executed.
if __name__ == '__main__':
    safety_switch()
