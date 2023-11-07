#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time

max_accel = 1
max_Dv_acc = 0.1
max_Dv_dec = 0.5
# max_decel = 2
cur_vel = 0
pub = rospy.Publisher('/controller/cmd_vel_smooth', Twist, queue_size=10)
last_time = 0

def callback_current_vel(data):
    global cur_vel
    cur_vel = data.twist.twist.linear.x
    
def callback_smooth(data):
    global cur_vel
    global last_time
    tar_vel = data.linear.x
    cur_time = rospy.get_rostime()
    if(last_time == 0):
        last_time = cur_time # time.time()
        return
    # dt = (cur_time-last_time).to_sec()
    # dt = min(dt, 0.2)
    last_time = cur_time
    data.linear.x = smooth_vel_assym(cur_vel, tar_vel) 
    print(cur_time.to_sec(), ", ", cur_vel, ", ", tar_vel, ", ", data.linear.x)
    pub.publish(data)

def smooth_vel(cur_vel, tar_vel, dt):
    global max_accel
    max_delta = max_accel * dt
    if (abs(cur_vel - tar_vel)<0.3):
        return tar_vel
    elif (tar_vel > cur_vel):
        return min(cur_vel + max_delta, tar_vel)
    else:
        return max(cur_vel - max_delta, tar_vel)


 
def smooth_vel_assym(cur_vel, tar_vel):
    global max_Dv_acc, max_Dv_dec
    if (abs(cur_vel - tar_vel)<0.3):
        return tar_vel
    # sign of cur vel
    signcv = 1
    if cur_vel < 0:
        signcv = -1
    cur_vel = cur_vel * signcv
    tar_vel = tar_vel * signcv
    # decide acc or dec
    max_delta = max_Dv_acc
    if tar_vel < cur_vel:
        # deacclelration
    	max_delta = max_Dv_dec
    # smooth
    cur_vel = cur_vel * signcv
    tar_vel = tar_vel * signcv
    if (tar_vel > cur_vel):
        # put the sign back
        return min(cur_vel + max_delta, tar_vel)
    else:
        return max(cur_vel - max_delta, tar_vel)

def smooth():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('smooth', anonymous=True)
    rospy.Subscriber("/jackal_velocity_controller/odom", Odometry, callback_current_vel)
    rospy.Subscriber("/controller/cmd_vel_inverted", Twist, callback_smooth)
    # Testing talker nodes
    #rospy.Subscriber("/fake_cur_vel", Odometry, callback_current_vel)
    #rospy.Subscriber("/fake_tar_vel", Twist, callback_smooth)
    rospy.spin()

if __name__ == '__main__':
    smooth()
