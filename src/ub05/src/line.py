#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
#import matplotlib.pyplot as plt

# from sensor_msgs.msg import LaserScan



def steer(desired_heading):
    global car_yaw, lastDiff

    Kp=1.0
    Kd=0.5
    calibratedZero=115

    current_heading = car_yaw
    print "desired heading", desired_heading

    diff = desired_heading - current_heading

    if diff > 180:
        diff -= 360
    if diff < -180:
        diff += 360

    print "heading diff", diff
    if lastDiff == float("Inf"):
        lastDiff = diff
    
    u = Kp * diff + Kd * (diff - lastDiff) + calibratedZero
    lastDiff = diff

    u = max(0, u)
    u = min(179, u)

    print "u", u

    return u

def odomCallback(odom):
    global init, offset_x, offset_y, time, forceZero, xvalues, yvalues

    position = odom.pose.pose.position
    # simulate start at (x, y) = (0, 0)
    y = position.y - offset_y
    

    # if rospy.Time.now() - time < rospy.Duration(0.05):
    #     return
    # time = rospy.Time.now()


    if not init:
        init = True
        offset_y = position.y
        offset_x = position.x
    

    if abs(position.x - offset_x) < 2.0:
        pub.publish(forward)
    else:
        pub.publish(stay)
        odom_sub.unregister()
        return

    diff_y = desired_y - y
    if abs(diff_y) < 0.05:
        pub.publish(stay)
        odom_sub.unregister()
        return


    
    print "y diff", diff_y
    desired_heading = 90 - math.atan(0.3 / diff_y)
    if abs(diff_y) < 0.05:
        desired_heading = 0.0

    desired_heading = min(desired_heading, 25)
    desired_heading = max(desired_heading, -25)


    u = steer(desired_heading)
    spub.publish(u)
    ypub.publish(y)
    #print(u)
    #print(y)


def yawCallback(yaw):
    global car_yaw
    car_yaw = yaw.data

# ---main---
rospy.init_node("line")
init=False
backward = Int16(300)
forward = Int16(-150)
stay = Int16(0)

time=rospy.Time.now()

pub = rospy.Publisher("/manual_control/speed", Int16, queue_size=1)
spub = rospy.Publisher("/manual_control/steering", Int16, queue_size=1)

ypub = rospy.Publisher("/ub05/y", Float32, queue_size=100)

forceZero = False

lastDiff = float("Inf")



# rospy.Subscriber("scan", LaserScan, scanCallback, queue_size=1)
global odom_sub
odom_sub = rospy.Subscriber("/odom", Odometry, odomCallback, queue_size=1)
rospy.Subscriber("/model_car/yaw", Float32, yawCallback, queue_size=1)

desired_y = rospy.get_param("~y")
offset_y = 0
offset_x = 0

car_yaw = 0

rospy.spin()
