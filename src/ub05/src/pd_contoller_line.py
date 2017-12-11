#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry


def odomCallback(odom):
    global init, offset_x, offset_y, time, forceZero, xvalues, yvalues, lastError, lastTime

    position = odom.pose.pose.position
    # simulate start at (x, y) = (0, 0)
    y = position.y - offset_y


    if not init:
        init = True
        offset_y = position.y
        offset_x = position.x
    

    if abs(position.x - offset_x) < 4.0:
        pub.publish(forward)
    else:
        pub.publish(stay)
        odom_sub.unregister()
        return


    #calculating error
    error = -(desired_y - y)
    deltaTime = (rospy.Time.now() - lastTime).to_sec()
    derivate = (error - lastError) / deltaTime

    lastError = error
    lastTime = rospy.Time.now()

    Kp = 500.0
    Kd = 350.0
    calibratedZero = 115

    #PD controller
    u = Kp * error + Kd * derivate + calibratedZero

    #Max Min constraint
    u = max(0, u)
    u = min(179, u)


    spub.publish(u)
    #publish y for plotting with rqt_plot
    ypub.publish(y)
    print(u)


#get car yaw
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


lastError = 0
lastTime = rospy.Time.now()

global odom_sub
odom_sub = rospy.Subscriber("/odom", Odometry, odomCallback, queue_size=1)
rospy.Subscriber("/model_car/yaw", Float32, yawCallback, queue_size=1)

desired_y = rospy.get_param("~y")
offset_y = 0
offset_x = 0

car_yaw = 0

rospy.spin()
