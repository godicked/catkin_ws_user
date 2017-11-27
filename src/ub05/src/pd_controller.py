#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan

# def scanCallback(scan_msg):
#     print("ScanCallback")

def yawCallback(yaw):
    global time,init, timer
    if init==False:
        init=True
        time = rospy.Time.now()
        print "start heading:", yaw.data


    if rospy.Time.now() - timer < rospy.Duration(0.05):
        return
    timer = rospy.Time.now()

    current_heading=yaw.data
    # print "current_heading: ", current_heading
    # print "desired_heading: ", desired_heading
    
    Kp=2.0
    calibratedZero=115

    diff = desired_heading - current_heading
    if diff > 180:
        diff -= 360
    if diff < -180:
        diff += 360
    
    u = Kp * diff + calibratedZero

    u = max(0, u)
    u = min(179, u)
    
    print ("u: " + str(u))
    spub.publish(u)
    if rospy.Time.now() - time > rospy.Duration(10.0):
        pub.publish(stay)
    else:
        pub.publish(forward)



# ---main---
rospy.init_node("pd_controller")
init=False
backward = Int16(300)
forward = Int16(-200)
stay = Int16(0)
time=rospy.Time.now()

timer = rospy.Time.now()

pub = rospy.Publisher("/manual_control/speed", Int16, queue_size=1)
spub = rospy.Publisher("/manual_control/steering", Int16, queue_size=1)

# rospy.Subscriber("scan", LaserScan, scanCallback, queue_size=1)
rospy.Subscriber("/model_car/yaw", Float32, yawCallback, queue_size=1)

desired_heading = rospy.get_param("~heading")

rospy.spin()
