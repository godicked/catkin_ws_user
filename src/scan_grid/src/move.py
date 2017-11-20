#!/usr/bin/env python

# --- imports ---
import rospy
import math
import tf
from std_msgs.msg import Int16
from sensor_msgs.msg import LaserScan

def distance(point):
    x = point[0]
    y = point[1]
    return math.sqrt(x*x + y*y)

def distanceToWall(points):
    best_dist = distance(points[0])
    best_point = points[0]

    for p in points:
        dist = distance(p)
        if dist < best_dist:
            best_dist = dist
            best_point = p
    
    return best_dist

def scanCallback(scan_msg):

    if mesureMode == 0:
        return

    angle = scan_msg.angle_min
    inc = scan_msg.angle_increment

    points = []

    for r in scan_msg.ranges:
        x = r * math.sin(angle) #+ trans[1] 
        y = r * math.cos(angle) #+ trans[0]
        
        angle += inc

        if r > scan_msg.range_max or r < scan_msg.range_min:
            continue

        points.append([x, y])
    
    
    
        

# --- main ---
rospy.init_node("move")

backward = Int16(150)
forward = Int16(-150)

mesureMode = 0

d01 = 0
d02 = 0
dl2 = 0
dr2 = 0


# pub = rospy.Publisher("/manual_control/speed", Int16, queue_size=100)


# rate = rospy.Rate(1)
# rate.sleep()

# pub.publish(forward)

# rate.sleep()

# pub.publish(0)

rospy.Subscriber("scan", LaserScan, scanCallback, queue_size=100)
rospy.spin()

