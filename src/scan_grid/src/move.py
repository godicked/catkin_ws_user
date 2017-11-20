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

def moveCar():
    global mesureMode, time, steering

    #print("steering : " + str(steering))
    #spub.publish(steering)

    rate = rospy.Rate(1)
    rate.sleep()
    print("move car")

    pub.publish(backward)
    rate.sleep()
    pub.publish(0)
    time = rospy.Time.now()

    print("mesure d02")
    mesureMode = 2


def computePoint(range, angle):
    x = range * math.sin(angle) #+ trans[1] 
    y = range * math.cos(angle) #+ trans[0]
    return (x, y)

def scanCallback(scan_msg):
    global mesureMode, d01, d02, time, dl2, dr2

    if mesureMode == 0:
        return

    angle = scan_msg.angle_min
    inc = scan_msg.angle_increment

    points = []

    for r in scan_msg.ranges:
        (x, y) = computePoint(r, angle)
        
        angle += inc

        if not (angle < -3 * math.pi / 4.0 or angle > 3 * math.pi / 4.0):
            continue
        if r > scan_msg.range_max or r < scan_msg.range_min:
            continue

        points.append([x, y])
    
    if mesureMode == 1:
        mesureMode = 0
        d01 = distanceToWall(points)
        print("d01 : " + str(d01))
        moveCar()
        return

    if mesureMode == 2:
        if rospy.Time.now() - time < rospy.Duration(2.0):
            return
        mesureMode = 0
        d02 = distanceToWall(points)
        print("d02 : " + str(d02))
        print("")

        (xl, yl) = computePoint(scan_msg.ranges[20], scan_msg.angle_min + scan_msg.angle_increment * 20)
        (xr, yr) = computePoint(scan_msg.ranges[340], scan_msg.angle_min + scan_msg.angle_increment * 340)

        dl2 = distance([xl, yl])
        dr2 = distance([xr, yr])
        print("dl2 : " + str(dl2))
        print("dr2 : " + str(dr2))
        print("")

        thetaL2 = math.acos(d02 / dl2)
        thetaR2 = math.acos(d02 / dr2)
        print("thetaL2 : " + str(thetaL2))
        print("thetaR2 : " + str(thetaR2))
        print("")

        theta02L = thetaL2 - 20 * scan_msg.angle_increment
        theta02R = thetaR2 + 20 * scan_msg.angle_increment
        print("theta02L : " + str(theta02L))
        print("theta02R : " + str(theta02R))
        print("")

        RL = (d02 - d01) / math.sin(theta02L)
        RR = (d02 - d01) / math.sin(theta02R)

        print("RL : " + str(RL))
        print("RR : " + str(RR))
        print("")

        if(abs(RL) >= 0.26):
            turning_angleL = math.asin(0.26 / RL)
            print("turning angle L : " + str(turning_angleL) + " ; " + str(math.degrees(turning_angleL)))
        if(abs(RR) >= 0.26):
            turning_angleR = math.asin(0.26 / RR)
            print("turning angle R : " + str(turning_angleR) + " ; " + str(math.degrees(turning_angleR)))

        quit()

# --- main ---
rospy.init_node("move")

backward = Int16(200)
forward = Int16(-150)

mesureMode = 1

time = 0

d01 = 0
d02 = 0
dl2 = 0
dr2 = 0

pub = rospy.Publisher("/manual_control/speed", Int16, queue_size=1)
spub = rospy.Publisher("/manual_control/steering", Int16, queue_size=1)

steering = rospy.get_param("~steering", 80)

rospy.Subscriber("scan", LaserScan, scanCallback, queue_size=1)
rospy.spin()

