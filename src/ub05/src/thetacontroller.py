#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
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

def computePoint(range, angle):
    x = range * math.sin(angle) #+ trans[1] 
    y = range * math.cos(angle) #+ trans[0]
    return (x, y)

def scanCallback(scan_msg):
    global init, lastDeltaHeading, lastTime
    points = []

    angle = scan_msg.angle_min
    inc = scan_msg.angle_increment

    # if init==False:
    #     init=True
    #     pub.publish(forward)
    # else: return


    for r in scan_msg.ranges:
        (x, y) = computePoint(r, angle)
        
        angle += inc

        if not (angle < -3 * math.pi / 4.0 or angle > 3 * math.pi / 4.0):
            continue
        if r > scan_msg.range_max or r < scan_msg.range_min:
            continue

        points.append([x, y])
    
    #Finding Theta

    l=scan_msg.ranges[240] #may be inf
    r=scan_msg.ranges[300] #may be inf
    
    rmin = scan_msg.ranges[0]
    for r in scan_msg.ranges:
        if rmin > r:
            rmin = r
    
    d = rmin

    asquared=pow(l,2)+math.pow(r,2) +2*l*r*math.cos(60)
    a=math.sqrt(asquared)
    phi=(math.sin(60)*r)/a
    #d=math.sin(phi)*l
    thetal=math.acos(d/l)
    theta=thetal -math.radians(30)
    s=0.2
    cy=d+ math.sin(theta)*s
    P=0.4
    L=0.5
    thetastar=math.atan((P-cy)/L)
    if l==float("inf") or r==float("inf"):
        return    
    print "l",l
    print "r",r
    print "phi", phi
    print "d",d
    print "thetal",thetal
    print "theta",theta
    print "cy", cy
    print ("thetastar: " + str(thetastar))

    deltaHeading = thetastar - theta
    derivate = (deltaHeading - lastDeltaHeading) / (rospy.Time.now() - lastTime).to_sec()

    lastTime = rospy.Time.now()
    lastDeltaHeading = deltaHeading
    
    Kp = 1.0
    Kd = 1.0
    callibZero = 115

    u = Kp * deltaHeading + Kd * derivate + callibZero

    spub.publish(u)

    # closestDistance=scan_msg.range(0)
    # if closestDistance<1.00:
    #     pub.publish(stay)
    #     print("Car stopped, obstacle in the way")

def odomCallback(odom_msg):
    return None
	

# ---main---
rospy.init_node("pd_controller")
init=False
backward = Int16(300)
forward = Int16(-300)
stay = Int16(0)
time=rospy.Time.now()
pub = rospy.Publisher("/manual_control/speed", Int16, queue_size=1)
spub = rospy.Publisher("/manual_control/steering", Int16, queue_size=1)

lastDeltaHeading = 0
lastTime = rospy.Time.now()

rospy.Subscriber("scan", LaserScan, scanCallback, queue_size=1)
rospy.Subscriber("/odom", Odometry, odomCallback, queue_size=1)
rospy.spin()
