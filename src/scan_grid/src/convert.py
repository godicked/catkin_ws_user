#!/usr/bin/env python

# --- imports ---
import rospy


def toInterval(A, B, a, b, val):
    return (val - A)*(b-a)/(B-A) + a


# --- main ---
rospy.init_node("convert")


angle = rospy.get_param("~angle", 80)

max_angle = 0.404
# for positive values, car left
pmax = 90 # min is 0 approx at value 90
pmin = 0  # max is 0.404 rad

# for negative values, car right
nmin = 179 # min is -0.404 rad at approx value 179
nmax = 90 # max is -0 rad at 90

print(angle)

if angle >= 0.0:
    value = toInterval(0, max_angle, pmin, pmax, max_angle - angle)
if angle < 0.0:
    value = toInterval( -max_angle, 0, nmin, nmax, angle)

print "steering value is ", value

