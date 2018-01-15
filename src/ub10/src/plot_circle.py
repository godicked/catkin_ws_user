#!/usr/bin/env python2
import numpy as np
import rospy

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from scipy.spatial import KDTree
from controller import Controller
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

ctrl = None
sub = None
car_length = 0.26
max_steer = math.pi / 3


def callback(data):
    # Get position from ros message
    x0 = data.pose.pose.position.x
    y0 = data.pose.pose.position.y
    orientation_q = data.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

    # Convert to matrix coordinate
    x, y = ctrl.world_to_matrix(x0, y0)

    # Get force vector from matrix
    x1, y1 = ctrl.matrix[x, y,:]


    # Formula from assignement
    fx = np.cos(yaw)*x1 + np.sin(yaw)*y1
    fy = -np.sin(yaw)*x1 + np.cos(yaw)*y1
    Kp = 4.0
    steering = Kp*np.arctan(fy/(2.5*fx))

    # if(np.abs(steering) > math.pi / 5):
    #     return

    # Compute speed and steering
    # When driving backward we use max steering
    if (fx>0):
        speed = -150 # forward
    else:
        speed = 150  # backward
        if (fy>0):
            steering = -max_steer
        if (fy<0):
            steering = max_steer

    # Limit steering
    if (steering > max_steer):
        steering = max_steer

    if (steering < -max_steer):
        steering = -max_steer

    # print(steering)
    
    # car steering conversion
    # steering = 90 + steering * (180/np.pi)

    global sub
    sub.unregister()

    # r = car_length * np.abs(np.tan((np.pi)/2-steering))

    r = car_length / np.abs(math.tan(steering))

    if (r>10):
        r = 10
    if (steering<0.0):
        r=-r
    xc = x0 - np.sin(yaw) * r
    yc = y0 + np.cos(yaw) * r

    print xc, yc, r

    
    circ = plt.Circle((xc, yc), r, color='r', fill=False)
    plt.axis([-3, 7, -3, 7])
    plt.gcf().gca().add_artist(circ)
    plt.gcf().gca().set_aspect(1, 'datalim') # keep circles as circles
    plt.plot([x0, x0 + math.cos(yaw)], [y0, y0 + math.sin(yaw)], 'b-', [x0, x0 + math.cos(yaw + steering)], [y0, y0 + math.sin(yaw +steering)], 'r-')
    plt.show(block=False)



def main():
    rospy.init_node("plot_circle")

    global ctrl
    ctrl = Controller(False)

    global sub
    sub = rospy.Subscriber("/visual_gps/odom", Odometry, callback, queue_size=100)

    rospy.spin()

if __name__ == "__main__":
    main()
