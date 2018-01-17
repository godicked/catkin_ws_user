#!/usr/bin/env python2

import rospy
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import math


res = 0.1 # 10cm resolution
width = 60
height = 40
max_steer = np.pi / 2
offsetx = 0.3
offsety = 0.3

class Controller:
    def __init__(self, register=True):
        self.matrix = np.load('matrix160cm.npy')

        self.pub_steer = rospy.Publisher("/manual_control/steering", Int16, queue_size=1)
        self.pub_speed = rospy.Publisher("/manual_control/speed", Int16, queue_size=100, latch=True)
        self.pub_marker = rospy.Publisher("/marker/forcefield", Marker, queue_size=10000)

        if register:
            self.sub_pos = rospy.Subscriber("/visual_gps/odom", Odometry, self.callback, queue_size=10)

    def callback(self, data):
        # compute steering
        steer, speed = self.steer(data.pose)
        
        print steer, speed

        # Publish steering
        self.pub_steer.publish(Int16(steer))
        self.pub_speed.publish(Int16(speed))
        
        # Publish force field
        self.publish_forcefield()

    
    def steer(self, pose):
        # Get position from ros message
        wx = pose.pose.position.x
        wy = pose.pose.position.y
        orientation_q = pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

        # Convert to matrix coordinate
        x, y = self.world_to_matrix(wx, wy)

        # Get force vector from matrix
        x1, y1 = self.matrix[x, y,:]


        # Formula from assignement
        fx = np.cos(yaw)*x1 + np.sin(yaw)*y1
        fy = -np.sin(yaw)*x1 + np.cos(yaw)*y1
        Kp = 8.0
        steering = -Kp*np.arctan(fy/(2.5*fx))


        # Compute speed and steering
        # When driving backward we use max steering
        if (fx>0):
            speed = -150 # forward
        else:
            speed = 150  # backward
            if (fy>0):
            	steering = max_steer
            if (fy<0):
            	steering = -max_steer

        # Limit steering
        if (steering > max_steer):
            steering = max_steer

        if (steering < -max_steer):
            steering = -max_steer

        # print(steering)



        
        # car steering conversion
        steering = 90 + steering * (180/np.pi)
        return (steering, speed)
    
    def world_to_matrix(self, x, y):
        # get matrix coordinate

        x -= offsetx
        y -= offsety

        x /= res
        y /= res

        # Edges cases
        if (x < 0):
            x = 0
        if (x > width - 1):
            x = width  - 1

        if (y < 0):
            y = 0
        if (y > height - 1):
            y = height - 1

        return (np.int(x), np.int(y))

    def publish_forcefield(self):
        id = 0

        for x in range(0, width, 4):
            for y in range(0, height, 4):
                x1, y1 = self.matrix[x, y,:]
                angle = math.atan2(y1, x1)
                orientation = quaternion_from_euler(0,0, angle)

                wx = x * res + offsetx
                wy = y * res + offsety

                arrow = self.build_arrow((wx,wy), orientation, id)
                self.pub_marker.publish(arrow)
                id += 1

    def publish_lookahead(self, pos, yaw):
        x, y = pos
        ori = quaternion_from_euler(0, 0, yaw)

        arrow = self.build_arrow((x, y), ori, 9999)

        self.pub_marker.publish(arrow)



    def build_arrow(self, pos, orientation, id):
        
        x, y = pos

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "force"
        marker.id = id
        marker.type = Marker.ARROW
        marker.action = Marker.MODIFY
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 1
        marker.pose.orientation.x = orientation[0]
        marker.pose.orientation.y = orientation[1]
        marker.pose.orientation.z = orientation[2]
        marker.pose.orientation.w = orientation[3]
        marker.scale.x = 0.15
        marker.scale.y = 0.03
        marker.scale.z = 0.03
        marker.color.a = 1.0
        marker.color.r = 0.7
        marker.color.g = 0
        marker.color.b = 0

        return marker


# Node initialization
def main():
    rospy.init_node('Controller')
    Controller()
    rospy.spin()

if __name__ == '__main__':
    main()