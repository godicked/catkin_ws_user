#!/usr/bin/env python
import roslib
import pickle
import sys
import rospy
import cv2
import numpy as np
from math import isnan, atan, cos, sin, pi, atan2
from std_msgs.msg import String, Float32
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from cv_bridge import CvBridge, CvBridgeError
import tf
import math


class image_converter:
  def __init__(self):
    self.gps_pub = rospy.Publisher("/visual_gps/odom", Odometry, queue_size = 100)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/usb_cam/image_rect_color", Image, self.callback, queue_size=1)
    # self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback, queue_size=1)

    self.lamp_pub = rospy.Publisher("/lamp_marker", Marker, queue_size=1)
    self.init = False

  def buildMarker(self, id, x, y, color):

    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "lamps"
    marker.id = id
    marker.type = Marker.SPHERE
    marker.action = Marker.MODIFY
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 1
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.a = 1.0
    marker.color.r = color[0] / 255.0
    marker.color.g = color[1] / 255.0
    marker.color.b = color[2] / 255.0

    return marker


  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # Blur image to remove color noise
    cv_image = cv2.GaussianBlur(cv_image, (5,5), 0)

    cv_image = cv2.flip(cv_image, 1)
    cv_image = cv2.transpose(cv_image)
    cv_image = cv2.flip(cv_image, 1)
    cv_image = cv2.flip(cv_image, 0)


    # rgb range
    lower_rgb = np.array([[0, 80, 0], #green
                         [140, 0 , 0], #red
                         [20, 20, 230], #blue
                         [120, 20, 190]]) #purple

    upper_rgb = np.array([[70, 140, 70], # green
                         [255, 73, 70], #red
                         [90, 60, 255], #blue 
                         [255, 150, 255]]) #purple

    rgb = upper_rgb
                    
    
    lower_rgb = np.fliplr(lower_rgb) # flip to b,g,r for cv
    upper_rgb = np.fliplr(upper_rgb) # flip to b,g,r for cv

    lamps_seen = [0]*4
    lamp_image = [0]*4

    # lamp_world = [
    #   [1.14, 2.29 ], #green lamp
    #   [3.03, 3.55 ], #red lamp
    #   [1.77, 4.18 ], #blue lamp
    #   [2.40, 2.29 ]] #purple lamp

    lamp_world = [
      [2.29, 1.14 ], #green lamp
      [3.55, 3.03], #red lamp
      [4.18, 1.77], #blue lamp
      [2.29, 2.40]] #purple lamp

    for i in range(4):
        # find lamp i in cv_image
        mask_rgb = cv2.inRange(cv_image, lower_rgb[i], upper_rgb[i])
        indices = np.nonzero(mask_rgb)
        
        # save lamp coordinate
        lamp_image[i] = [ np.mean(indices[1]), np.mean(indices[0]) ]

        # check if lamp is found
        if not isnan(lamp_image[i][0]):
            lamps_seen[i] = 1
            cv2.circle(cv_image,((int)(lamp_image[i][0]),(int)(lamp_image[i][1])), 10, upper_rgb[i], -1)

    print(lamps_seen)

    
    #centers of lamps in image
    center_im = np.mean([lamp_image[i] for i in range(4) if lamps_seen[i]], axis=0)

    # print 'center image', center_im

    #center of lamps in real world
    center_rw = np.mean([lamp_world[i] for i in range(4) if lamps_seen[i]], axis=0)


    # calculate rotation matrix
    scaling = []
    sum_x=0
    sum_y=0

    for i in range(4):
        if lamps_seen[i]:

            #compute scaling factor between real world and image
            scaling.append( np.sqrt(((lamp_world[i][0]-center_rw[0])**2+( lamp_world[i][1]-center_rw[1] )**2) / float(((lamp_image[i][0]-center_im[0])**2+( lamp_image[i][1]-center_im[1] )**2)) ) )

            i_x=lamp_image[i][0] - center_im[0]
            i_y=lamp_image[i][1] - center_im[1]
            w_x=lamp_world[i][0] - center_rw[0]
            w_y=lamp_world[i][1] - center_rw[1]
            
            det = i_x*w_y - i_y*w_x
            dot = i_x*w_x + i_y*w_y
            # print 'angle', math.degrees(atan2(det, dot))
            sum_x += det
            sum_y += dot

            # print'lamp', i, 'pos', lamp_image[i]
            # print'lamp pos center', lamp_image[i] - center_im

    scaling_mean = np.mean( scaling)
    rotation_angle_mean = atan2(sum_x, sum_y)

    print 'yaw', math.degrees(rotation_angle_mean)

    R = scaling_mean * np.array([
        [cos(rotation_angle_mean), -sin(rotation_angle_mean)],
        [sin(rotation_angle_mean),  cos(rotation_angle_mean)]
        ])

    # center of image
    pos_image = np.array([cv_image.shape[1] / 2.0, cv_image.shape[0] / 2.0])
    cv2.circle(cv_image,((int)(pos_image[0]),(int)(pos_image[1])), 10, (255,255,255), -1)

    # cv2.circle(cv_image,((int)(10),(int)(200)), 10, (255,255,255), -1)


    
    # transform image position to world position
    pos_world = np.dot(R, pos_image - center_im) + center_rw

    print 'position', pos_world

    # Initialise offset to start at position (0,0) with rotation 0
    if not self.init:
      self.init = True
      self.angle_offset = rotation_angle_mean
      self.position_offset = [pos_world[0], pos_world[1]]


    # sending odometry  
    odom = Odometry()
    odom.header.frame_id  = 'map'
    odom.header.seq = data.header.seq
    odom.pose.pose.position.x = pos_world[0]
    odom.pose.pose.position.y = pos_world[1]

    q = tf.transformations.quaternion_from_euler(0, 0, rotation_angle_mean )
    odom.pose.pose.orientation = Quaternion(*q)
    
    if sum(lamps_seen) >= 3:
    	self.gps_pub.publish( odom )


    # Publish tranform from map to odom
    br = tf.TransformBroadcaster()
    br.sendTransform( (self.position_offset[0], self.position_offset[1], 0),
                      tf.transformations.quaternion_from_euler(math.pi, 0, self.angle_offset),
                      rospy.Time.now(),
                      "odom",
                      "map")

    for i in range(4):
      marker = self.buildMarker(i, lamp_world[i][0], lamp_world[i][1], rgb[i])
      self.lamp_pub.publish(marker)

    # cv2.imshow('flipped', cv_image)

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        return


def main(args):
  rospy.init_node('visual_gps', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
  main(sys.argv)


