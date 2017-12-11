#!/usr/bin/env python
import roslib
import pickle
import sys
import rospy
import cv2
import numpy as np
from math import isnan, atan, cos, sin, pi, atan2
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from cv_bridge import CvBridge, CvBridgeError
import tf
import math


class image_converter:
  def __init__(self):
    self.image_pub = rospy.Publisher("/image_processing/usb_cam_s", Image, queue_size=1)
    self.gps_pub = rospy.Publisher("/visual_gps/odom", Odometry, queue_size = 10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/usb_cam/image_rect_color",Image,self.callback, queue_size=1)
    self.yaw_sub = rospy.Subscriber("/model_car/yaw", Float32, self.yawCallback, queue_size=1)
    self.init = False
    self.angle_offset = 0.0
    self.position_offset = [0,0]

  def yawCallback(self, yaw):
    self.yaw = yaw.data

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # Blur image to remove color noise
    cv_image = cv2.GaussianBlur(cv_image, (5,5), 0)

    # cv2.imshow("img", cv_image)
    # k = cv2.waitKey(5) & 0xFF
    # if k == 27:
        # return


    # define range of lamps in RGB
    # lower_rgb = np.array([[30, 110, 50], #green
    #                      [220, 40 , 40], #red
    #                      [70, 40, 240], #blue
    #                      [190, 100, 190]]) #purple

    # upper_rgb = np.array([[70, 140, 70], # green
    #                      [255, 73, 70], #red
    #                      [90, 60, 255], #blue 
    #                      [255, 150, 255]]) #purple

    lower_rgb = np.array([[0, 110, 0], #green
                         [140, 0 , 0], #red
                         [20, 20, 230], #blue
                         [120, 20, 190]]) #purple

    upper_rgb = np.array([[70, 140, 70], # green
                         [255, 73, 70], #red
                         [90, 60, 255], #blue 
                         [255, 150, 255]]) #purple
                    
    
    lower_rgb = np.fliplr(lower_rgb) # flip to b,g,r for cv
    upper_rgb = np.fliplr(upper_rgb) # flip to b,g,r for cv

    lamps_seen = [0]*4
    lamp_image = [0]*4

    for k in range(4):
        # find lamp k in cv_image
        mask_rgb = cv2.inRange(cv_image, lower_rgb[k], upper_rgb[k])
        indices = np.nonzero(mask_rgb)
        
        # save lamp coordinate
        lamp_image[k] = [ np.mean(indices[0]), np.mean(indices[1]) ]

        # check if lamp is found
        if not isnan(lamp_image[k][0]):
            lamps_seen[k] =1
            #image = cv2.circle(image, tuple([ int(lamp_image[k][1]) ,int( lamp_image[k][0]) ]), 6, upper_rgb[k], 2)


    # lamp_world = [
    #     [2.29, 1.14 ], #green lamp
    #     [3.55, 3.03 ], #red lamp
    #     [4.18, 1.77 ], #blue lamp
    #     [2.29, 2.40 ]] #purple lamp

    lamp_world = [
        [1.14, 2.29 ], #green lamp
        [3.03, 3.55 ], #red lamp
        [1.77, 4.18 ], #blue lamp
        [2.40, 2.29 ]] #purple lamp

    #centers of lamps in image
    center_im = np.mean(
        np.array([ lamp_image[k] for k in range(4) if lamps_seen[k] ]),
        axis=0
        )

    # print 'center image', center_im

    #center of lamps in real world
    center_rw = np.mean(
        np.array([ lamp_world[k] for k in range(4) if lamps_seen[k] ]),
        axis=0)


    # calculate rotation matrix
    scaling = []
    rotation_angle = []
    sum_x=0
    sum_y=0
    for k in range(4):
        if lamps_seen[k]:
            scaling.append( np.sqrt(  ((lamp_world[k][0]-center_rw[0])**2+( lamp_world[k][1]-center_rw[1] )**2) / float(((lamp_image[k][0]-center_im[0])**2+( lamp_image[k][1]-center_im[1] )**2)) ) )

            atan_world = atan2( (lamp_world[k][1] - center_rw[1] ), (lamp_world[k][0] - center_rw[0]) )

            atan_image = atan2( (lamp_image[k][1] - center_im[1]), (lamp_image[k][0] - center_im[0]) )


            # print 'atan1', atan_1
            # print 'atan2', atan_2

            #rotation between world and image is rotation between both points
            rotation_angle.append( atan_world - atan_image )

            b_x=lamp_image[k][0] - center_im[0]
            b_y=lamp_image[k][1] - center_im[1]
            w_x=lamp_world[k][0] - center_rw[0]
            w_y=lamp_world[k][1] - center_rw[1]
            sum_x+=(b_x*w_y-b_y*w_x)
            sum_y+=b_x*w_x+b_y*w_y

            if rotation_angle[-1] > 2*pi:
                rotation_angle[-1] -= 2*pi
            if rotation_angle[-1] < 0:
                rotation_angle[-1] += 2*pi

    print(lamps_seen)
    scaling_mean = np.mean( scaling)
    # rotation_angle_mean = np.mean( rotation_angle[1:], axis = 0 )
    # print 'angle_mean 1', math.degrees(rotation_angle_mean)
    rotation_angle_mean = atan2(sum_x,sum_y)


    # print(math.degrees(self.angle_offset))

    

    print 'angle_mean', math.degrees(rotation_angle_mean)

    # print( 'angle: ' + str(rotation_angle))
    # print( 'scaling: ' + str(scaling_mean))

    R = scaling_mean * np.array([
        [cos(rotation_angle_mean), -sin(rotation_angle_mean)],
        [sin(rotation_angle_mean),  cos(rotation_angle_mean)]
        ])

    # current_pos_im = [226, 317]
    pos_image = [cv_image.shape[0] / 2.0, cv_image.shape[1] / 2.0]
    # print "pos_image", pos_image
    pos_world = np.dot(R, pos_image - center_im) + center_rw
    pos_world -= self.position_offset

    print 'position in world', pos_world

    if not self.init:
      self.init = True
      self.angle_offset = rotation_angle_mean
      self.position_offset = pos_world

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)
      
    odom1 = Odometry()
    odom1.header.frame_id  = 'odom'
    odom1.header.seq = data.header.seq
    odom1.pose.pose.position.x = -pos_world[1]
    odom1.pose.pose.position.y = pos_world[0]

    # print 'angle offset: ', math.degrees(self.angle_offset)
    rotation_angle_mean -= self.angle_offset
    print 'rotation with offset:', math.degrees(rotation_angle_mean)


    odom1.pose.pose.orientation = Quaternion( *tf.transformations.quaternion_from_euler(0, 0, rotation_angle_mean ) )

    if sum(lamps_seen) >= 3:
    	self.gps_pub.publish( odom1 )

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


