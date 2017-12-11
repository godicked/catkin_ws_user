#!/usr/bin/env python
import roslib
import pickle
import sys
import rospy
import cv2
import numpy as np
from math import isnan, atan, cos, sin, pi, atan2
from sklearn import linear_model
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from cv_bridge import CvBridge, CvBridgeError
import tf


xt_list = [[0]*2]*2
sigmat_list = [[[0]*2]*2]*2 


class image_converter:
  def __init__(self):
    self.image_pub = rospy.Publisher("/image_processing/usb_cam_s", Image, queue_size=1)
    self.gps_pub = rospy.Publisher("/visual_gps/odom", Odometry, queue_size = 10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback, queue_size=1)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    ### BUILD RGB ###
    b,g,r = cv2.split(cv_image)
    r_blur = cv2.GaussianBlur(r,(5,5),0)
    g_blur = cv2.GaussianBlur(g,(5,5),0)
    b_blur = cv2.GaussianBlur(b,(5,5),0)


    yuv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2YUV);
    y,u,v = cv2.split(yuv)
    v_blur = cv2.GaussianBlur(v,(5,5),0)


    # define range of balloons in RGB
    lower_rgb = np.array([[10, 46, 0], #green
                         [220, 40 , 40], #red
                         [70, 40, 240], #blue
                         [230, 100, 230]]) #purple
    lower_rgb = np.fliplr(lower_rgb) # flip to b, g, r

    upper_rgb = np.array([[33,57, 12], # green
                         [255, 73, 70], #red
                         [90, 60, 255], #blue 
                         [255, 150, 255]]) #purple
    upper_rgb = np.fliplr(upper_rgb) # flip to b, g, r

    lower_v = np.array([[166],
                       [124],
                       [50],
                       [83]])
    upper_v = np.array([[178],
                       [138],
                       [70],
                       [110]])

    image = cv_image
    #images = {}
    balloons_seen = [0]*4
    balloon_pos_im = [0]*4

    for k in range(4):
        mask_rgb = cv2.inRange(cv_image, lower_rgb[k], upper_rgb[k])
        mask_v   = cv2.inRange(v  , lower_v[k], upper_v[k])
        just_balloon = cv2.bitwise_and(mask_rgb, mask_v)
        #images[k] = cv2.bitwise_and(mask_rgb, mask_v)
        indices = np.nonzero(mask_rgb)
        
        balloon_pos_im[k] = [ np.mean(indices[0]), np.mean(indices[1]) ]
        if not isnan(balloon_pos_im[k][0]): # check, if balloon appeared in picture
            balloons_seen[k] =1
            image = cv2.circle(image, tuple([ int(balloon_pos_im[k][1]) ,int( balloon_pos_im[k][0]) ]), 6, upper_rgb[k], 2)


    balloon_pos_rw = [
        [2.29, 1.14 ], #green lamp
        [3.55, 3.03 ], #red lamp
        [4.18, 1.77 ], #blue lamp
        [2.29, 2.40 ]] #purple lamp

    #centers of lamps in image
    center_im = np.mean(
        np.array([ balloon_pos_im[k] for k in range(4) if balloons_seen[k] ]),
        axis=0
        )

    print 'center image', center_im

    #center of lamps in real world
    center_rw = np.mean(
        np.array([ balloon_pos_rw[k] for k in range(4) if balloons_seen[k] ]),
        axis=0)
#    print('center  rw ', center_rw)

    print "center rw", center_rw

    # calculate rotation matrix
    scaling = []
    rotation_angle = []
    sum_x=0
    sum_y=0
    for k in range(4):
        if balloons_seen[k]:
            scaling.append( np.sqrt(  ((balloon_pos_rw[k][0]-center_rw[0])**2+( balloon_pos_rw[k][1]-center_rw[1] )**2) / float(((balloon_pos_im[k][0]-center_im[0])**2+( balloon_pos_im[k][1]-center_im[1] )**2)) ) )

            atan_1 = atan2( (balloon_pos_rw[k][1] - center_rw[1] ), (balloon_pos_rw[k][0] - center_rw[0]) )

            atan_2 = atan2( (balloon_pos_im[k][1] - center_im[1]), (balloon_pos_im[k][0] - center_im[0]) )

            rotation_angle.append( atan_1 - atan_2 )
            b_x=balloon_pos_im[k][0] - center_im[0]
            b_y=balloon_pos_im[k][1] - center_im[1]
            w_x=balloon_pos_rw[k][0] - center_rw[0]
            w_y=balloon_pos_rw[k][1] - center_rw[1]
            sum_x+=(b_x*w_y-b_y*w_x)
            sum_y+=b_x*w_x+b_y*w_y
            if rotation_angle[-1] > 2*pi:
                rotation_angle[-1] -= 2*pi
            if rotation_angle[-1] < 0:
                rotation_angle[-1] += 2*pi

    print(balloons_seen)
    scaling_mean = np.mean( scaling)
    rotation_angle_mean = np.mean( rotation_angle[1:], axis = 0 )
    rotation_angle_mean = atan2(sum_x,sum_y)

    print( 'angle: ' + str(rotation_angle))
    print( 'scaling: ' + str(scaling_mean))

    R = scaling_mean * np.array([
        [cos(rotation_angle_mean), -sin(rotation_angle_mean)],
        [sin(rotation_angle_mean),  cos(rotation_angle_mean)]
        ])

    # current_pos_im = [226, 317]
    current_pos_im = [240, 320]
    current_pos_rw = np.dot(R, current_pos_im - center_im) + center_rw


    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
    except CvBridgeError as e:
      print(e)
      

    odom1 = Odometry()
    odom1.header.frame_id  = 'map'
    odom1.header.seq = data.header.seq
    odom1.pose.pose.position.x = current_pos_rw[0]
    odom1.pose.pose.position.y = -current_pos_rw[1]

    odom1.pose.pose.orientation = Quaternion( *tf.transformations.quaternion_from_euler(0, 0, pi-1.0*rotation_angle_mean ) )

    if sum(balloons_seen) >= 3:
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


