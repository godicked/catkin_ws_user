#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/image_processing/grayscale",Image, queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/app/camera/color/image_raw",Image,self.callback, queue_size=1)


  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)


    #RGB colorspace
    rgb = cv_image
    
    lower = np.array([200,200,200])
    upper = np.array([255,255,255])

    # Threshold the HSV image to get only white
    rgbmask = cv2.inRange(rgb, lower, upper)


    #HSV colorspace
    hsv=cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    
    lower = np.array([0,0,200])
    upper = np.array([360,40,255])

    # Threshold the HSV image to get only white
    hsvmask = cv2.inRange(hsv, lower, upper)


    #YUV colorspace
    yuv=cv2.cvtColor(cv_image, cv2.COLOR_BGR2YUV)
    
    lower = np.array([200,0,0])
    upper = np.array([255,255,255])

    # Threshold the HSV image to get only white
    yuvmask = cv2.inRange(yuv, lower, upper)


    Show images
    cv2.imshow('rgb', rgbmask)
    cv2.imshow('hsv', hsvmask)
    cv2.imshow('yuv', yuvmask)

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        return

    # try:
    #   self.image_pub.publish(self.bridge.cv2_to_imgmsg(gray, "passthrough"))
    # except CvBridgeError as e:
    #   print(e)
      

def main(args):
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
	main(sys.argv)