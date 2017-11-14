#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import numpy.matlib


#import matplotlib
#matplotlib.use('Agg')
from matplotlib import pyplot as plt

# from __future__ import print_function



class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/image_processing/white_points",PoseArray, queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/image_processing/blackwhite",Image,self.callback, queue_size=1)
    
  def findPoint(self, x, y, width, height, img):
		for i in range(x, x+width):
			for j in range(y, y+height):
				if img[j, i] >= 200:
					return [i, j] # as [x,y]

  def solvePnP(self, objectPoints, cameraPoints):
    intrinsics = np.matrix('614.1699 0 329.9491; 0 614.9002 237.2788; 0 0 1')
    distCoeffs = np.matrix('0.1115 -0.1089 0 0')

    rvec = np.zeros((3,1), dtype=np.double)
    tvec = np.zeros((3,1), dtype=np.double)

    print "objectPoints: \n" + str(objectPoints)
    print "cameraPoints: \n" + str(cameraPoints)

    cv2.solvePnP(objectPoints, cameraPoints, intrinsics, distCoeffs, rvec, tvec)

    print "rvec: \n" + str(rvec)
    print "tvec: \n" + str(tvec)
    rmat= np.matlib.zeros((3, 3), dtype=np.double)
    cv2.Rodrigues(rvec,rmat)
    print "rmat: \n" + str(rmat)
    inv_rmat=np.linalg.inv(rmat)
    inv_rvec=-inv_rmat*tvec
    print "inv_rmat: \n" + str(inv_rmat)
    print "inv_rvec: \n" + str(inv_rvec)

    yaw=np.arctan(inv_rmat[[1,0]],inv_rmat[[0,0]])
    pitch = np.arctan(-inv_rmat[[2,0]],np.sqrt(np.power(inv_rmat[[2,1]],2)+np.power(inv_rmat[[2,2]],2)))
    roll = np.arctan(inv_rmat[[2,1]],inv_rmat[[2,2]])

    print "yaw: \n" + str(yaw)
    print "pitch: \n" + str(pitch)
    print "roll: \n" + str(roll)

 #   sy = np.sqrt(rmat[[0,0]] * rmat[[0,0]] + rmat[[1,0]] * rmat[[1,0]] )
    
		
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
    except CvBridgeError as e:
      print(e)
      
    width = cv_image.shape[1]
    height = cv_image.shape[0]
    
    cameraPoints = []
    
    cameraPoints.append(self.findPoint(260, 320, 40, 40, cv_image))
    
    cameraPoints.append(self.findPoint(290, 210, 40, 40, cv_image))
    
    cameraPoints.append(self.findPoint(300, 150, 40, 40, cv_image))
    cameraPoints.append(self.findPoint(420, 160, 40, 40, cv_image))
    cameraPoints.append(self.findPoint(450, 210, 40, 40, cv_image))
    cameraPoints.append(self.findPoint(490, 330, 40, 40, cv_image))
    
    
    print "with: \n" + str(width)
    print "height: \n" + str(height)
    print "cameraPoints: \n" + str(cameraPoints)
		
    objectPoints = []
    objectPoints.append([0, 80, 0])
    objectPoints.append([0, 40, 0])
    objectPoints.append([0, 0, 0])
    objectPoints.append([28, 80, 0])
    objectPoints.append([28, 40, 0])
    objectPoints.append([28, 0, 0])

    self.solvePnP(np.array(objectPoints, dtype=np.float_), np.array(cameraPoints, dtype=np.float_))

    

    
      

def main(args):
  rospy.init_node('blackwhite_converter', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
	main(sys.argv)
