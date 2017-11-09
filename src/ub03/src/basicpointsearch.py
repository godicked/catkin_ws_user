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

#import matplotlib
#matplotlib.use('Agg')
from matplotlib import pyplot as plt

# from __future__ import print_function



class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/image_processing/white_points",PoseArray, queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/image_processing/blackwhite",Image,self.callback, queue_size=1)

	def findPoint(x, y, width, height, img):
		for i in range(x, x+width):
			for j in range(y, y+height):
				if img[i, j] >= 200:
					return [i, j]
		
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
    except CvBridgeError as e:
      print(e)
      
    width = cv_image.shape[0]
    height = cv_image.shape[1]
		
		"""    
    count = 0
    for x in range(width):
    	for y in range(height):
    		if cv_image[x, y] <= 200:
    			count += 1
    
    print(count)
    """
    
    position = self.findPoint(30, 20, 100, 100, cv_image)
    print(position)
    
    print(width)
    print(height)

		
    #try:
      #self.image_pub.publish(self.bridge.cv2_to_imgmsg(thresh1, "mono8"))
    #except CvBridgeError as e:
      #print(e)

      

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
