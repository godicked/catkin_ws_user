#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import ransac

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


    #HSV colorspace
    hsv=cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    
    lower = np.array([0,0,200])
    upper = np.array([360,40,255])

    # Threshold the HSV image to get only white
    hsvmask = cv2.inRange(hsv, lower, upper)

    width = hsvmask.shape[1]
    height = hsvmask.shape[0]

    print "height", height
    print "width", width

    xvals = []
    yvals = []

    for j in range(0, width, 4):
        for i in range(170, height, 4):
            if hsvmask[i,j] == 255:
                xvals.append([i])
                yvals.append([j])


    A = np.array(xvals)
    B = np.array(yvals)

    all_data = np.hstack( (A, B ))

    n_samples = len(all_data)
    n_inputs = 1
    n_outputs = 1

    print "count white points", len(all_data)
    print "all data", all_data


    input_columns = range(n_inputs) # the first columns of the array
    output_columns = [n_inputs+i for i in range(n_outputs)] # the last columns of the array

    debug = False
    model = ransac.LinearLeastSquaresModel(input_columns,output_columns,debug=debug)

    # run RANSAC algorithm
    ransac_fit, ransac_data = ransac.ransac(all_data,model,
                                     100, 5000, 10, 0, # misc. parameters
                                     debug=debug,return_all=True)
    
    if 1:
        import pylab

        sort_idxs = np.argsort(A[:,0])

        # print "sort", sort_idxs

        A_col0_sorted = A[sort_idxs] # maintain as rank-2 array

        # print "A", A_col0_sorted[:,0]

        # print "ransac fit", ransac_fit

        if 1:
            pylab.plot( A[:,0], B[:,0], 'k.', label='data' )
            pylab.plot( A[ransac_data['inliers'],0], B[ransac_data['inliers'],0], 'bx', label='RANSAC data' )
        # else:
        #     pylab.plot( A_noisy[non_outlier_idxs,0], B_noisy[non_outlier_idxs,0], 'k.', label='noisy data' )
        #     pylab.plot( A_noisy[outlier_idxs,0], B_noisy[outlier_idxs,0], 'r.', label='outlier data' )
        pylab.plot( A_col0_sorted[:,0],
                    np.dot(A_col0_sorted,ransac_fit)[:,0],
                    label='RANSAC fit' )
        # pylab.plot( A_col0_sorted[:,0],
        #             numpy.dot(A_col0_sorted,perfect_fit)[:,0],
        #             label='exact system' )
        # pylab.plot( A_col0_sorted[:,0],
        #             numpy.dot(A_col0_sorted,linear_fit)[:,0],
        #             label='linear fit' )
        #pylab.legend()
        pylab.show()
    
    self.image_sub.unregister()

      

def main(args):
  rospy.init_node('ransac', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
	main(sys.argv)