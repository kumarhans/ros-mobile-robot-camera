#!/usr/bin/env python
  
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError
import message_filters
from sensor_msgs.msg import Image, CameraInfo
import numpy as np

import cv2
import matplotlib.pyplot as plt
import numpy as np


def variance_of_laplacian(image):
	# compute the Laplacian of the image and then return the focus
	# measure, which is simply the variance of the Laplacian
	return cv2.Laplacian(image, cv2.CV_64F).var()
 
 

class image_converter:

  def __init__(self):
  
  
    self.image_subLeft = message_filters.Subscriber('/simulated/camera/left/image_raw', Image)
    self.image_subRight = message_filters.Subscriber('/simulated/camera/right/image_raw', Image)
    self.ts = message_filters.ApproximateTimeSynchronizer([self.image_subLeft, self.image_subRight], 10,1)
    self.ts.registerCallback(self.callback)

    self.image_pub1 = rospy.Publisher("/simulated/camera/left/blurred/image_raw",Image)
    self.image_pub2 = rospy.Publisher("/simulated/camera/right/blurred/image_raw",Image)

    self.bridge = CvBridge()
    self.cycle = 0
    self.imageCount = 0
    self.imageArr1 = []
    self.imageArr2 = []
    self.blurNum = 2
    self.x = np.zeros(1000)
    self.num = 0
    
  def callback(self,image1,image2):
   
    try:
      cv_image1 = self.bridge.imgmsg_to_cv2(image1, "bgr8")
      cv_image2 = self.bridge.imgmsg_to_cv2(image2, "bgr8")
    except CvBridgeError as e:
      print(e)

    if len(self.imageArr1) < self.blurNum:
        self.imageArr1.append(cv_image1)
        self.imageArr2.append(cv_image2)

    if len(self.imageArr1) == self.blurNum:
        a1 = np.mean(np.array(self.imageArr1),axis=0)
        arr1 = np.array(np.round(a1),dtype=np.uint8)

        a2 = np.mean(np.array(self.imageArr2),axis=0)
        arr2 = np.array(np.round(a2),dtype=np.uint8)

        stamp = image1.header.stamp
        i1 = self.bridge.cv2_to_imgmsg(arr1, "bgr8")
        i2 = self.bridge.cv2_to_imgmsg(arr2, "bgr8")

        i1.header.stamp = stamp
        i2.header.stamp = stamp

        self.image_pub1.publish(i1)
        self.image_pub2.publish(i2)

        gray = cv2.cvtColor(arr1, cv2.COLOR_BGR2GRAY)
        fm = variance_of_laplacian(gray)
        #self.x[self.num] = fm
       

        # cv2.imshow("Image window",arr2)
        # cv2.waitKey(3)
        self.imageArr1.pop(0)
        self.imageArr2.pop(0)
        self.num += 1
        
        # if self.num == 1000:
        #   plt.plot(self.x)
        #   plt.xlabel('frame#')
        #   plt.ylabel('blurry score (higher is less blurry)')
        #   plt.show()

     
    # if self.imageCount % 10 == 0:
    #     cv2.imwrite('periodicSlamCycle%dPhot0%d.png' % (self.cycle,self.imageCount),cv_image)
    # self.imageCount += 1
    
 

 

def main(args):
    ic = image_converter()
    rospy.init_node('image_blurrer', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

 