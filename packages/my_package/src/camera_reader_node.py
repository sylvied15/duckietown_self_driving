#!/usr/bin/env python3

import os
import rospy
import numpy as np
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, Image

import cv2
from cv_bridge import CvBridge

sigma = 1.2
x, y = 130, 75
w1, h1 = 640, 480
w, h = 55, 75
class CameraReaderNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(CameraReaderNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self.bande_rouge = False
        # bridge between OpenCV and ROS
        self._bridge = CvBridge()
        # construct subscriber
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)
        self.image_pub = rospy.Publisher('image', Image, queue_size=1)

    def callback(self, msg):
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        cropped_image = image[y:y+h, x:x+w]

        img2hsv = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2HSV)

        img2 = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2GRAY)
    
        img2_gaussian_filter = cv2.GaussianBlur(img2,(0,0), sigma)
        sobelx2 = cv2.Sobel(img2_gaussian_filter,cv2.CV_64F,1,0)
        sobely2 = cv2.Sobel(img2_gaussian_filter,cv2.CV_64F,0,1)
            
        Gmag2 = np.sqrt(sobelx2*sobelx2 + sobely2*sobely2)
        threshold = np.ones((1,1), dtype=np.uint8)
        threshold=threshold*250
        mask_mag2 = (Gmag2 > threshold)

        lower_red = np.array([0, 70, 80])
        upper_red = np.array([10, 255, 255])

        mask_red = cv2.inRange(img2hsv, lower_red, upper_red)
        
        mask = np.ones((h, w), dtype=np.uint8)

        mask = mask_mag2*mask_red
        
        ros_img = self._bridge.cv2_to_imgmsg(cv2.convertScaleAbs(mask), "mono8")
        rate=rospy.Rate(20)
        self.image_pub.publish(ros_img)
        rate.sleep()

        l = float(np.sum(mask))
            
        if (l>6000.0):
            print("J'ai détecté un panneau stop dans l'image.")
            self.stop=True
            
if __name__ == '__main__':
    # create the node
    node = CameraReaderNode(node_name='camera_reader_node')
    # keep spinning
    rospy.spin()

