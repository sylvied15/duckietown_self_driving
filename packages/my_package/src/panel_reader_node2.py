#!/usr/bin/env python3

import os
import rospy
import cv2
import numpy as np
import time

from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped
from sensor_msgs.msg import CompressedImage, Image

from cv_bridge import CvBridge



x, y = 130, 75
w1, h1 = 640, 480
w, h = 55, 75

sigma = 1.2

def detecter_lignes(image):
    imghsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    img_gaussian_filter = cv2.GaussianBlur(img,(0,0), sigma)
    sobelx = cv2.Sobel(img_gaussian_filter,cv2.CV_64F,1,0)
    sobely = cv2.Sobel(img_gaussian_filter,cv2.CV_64F,0,1)
    
    Gmag = np.sqrt(sobelx*sobelx + sobely*sobely)

    threshold = np.ones((1,1), dtype=np.uint8)
    threshold=threshold*70
    mask_mag = (Gmag > threshold)
    
    white_lower_hsv = np.array([0, 0, 200])
    white_upper_hsv = np.array([360, 50, 255])
    yellow_lower_hsv = np.array([10, 100, 150])
    yellow_upper_hsv = np.array([70, 250, 250])
    
    mask_white = cv2.inRange(imghsv, white_lower_hsv, white_upper_hsv)
    mask_yellow = cv2.inRange(imghsv, yellow_lower_hsv, yellow_upper_hsv)
        
    mask_sobelx_pos = (sobelx > 0)
    mask_sobelx_neg = (sobelx < 0)
    mask_sobely_pos = (sobely > 0)
    mask_sobely_neg = (sobely < 0)
       
    mask_left_edge = np.ones((h, w), dtype=np.uint8)
    mask_right_edge = np.ones((h, w), dtype=np.uint8)

    mask_left_edge = np.ones((h, w), dtype=np.uint8)
    mask_right_edge = np.ones((h, w), dtype=np.uint8)
            
            
    mask_ground = np.ones(img.shape, dtype=np.uint8)

    mask_ground[:230, :640] = 0

    mask_right=np.ones(img.shape, dtype=np.uint8)
    mask_right[:480, :320] = 0

    mask_left=np.ones(img.shape, dtype=np.uint8)
    mask_left[:480, 320:640] = 0
            
    #Les masques left et right détectent respectivement la ligne jaune et la ligne blanche
    mask_left_edge = mask_ground*mask_mag * mask_sobelx_neg * mask_sobely_neg * mask_yellow*mask_left
    mask_right_edge = mask_ground*mask_mag * mask_sobelx_pos * mask_sobely_neg * mask_white*mask_right
            
            
            
    #pour le moteur de droite, on ne corrige la trajectoire que si la ligne blanche est detectée a gauche, et inversement
    left = np.zeros((480, 640), dtype=np.uint8)
    for i in range(0,480):
        for j in range(250,300):
            left[i,j] = 1
                        
    right = np.zeros((480, 640), dtype=np.uint8)
    for i in range(0,480):
        for j in range(350,400):
            right[i,j]=1
                    
  
        
    l = float(np.sum(mask_left_edge * left))
    r = float(np.sum(mask_right_edge * right))
            
    print('l :', l)
    print('r :', r)


    return(l, r)
        
def detecter_stop(image):
    bande_rouge=False
    x0, y0 = 150, 400
    w0, h0 = 350, 80


    cropped_image0 = image[y0:y0+h0, x0:x0+w0]

    imghsv0 = cv2.cvtColor(cropped_image0, cv2.COLOR_BGR2HSV)

    img0 = cv2.cvtColor(cropped_image0, cv2.COLOR_BGR2GRAY)
    
    img_gaussian_filter0 = cv2.GaussianBlur(img0,(0,0), sigma)
    sobelx0 = cv2.Sobel(img_gaussian_filter0,cv2.CV_64F,1,0)
    sobely0 = cv2.Sobel(img_gaussian_filter0,cv2.CV_64F,0,1)
    
    Gmag0 = np.sqrt(sobelx0*sobelx0 + sobely0*sobely0)
    
    threshold0 = np.ones((1,1), dtype=np.uint8)
    threshold0=threshold0*70
    mask_mag0 = (Gmag0 > threshold0)
    
    lower_red = np.array([0, 70, 80])
    upper_red = np.array([10, 255, 255])

    mask_red0 = cv2.inRange(imghsv0, lower_red, upper_red)
        
    mask0 = np.ones((h0, w0), dtype=np.uint8)

    mask0 = mask_mag0*mask_red0
            
                            
    l0 = float(np.sum(mask0))
    if l0 > 7000.0:
        bande_rouge = True  
            
    return (bande_rouge)

class PanelReaderNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(PanelReaderNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # static parameters
        vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{vehicle_name}/camera_node/image/compressed"
        wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"
        self._bridge = CvBridge()
        self._vel_left = 0.05
        self._vel_right = 0.05
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)
        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)
        self.bande_rouge=False
        self.image_pub_left = rospy.Publisher('left_image', Image, queue_size=1)
        self.image_pub_right = rospy.Publisher('right_image', Image, queue_size=1)
        self.stop_detected=False
        self.ls=0
        self.rs=0
        self.l_max = 0
        self.r_max = 0 
        self.l_min = float('inf')
        self.r_min = float('inf')
        self.stop_detected_time = 0
        
    
    	
    
    def callback(self, msg):
        rate=rospy.Rate(30)
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        stop = detecter_stop(image)
        if stop and not self.stop_detected and (time.time()-self.stop_detected_time >5):
            print("J'ai détecté un stop")
            self._vel_left = 0
            self._vel_right = 0
            self._publisher.publish(WheelsCmdStamped(vel_left=self._vel_left, vel_right=self._vel_right))
            time.sleep(3)
            self.stop_detected_time = time.time()
            self.stop_detected = True
        elif not stop and self.stop_detected:
            print("Le stop n'est plus détecté. Redémarrage du robot.")
            self.stop_detected = False

        else:
            (l, r) = detecter_lignes(image)
            self.l_max = max(l, self.l_max)
            self.r_max = max(r, self.r_max)
            self.l_min = min(l, self.l_min)
            self.r_min = min(r, self.r_min)

            if self.l_max==self.l_min :
            	self.ls=0
            else :
                self.ls =((l - self.l_min) / (self.l_max - self.l_min))
            
            
            if self.r_max==self.r_min :
            	self.rs=0
            else :
                self.rs =((r - self.r_min) / (self.r_max - self.r_min))
            
            print('ls :', self.ls)
            print('rs :', self.rs)
            self._vel_left = 0.1 + 0.11 * self.ls-0.11*self.rs
            self._vel_right = 0.1 + 0.11 * self.rs-0.11*self.ls
            print(self._vel_left)
            print(self._vel_right)
            message = WheelsCmdStamped(vel_left=self._vel_left, vel_right=self._vel_right)
            self._publisher.publish(message)
            
        

    def on_shutdown(self):
        stop = WheelsCmdStamped(vel_left=0, vel_right=0)
        self._publisher.publish(stop)

if __name__ == '__main__':
    # create the node
    node = PanelReaderNode(node_name='panel_reader_node')
    # Create a rate
    rospy.Timer(rospy.Duration(1.0/10.0), node.callback)
    rospy.spin()
