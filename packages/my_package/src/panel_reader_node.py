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


DIRECTION_LEFT = 0.1
DIRECTION_RIGHT = 0.1

x, y = 130, 75
w1, h1 = 640, 480
w, h = 55, 75

sigma = 1.2


class PanelReaderNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(PanelReaderNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # static parameters
        vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{vehicle_name}/camera_node/image/compressed"
        wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"
        self._bridge = CvBridge()
        self._vel_left = DIRECTION_LEFT
        self._vel_right = DIRECTION_RIGHT
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)
        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)
        self.l_max = 0
        self.r_max = 0 
        self.l_min = float('inf')
        self.r_min = float('inf')
        self.image_pub_left = rospy.Publisher('left_image', Image, queue_size=1)
        self.image_pub_right = rospy.Publisher('right_image', Image, queue_size=1)
    
    def callback(self, msg):
        while not rospy.is_shutdown():
        #On commence par définir la frenquence en Hz de reception d image par la camera
            rate = rospy.Rate(10)
            image = self._bridge.compressed_imgmsg_to_cv2(msg)
            
	#on convertit l image en HSV et en niveau de gris pour fabriquer nos filtres
            imghsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    	#on applique un filtre gaussien pour diminuer le bruit
            img_gaussian_filter = cv2.GaussianBlur(img,(0,0), sigma)
            
        #On applique le filtre Sobel par rapport aux axes x et y 
            sobelx = cv2.Sobel(img_gaussian_filter,cv2.CV_64F,1,0)
            sobely = cv2.Sobel(img_gaussian_filter,cv2.CV_64F,0,1)
    
       #on cree un filtre nous donnant le gradiant de limage puis on filtre seulement les forts contrastes
            Gmag = np.sqrt(sobelx*sobelx + sobely*sobely)
    
            threshold = np.ones((1,1), dtype=np.uint8)
            threshold=threshold*250
            mask_mag = (Gmag > threshold)
    
       #bornes up et low pour appliquer un filtre HSV pour du jaune et du blanc (lignes de la route)
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
        
            mask_left_edge = np.ones((h1, w1), dtype=np.uint8)
            mask_right_edge = np.ones((h1, w1), dtype=np.uint8)
            
            H = np.array([-4.137917960301845e-05, -0.00011445854191468058, -0.1595567007347241, 
              0.0008382870319844166, -4.141689222457687e-05, -0.2518201638170328, 
              -0.00023561657746150284, -0.005370140574116084, 0.9999999999999999])

            H = np.reshape(H,(3, 3))
            Hinv = np.linalg.inv(H)
            
            mask_ground = np.ones(img.shape, dtype=np.uint8)
            # Real world coordinates transformed into corresponding image coordinates
            CC1 = np.array([[3.0, 5.0, 1]]).transpose()
            CC1prime = Hinv.dot(CC1)
            CC1prime = np.uint(CC1prime / CC1prime[2])
            CC2 = np.array([[3.0, -5.0, 1]]).transpose()
            CC2prime = Hinv.dot(CC2)
            CC2prime = np.uint(CC2prime / CC2prime[2])

            xmin = np.minimum(CC1prime[0], CC2prime[0])
            xmax = np.maximum(CC1prime[0], CC2prime[0])
            ymin = np.minimum(CC1prime[1], CC2prime[1])
            ymax = np.maximum(CC1prime[1], CC2prime[1])

            xmin = np.minimum(xmin, 0).item()
            xmax = np.minimum(xmax, w).item()
            ymin = np.minimum(ymin, 0).item()
            ymax = np.minimum(ymax, h).item()

            mask_ground[:ymax, :xmax] = 0
    
            mask_left_edge = mask_ground*mask_mag * mask_sobelx_neg * mask_sobely_neg * mask_yellow
            mask_right_edge = mask_ground*mask_mag * mask_sobelx_pos * mask_sobely_neg * mask_white
            
            ros_img_left = self._bridge.cv2_to_imgmsg(cv2.convertScaleAbs(Gmag * mask_left_edge), "mono8")
            ros_img_right = self._bridge.cv2_to_imgmsg(cv2.convertScaleAbs(Gmag * mask_right_edge), "mono8")
            
            mask_ground = np.ones(img.shape, dtype=np.uint8)
            for i in range(480):
                for j in range(320):
                    mask_ground[i][j]=0


            self.image_pub_left.publish(ros_img_left) 
            self.image_pub_right.publish(ros_img_right) 

            self.left = np.zeros((h1, w1), dtype=np.uint8)
            for i in range(0,480):
                for j in range(320,500):
                            self.left[i,j] = 1
                        
            self.right = np.zeros((h1, w1), dtype=np.uint8)
            for i in range(0,480):
                for j in range(150,320):
                    self.right[i,j]=1
                    
  
        
            l = float(np.sum(mask_left_edge * self.left))
            r = float(np.sum(mask_right_edge * self.right))

      
            
            self.l_max = max(l, self.l_max)
            self.r_max = max(r, self.r_max)
            self.l_min = min(l, self.l_min)
            self.r_min = min(r, self.r_min)

            if self.l_max==self.l_min :
            	ls=0
            else :
                ls =((l - self.l_min) / (self.l_max - self.l_min))
            
            
            if self.r_max==self.r_min :
            	rs=0
            else :
                rs =((r - self.r_min) / (self.r_max - self.r_min))
            

            vel_left = 0.2 + ls * 0.5
            vel_right = 0.2 + rs * 0.5

            message = WheelsCmdStamped(vel_left=self._vel_left, vel_right=self._vel_right)
            self._publisher.publish(message)
        
            image_cropped = image[400:460, 250:450, :]

            blurred = cv2.GaussianBlur(image_cropped, (5, 5), 0)

            lower_red = np.array([0, 100, 100])
            upper_red = np.array([10, 255, 255])

            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            mask = cv2.inRange(hsv, lower_red, upper_red)

            mask = cv2.resize(mask, (image_cropped.shape[1], image_cropped.shape[0]))

            red_objects = cv2.bitwise_and(image_cropped, image_cropped, mask=mask)

            gray = cv2.cvtColor(red_objects, cv2.COLOR_BGR2GRAY)

            contours, _ = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            cv2.drawContours(image_cropped, contours, -1, (0, 0, 255), 2)
            
            if (len(contours) > 0):
                print("J'ai détecté un panneau stop dans l'image.")
                message = WheelsCmdStamped(vel_left=0, vel_right=0)
                self._publisher.publish(message)
                time.sleep(3)
                message = WheelsCmdStamped(vel_left=self._vel_left, vel_right=self._vel_right)
                self._publisher.publish(message)
                time.sleep(5)

                

    def on_shutdown(self):
        stop = WheelsCmdStamped(vel_left=0, vel_right=0)
        self._publisher.publish(stop)

if __name__ == '__main__':
    # create the node
    node = PanelReaderNode(node_name='panel_reader_node')
    # run node
    rospy.spin()

