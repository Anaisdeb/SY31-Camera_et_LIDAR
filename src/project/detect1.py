#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Created on Thu Jun  3 15:04:35 2021

@authors: Merwane and Anaïs
"""

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point32
from std_msgs.msg import Float32


class CameraNode:
    def __init__(self):
        # Creates a node called and registers it to the ROS master
        rospy.init_node('detect')

        # CvBridge is used to convert ROS messages to matrices manipulable by OpenCV
        self.bridge = CvBridge()

        # Initialize the node parameters
        self.position= Point32()
        self.surface= Float32()
        
        H = int(194/ 2) # de 0..180
        S = int(100* 2.55) # de 0..255
        V = int(54* 2.55) # de 0.. 255
        offsetH = 10
        offsetHSV = 75
        
        self.HSVmin = np.array([H - offsetH, S - offsetHSV, V - offsetHSV])
        self.HSVmax = np.array([H + offsetH, S + offsetHSV, V + offsetHSV])

        # Publisher to the output topics.
        self.pub_img = rospy.Publisher('~output', Image, queue_size=10)
        
        #Publisher for the characteristics of the target
        self.objet = rospy.Publisher('direction', Point32,queue_size=10)
        self.surf = rospy.Publisher('surf', Float32,queue_size=10)

        # Subscriber to the input topic. self.callback is called when a message is received
        self.subscriber = rospy.Subscriber('/camera/image_rect_color', Image, self.callback)

    def callback(self, msg):
        '''
        Function called when an image is received.
        msg: Image message received
        img_bgr: Width*Height*3 Numpy matrix storing the image
        '''
        # Convert ROS Image -> OpenCV
        try:
            img_bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logwarn(e)
            return
        
        y, x, z = img_bgr.shape
        
        ## Conversion and HSV mask
        img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        hsv_mask = cv2.inRange(img_hsv, self.HSVmin, self.HSVmax )
        
        # Blur image
        dest = cv2.blur(hsv_mask, (5,5) )
        
        ## Contour calculation
        contours, hierarchy = cv2.findContours(dest, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        ## Calculating the maximum area
        if (contours == []) :
           self.position.x=0
           self.position.y=0
           self.position.z=0
           self.objet.publish(self.position)
           self.surface.data= 0
           self.surf.publish(self.surface)
        else:
            aireMax = 0
            iMax = 0
            for i in range(0, len(contours)) :
                aireContour = cv2.contourArea(contours[i])
                if(aireContour > aireMax) :
                    aireMax = aireContour
                    iMax=i
            
            # Calculation of the area center
            center = np.mean(contours[iMax], axis=0)
            coordcenter = center[0]
            coordcenter = (int(coordcenter[0]), int(coordcenter[1]))

            # Drawing center
            cv2.circle(img_bgr, coordcenter, 20, (0,0,0), -1)
            
            # Positioning of the target in the camera image
            if ( coordcenter[0] < x/3 ) :
                self.position.x=1
                self.position.y=0
                self.position.z=0
                self.objet.publish(self.position)
                #print("tourner à gauche")
            elif ( coordcenter[0] > (2*x)/3 ) :
                self.position.x=0
                self.position.y=0
                self.position.z=1
                self.objet.publish(self.position)
                #print("tourner à droite")
            else:
                self.position.x=0
                self.position.y=1
                self.position.z=0
                self.objet.publish(self.position)
                #print("aller tout droit")
            
            # Drawing of contours
            cv2.drawContours(img_bgr, contours, -1, (0,255,0), 3)
            cv2.drawContours(img_bgr, contours[iMax], -1, (255,0,0), 3)
            
            ## Calculation and drawing of the rectangle
            x, y, w, h = cv2.boundingRect(contours[iMax]) # retourne coordonnée d'un coin et la taille de l'image
            self.surface.data= w*h
            self.surf.publish(self.surface)
            cv2.rectangle(img_bgr, (x,y), (x+w, y+h), (0,0,255),3)
            
            # Calculation and drawing of the convexHull
            hull = []
            hull.append(cv2.convexHull(contours[iMax], False) )
            cv2.drawContours(img_bgr, hull, -1, (255,0,255), 3)
        
        # Convert OpenCV -> ROS Image and publish
        try:
            self.pub_img.publish(self.bridge.cv2_to_imgmsg(img_bgr, "bgr8")) # /!\ 'mono8' for grayscale images, 'bgr8' for color images
        except CvBridgeError as e:
            rospy.logwarn(e)

if __name__ == '__main__':
    # Start the node and wait until it receives a message or stopped by Ctrl+C
    node = CameraNode()
    rospy.spin()
