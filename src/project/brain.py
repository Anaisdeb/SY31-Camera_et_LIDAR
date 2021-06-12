#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Created on Thu Jun  3 15:04:35 2021

@authors: Merwane and Anaïs
"""
from sensor_msgs.msg import LaserScan
import rospy
import numpy as np 
from geometry_msgs.msg import Point32, Twist
from std_msgs.msg import Float32

# Initialize distances by obstacle detection zone
d_right=0
d_fright=0
d_front=0
d_fleft=0
d_left=0
d_back=0
d_bleft=0
d_bright=0

# Initialize the position of the obstacles
coords=None 
dists=None

# Initialize the target surface
surfa=0

class Brain:
    def __init__(self):
        # Creates a node called and registers it to the ROS master
        rospy.init_node('brain')
        
        # Suscriber for the characteristics of the target
        self.position_cible= rospy.Subscriber('direction',Point32,self.callback)
        self.surface_cible= rospy.Subscriber('surf',Float32,self.callback2)
        
        # Suscriber for the LiDAR detection
        self.laser = rospy.Subscriber('/scan', LaserScan, self.clbk_laser)
        
        # Publisher for moving instructions
        self.cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    def callback(self,msg):
        # Initialize the command line to control the velocity
        cmd=Twist()
        
        print("surfa : \n", surfa)
        global d_right
        global d_fright
        global d_front
        global d_fleft
        global d_left
        global d_back
        global d_bleft
        global d_bright
        
        # Recover the nearest obstacle that is not in front of the robot
        do=min(d_right,d_left,d_fleft,d_fright,d_back,d_bleft,d_bright,d_front)
        
        # Recover the nearest obstacle 
        d1=min(d_right,d_left,d_fleft,d_fright,d_back,d_bleft,d_bright)
        
        # Initialize the distance considered for an obstacle to interfere with the robot
        d=0.3
        
        print("do :",do)
        
        # If the target is not present (surface too small), stay still
        if(surfa <10000):
            cmd.linear.x=0.0
            cmd.angular.z=0.0
            self.cmd.publish(cmd)
        # If the target is close enough, do not move forward, center the target
        elif(surfa >=50000):
            if(msg.x==1 and msg.y==0 and msg.z==0): # object on the left
                print("objet à gauche \n")
                cmd.linear.x=0.0
                cmd.angular.z=0.3
                self.cmd.publish(cmd)
            elif(msg.x==0 and msg.y==1 and msg.z==0): # object in the center
                print("objet centre \n")
                cmd.linear.x=0.0
                cmd.angular.z=0.0
                self.cmd.publish(cmd)
            elif(msg.x==0 and msg.y==0 and msg.z==1): # object on the right
                print("objet à droite")
                cmd.linear.x=0.0
                cmd.angular.z=-0.7
                self.cmd.publish(cmd)
            elif(msg.x==0 and msg.y==0 and msg.z==0): # no object
                cmd.linear.x=0.0
                cmd.angular.z=0.0
                self.cmd.publish(cmd)
        # If the target is not close enough and there is no obstacle 
        # Or if the target is not close enough and there is an obstacle that is not facing the robot, 
        # Move closer
        elif((surfa > 10000 and surfa <50000 and (do>d))or((d1<d and d_front>d)and(surfa > 10000 and surfa <50000))):
            if(msg.x==1 and msg.y==0 and msg.z==0): # object on the left
                print("objet à gauche \n")
                cmd.linear.x=0.2
                cmd.angular.z=0.7
                self.cmd.publish(cmd)
            elif(msg.x==0 and msg.y==1 and msg.z==0): # object in the center
                cmd.linear.x=0.2
                cmd.angular.z=0.0
                self.cmd.publish(cmd)
            elif(msg.x==0 and msg.y==0 and msg.z==1): # object on the right
                print("objet à droite")
                cmd.linear.x=0.2
                cmd.angular.z=-0.7
                self.cmd.publish(cmd)
            elif(msg.x==0 and msg.y==0 and msg.z==0):# no object
                print("pas d'objet")
                cmd.linear.x=0.0
                cmd.angular.z=-0.0
                self.cmd.publish(cmd)
        # If an obstacle is interfering with the robot and stands in front of it, do not move forward
        # Rotate if there is a target to go around the obstacle
        elif(d1<d and d_front < d):
            print("obstacle gênant : impossible de bouger")
            cmd.linear.x=0.0
            # If there is no target, do not move at all
            if (msg.x==0 and msg.y==0 and msg.z==0 or surfa <10000):
                cmd.angular.z=-0.0
            self.cmd.publish(cmd)            
            
    def clbk_laser(self,msg):
        global coords
        global dists
        coords = []
        dists=[]
        
        # Recover the distance of the obstacles from the robot
        for i, theta in enumerate(np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)):
        # Remove points too close by assigning them a value greater than the distance considered for the obstacle
            if msg.ranges[i] < 0.1:
                dists.append(3.5)
                continue

        # Polar to Cartesian transformation
            coords.append([msg.ranges[i]*np.cos(theta), msg.ranges[i]*np.sin(theta)])
            dists.append(msg.ranges[i])
        
        global d_right
        global d_fright
        global d_front
        global d_fleft
        global d_left
        global d_back
        global d_bleft
        global d_bright
        
        # Positioning of obstacles in one of the eight laser detection fields
        if(min(min(dists[60:120]), 10)>0.1):
            d_left=min(min(dists[60:120]), 3.5)
            print(d_left)
        if(min(min(min(dists[0:30]), 3.5),min(min(dists[330:359]), 3.5))>0.1):
            d_front=min(min(min(dists[0:30]), 3.5),min(min(dists[330:359]), 3.5))
        if(min(min(dists[30:59]), 3.5)>0.1):
            d_fleft=min(min(dists[30:59]), 3.5)
        if(min(min(dists[300:329]), 3.5)>0.1):
            d_fright=min(min(dists[300:329]), 3.5)
        if(min(min(dists[240:299]), 3.5)>0.1):
            d_right=min(min(dists[240:299]), 3.5)
        if(min(min(dists[150:210]), 3.5)>0.1):
            d_back=min(min(dists[150:210]), 3.5)
        if(min(min(dists[120:150]), 3.5)>0.1):
            d_bleft=min(min(dists[120:150]), 3.5)
        if(min(min(dists[210:240]), 3.5)>0.1):
            d_bright=min(min(dists[210:240]), 3.5)
            
    # Recover the target surface        
    def callback2(self,surface):
        global surfa
        print('surface: ',surface.data)
        surfa=surface.data
        
if __name__ == '__main__':
    # Start the node and wait until it receives a message or stopped by Ctrl+C
    node = Brain()
    rospy.spin()