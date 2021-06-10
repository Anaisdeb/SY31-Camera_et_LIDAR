#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Created on Thu Jun  3 15:04:35 2021

@author: merwane
"""
from sensor_msgs.msg import LaserScan
import rospy
from geometry_msgs.msg import Point32, Twist
from std_msgs.msg import Float32


d_right=0
d_fright=0
d_front=0
d_fleft=0
d_left=0
d_back=0
d_bleft=0
d_bright=0



pub = None
surfa=0
class Brain:
    def __init__(self):
        #global pub
        # Creates a node called and registers it to the ROS master
        rospy.init_node('brain')
        
        
        self.position_cible= rospy.Subscriber('direction',Point32,self.callback)
        self.surface_cible= rospy.Subscriber('surf',Float32,self.callback2)
        #self.present=rospy.Subscriber('present',Float32,self.callback3)
        self.cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.laser = rospy.Subscriber('/scan', LaserScan, self.clbk_laser)
        
    
    def callback(self,msg):
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
        do=min(d_right,d_left,d_fleft,d_fright,d_back,d_bleft,d_bright)
        d=1.0
        print("do :",do)
        
        #print('direction :\n ',msg)
        if(surfa >=50000 and do>d):
            if(msg.x==1 and msg.y==0 and msg.z==0):#objet à gauche
                print("objet à gauche \n")
                cmd.linear.x=0.0
                cmd.angular.z=0.3
                self.cmd.publish(cmd)
            elif(msg.x==0 and msg.y==1 and msg.z==0):#objet centre
                print("objet centre \n")
                cmd.linear.x=0.0
                cmd.angular.z=0.0
                self.cmd.publish(cmd)
            elif(msg.x==0 and msg.y==0 and msg.z==1):#objet à droite
                print("objet à droite")
                cmd.linear.x=0.0
                cmd.angular.z=-0.3
                self.cmd.publish(cmd)
            elif(msg.x==0 and msg.y==0 and msg.z==0):#pas d'objet
                cmd.linear.x=0.0
                cmd.angular.z=0.0
                self.cmd.publish(cmd)
        
        elif(surfa <50000 and do>d):
            if(msg.x==1 and msg.y==0 and msg.z==0):#objet à gauche
                print("objet à gauche \n")
                cmd.linear.x=0.2
                cmd.angular.z=-0.3
                self.cmd.publish(cmd)
            elif(msg.x==0 and msg.y==1 and msg.z==0):#objet centre
                print("objet centre \n")
                cmd.linear.x=0.2
                cmd.angular.z=0.0
                self.cmd.publish(cmd)
            elif(msg.x==0 and msg.y==0 and msg.z==1):#objet à droite
                print("objet à droite")
                cmd.linear.x=0.2
                cmd.angular.z=0.3
                self.cmd.publish(cmd)
            elif(msg.x==0 and msg.y==0 and msg.z==0):#pas d'objet
                print("objet à droite")
                cmd.linear.x=0.0
                cmd.angular.z=-0.0
                self.cmd.publish(cmd)
        elif(do>d):
            print("obstacle gene impossible de bouger")
            cmd.linear.x=0.0
            cmd.angular.z=-0.0
            self.cmd.publish(cmd)
            
            
    def clbk_laser(self,msg):
        global d_right
        global d_fright
        global d_front
        global d_fleft
        global d_left
        global d_back
        global d_bleft
        global d_bright
        
        if(min(min(msg.ranges[60:120]), 10)>0.1):
            d_left=min(min(msg.ranges[60:120]), 3.5)
            print(d_left)
        if(min(min(min(msg.ranges[0:30]), 3.5),min(min(msg.ranges[330:359]), 3.5))>0.1):
            d_front=min(min(min(msg.ranges[0:30]), 3.5),min(min(msg.ranges[330:359]), 3.5))
        if(min(min(msg.ranges[30:59]), 3.5)>0.1):
            d_fleft=min(min(msg.ranges[30:59]), 3.5)
        if(min(min(msg.ranges[300:329]), 3.5)>0.1):
            d_fright=min(min(msg.ranges[300:329]), 3.5)
        if(min(min(msg.ranges[240:299]), 3.5)>0.1):
            d_right=min(min(msg.ranges[240:299]), 3.5)
        if(min(min(msg.ranges[150:210]), 3.5)>0.1):
            d_back=min(min(msg.ranges[150:210]), 3.5)
        if(min(min(msg.ranges[120:150]), 3.5)>0.1):
            d_bleft=min(min(msg.ranges[120:150]), 3.5)
        if(min(min(msg.ranges[210:240]), 3.5)>0.1):
            d_bright=min(min(msg.ranges[210:240]), 3.5)
            
            
            
    def callback2(self,surface):
        global surfa
        print('surface: ',surface.data)
        surfa=surface.data
    '''def callback3(self,present):
        print("present:",present.data)'''
if __name__ == '__main__':
    # Start the node and wait until it receives a message or stopped by Ctrl+C
    node = Brain()
    rospy.spin()
        