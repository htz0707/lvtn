#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from math import *
import roslib
import sys
import rospy
import cv2
import numpy as np
import message_filters
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class controller:

    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('run2', anonymous=True)
	self.bridge = CvBridge()
        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/drone2/mavros/setpoint_velocity/cmd_vel',TwistStamped, queue_size=10)
	self.local_publisher = rospy.Publisher('/drone2/mavros/setpoint_position/local',PoseStamped, queue_size=10)

        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('/drone2/mavros/local_position/velocity_local',TwistStamped, self.update_pose)
	self.image_sub = rospy.Subscriber("/webcam/image_raw", Image, self.update_callback)

        self.pose = TwistStamped()
        self.rate = rospy.Rate(10)
	self.cX0 = 0
	self.cY0 = 0
	self.area0 = 0
	self.fx0 = 0
	self.fy0 = 0
	self.fz0 = 0
	self.fw0 = 0
	self.start = 0
	self.preArea = 0
	self.countx = 0
	self.county = 0
	self.countz = 0
	self.countn = 0
	self.width = 0
	self.height = 0
	self.Arealist = [2460.0, 1.0, 1.0]
	self.countArea = 0

    def update_callback(self, rgb_data):
	
	try:
            img = self.bridge.imgmsg_to_cv2(rgb_data, "bgr8")
      	    

      	    self.width = int(img.shape[1])
     	    self.height = int(img.shape[0])
	    #img = cv2.resize(imgs, (self.width, self.height))
	    imgHsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
      	    lower = np.array([0,0,18])
      	    upper = np.array([179,255,67])
	    mask = cv2.inRange(imgHsv,lower,upper)
	    mask = cv2.erode(mask, None, iterations=2)
    	    mask = cv2.dilate(mask, None, iterations=2)
 	    contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
	    
	    
	    if len(contours)==0:
		print ("000000")
		if(((self.fx0!=0) or(self.fy0!=0) or (self.fz0!=0))and(self.countn < 4)):
		    self.move2goal(self.fx0,self.fy0,self.fz0,self.fw0)
		    self.countn = self.countn + 1
		    print ("////////")
		else:
		    self.countn = 0
		    self.move2goal(0,0,0,2)  
		    print ("======")
            else:
	    	c = max(contours, key = cv2.contourArea)
		if cv2.contourArea(c) > 200 :   
       	    	    x,y,w,h = cv2.boundingRect(c)
	    	    M = cv2.moments(c)
		    fx=fy=fz=fw = 0
	    	    cX = int(M["m10"] / M["m00"])
	    	    cY = int(M["m01"] / M["m00"])
		    area= cv2.contourArea(c)
	    	    print (area)
 	    	    #print (self.countx)
 	    	    print (self.countArea)
 	    	    print (0.5*self.height)
	    	    print (cY)
 	    	    print (0.5*self.width)

		    
		    fy = self.goY(self.area0, area, self.fy0)
		    fx = self.goX(self.cX0, cX, self.fx0, self.area0, area)
		    #fw = self.goW(self.cX0, cX, self.fw0)
		    fz = self.goZ(self.cY0, cY, self.fz0)
		    
		    self.PreArea = self.area0
		    self.fx0 = fx
		    self.fy0 = fy
		    self.fz0 = fz
		    #self.fw0 = fw
		    self.move2goal(fx,fy,fz,fw)
		    self.cX0 = cX
		    self.cY0 = cY
		    self.area0 = area
		    self.start = rospy.get_time()
	    	    cv2.circle(img, (cX, cY), 2, (255, 255, 255), -1)
            	    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255,0), 2)

	except CvBridgeError as e:
      	    print(e)
    	cv2.imshow("ball detection", img)
    	cv2.waitKey(1)

	  
    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose = data
    def goY(self, dy0, dy, vy0):
	if(dy > 3600):
	    vy = -1
	elif(dy > 2200):
	    vy = dy/5000.0
	    vy = -self.compareVel(abs(vy))
	elif(dy > 1200):
	    vy = 0
	    print("======gan======")
	elif(dy >= 600):
	    self.countArea = 0
	    vy = abs(dy-dy0)/1000.0
	    vy = self.compareVel(abs(vy))
	    print("=====xa======")
	else:
	    vy = 1
	return vy

    def goX(self, dx0, dx, vx0, ds0, ds):
	if((dx > 0.49*self.width)and(dx < 0.51*self.width)):
	    vx = 0
	    print("======---------======")
	else:
	    if(dx>0.53*self.width):
		vx = 0.01*(dx-0.5*self.width)
		vx = self.compareVel(abs(vx))
		print("=====->->->->======")
	    elif(dx<0.47*self.width):
		vx = 0.01*(0.5*self.width-dx)
		vx = -self.compareVel(abs(vx))
		print("=====<-<-<-<-======")
	    else:
		vx = vx0/1.05
	    
	    self.countx = self.countx + 1

	return vx

    def goW(self, dx0, dx, vw0):
	if((dx > 0.49*self.width)and(dx < 0.51*self.width)):
	    vw = 0
	    print("======---------======")
	else:
	    if(dx>0.52*self.width):
		vw = -0.02*(dx-0.5*self.width)
		vw = -self.compareVel(abs(vw))
		print("=====>>>>======")
	    elif(dx<0.48*self.width):
		vw = 0.02*(0.5*self.width-dx)
		vw = self.compareVel(abs(vw))
		print("=====<<<<======")
	    else:
		vw = vw0/1.05
	    
	    #self.countx = self.countx + 1
	#if(self.countx>3):
	    #vx = 2*vx
	    #self.countx=0
	return vw

    def goZ(self, dz0, dz, vz0):
	if((dz > 0.49*self.height)and(dz < 0.51*self.height)):
	    vz = 0
	    print("======||||||||-======")
	else:
	    if(dz > 0.53*self.height):
		vz = -0.02*(dz-0.5*self.height)
		vz = -self.compareVel(abs(vz))
		print("=====down======")
	    elif(dz < 0.47*self.height):
		vz = 0.02*(0.5*self.height-dz)
		vz = self.compareVel(abs(vz))
		print("=====up======")
	    else:
		vz = vz0/1.05
	
	    self.countz = self.countz + 1
	#if(self.countz>3):
	    #vz = 2*vz
	    #self.countz=0
	return vz
    def compareVel(self, Dv):
	if(Dv>0.5):
	    vp = 0.5
	    return vp
	else: 
	    return Dv

    def move2goal(self,flagx,flagy,flagz,flagw):
        """Moves the turtle to the goal."""
	count = 1
	vel_msg = TwistStamped()

	vel_msg.twist.linear.z = flagz*1.0
	vel_msg.twist.linear.x = flagx*1.0
	vel_msg.twist.linear.y = flagy*1.0
	vel_msg.twist.angular.z = flagw*1.0
        
		    
	self.velocity_publisher.publish(vel_msg)
	print "x: "
	print self.pose.twist.linear.x
	print "y: "
	print self.pose.twist.linear.y
	print "z: "
	print self.pose.twist.linear.z
	print "angular.z: "
	print self.pose.twist.angular.z
	print "-------------"
	    

        count=count+1
        self.rate.sleep()

        
    def loop(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        x = controller()
        x.loop()
    except rospy.ROSInterruptException:
        pass
