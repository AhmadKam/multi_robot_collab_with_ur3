#!/usr/bin/env python

#Libraries and stuff
import rospy
import cv2
from geometry_msgs.msg import Point
from std_msgs.msg import Int8
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from collections import deque

from imutils.video import VideoStream
import imutils
import numpy as np

import time


#define variables
colourLower = (0,0,0)
colourUpper = (0,0,0)

#this is the publisher definition for center pixel
pub = rospy.Publisher('BalloonCentre', Point, queue_size=1)
rospy.init_node('OpenCV', disable_signals=True)
global centre
centre = Point()
global oldZ
oldZ = 1400

#definition of callbacks

#Callback for the Image Subscriber
def callback(data):
	
	global centre

	#Convert the Recieved image to Mat (OpenCV Image Type)
	cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
	frame = cv_image
	blurred = cv2.GaussianBlur(frame, (11, 11), 0)
	hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
	#create a mask 	
	mask = cv2.inRange(hsv, colourLower, colourUpper)
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)
	# find contours in the mask and initialize the current (x, y) center of the ball
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)
	center = None
	# only proceed if at least one contour was found
	if len(cnts) > 0:
		# find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and
		# centroid
		c = max(cnts, key=cv2.contourArea)
		((x, y), radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
	
	centre.x = center[0]
	centre.y = center[1]

#Callback for the Colour Check Subscriber
def callback2(data):
	global colourLower
	global colourUpper
	if data.data == 0:
		colourLower = (100, 86, 6)
		colourUpper = (125, 255, 255)
	elif data.data == 1:
		colourLower = (29, 86, 6)
		colourUpper = (64, 255, 255)

#Callback for the Depth Check Subscriber
def callbackDepth(data):
	global centre
	global oldZ
	cv_image = bridge.imgmsg_to_cv2(data)
	centre.z = cv_image[centre.x,centre.y]
	tempZ = centre.z
	if tempZ>2000:
		centre.z = tempZ
	tempZ = centre.z
	tempX = centre.x
	while centre.z == 0:
		tempX = tempX+1
		centre.z = cv_image[tempX,centre.y]


#Initialise the subscribers and converter
bridge = CvBridge()
image_sub = rospy.Subscriber("/kinect2/sd/image_color_rect",Image,callback, queue_size=1)
colour_sub = rospy.Subscriber("Colour", Int8, callback2, queue_size=1)

#initialise the Depth Sub
depth_sub = rospy.Subscriber("/kinect2/sd/image_depth_rect",Image,callbackDepth, queue_size=1)


#main loop
while True:
	pub.publish(centre)
	rate.sleep()
