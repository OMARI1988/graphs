#! /usr/bin/env python2.7

import roslib
roslib.load_manifest('graphs')
import sys
import inspect, os
from optparse import OptionParser
import rospy
import sensor_msgs.msg
from cv_bridge import CvBridge
import cv
import cv2
import numpy as np
import scipy.ndimage.morphology as morphology

from qsr.msg import Objects

global x,y,z,rgb_flag,OBJ


if __name__ == '__main__':	
    global x,y,z,rgb_flag,OBJ
    rgb_flag = 0
    OBJ = {}

    def nothing(x):
	    pass

    pkgdir = roslib.packages.get_pkg_dir("opencv2")
    haarfile = os.path.join(pkgdir, "opencv/share/opencv/haarcascades/haarcascade_frontalface_alt.xml")
    br = CvBridge()	# Create a black image, a window

#--------------------------------------------------------------------------------------#
    def detect_and_draw(imgmsg):
	global x,y,rgb_flag,OBJ

        img = br.imgmsg_to_cv2(imgmsg, desired_encoding="passthrough")
	
    	# Take each frame
    	simg = cv2.GaussianBlur(img,(5,5),0)

    	# Convert BGR to HSV
    	hsv = cv2.cvtColor(simg, cv2.COLOR_BGR2HSV)
    	hsv2 = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	#1
	
    	#lower_blue = np.array([8,144,138])
    	#upper_blue = np.array([21,255,237])

	for i in range(1,len(OBJ)+1):
    		# Threshold the HSV image to get only orange colors
    		mask = cv2.inRange(hsv, np.array(OBJ[i][0:3]), np.array(OBJ[i][3:6]))
	
		# filter and fill the mask
		kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(OBJ[i][6],OBJ[i][6]))
		mask2 = cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernel)

		moments = cv2.moments(mask2) 
		area = moments['m00'] 
		#there can be noise in the video so ignore objects with small areas 
		if(area > 10000): 
	    		#determine the x and y coordinates of the center of the object 
	    		#we are tracking by dividing the 1, 0 and 0, 1 moments by the area 
	    		x1 = int(moments['m10'] / area)
	    		y1 = int(moments['m01'] / area)
	
	    		x[i-1] = x1
	    		y[i-1] = y1
	    		cv2.line(img,(x1,y1-20),(x1,y1+20),(255,50,100),3)
	    		cv2.line(img,(x1-20,y1),(x1+20,y1),(255,50,100),3)
			cv2.putText(img,'obj'+str(i-1),(x1+20,y1+20), cv2.FONT_HERSHEY_SIMPLEX, .5,(0,0,0),1)
	    		rgb_flag = 1
		else:
	    		x[i-1] = 0
	    		y[i-1] = 0
	    		rgb_flag = 1
			

    	cv2.imshow('RGB',img)
    	#cv2.imshow('Mask',mask2)
        k = cv2.waitKey(5) & 0xFF

#--------------------------------------------------------------------------------------#
    def depth_calculation(imgmsg):
	global x,y,z,rgb_flag

        depth = br.imgmsg_to_cv2(imgmsg, desired_encoding="passthrough")
	if rgb_flag == 1:
	    for i in range(len(x)):
		if x[i] != 0 and y[i] != 0:
			z[i] = float(depth[y[i],x[i]])
		if x[i] == 0 and y[i] == 0:
			z[i] = 0
	rgb_flag = 0

    	#cv2.imshow('Depth',depth)

#--------------------------------------------------------------------------------------#
    def talker():
	global x,y,z,OBJ
    	pub = rospy.Publisher('Objects_position', Objects)
    	r = rospy.Rate(25) # 10hz
       	fx = 525.0  # focal length x
	fy = 525.0  # focal length y
	cx = 319.5  # optical center x
	cy = 239.5  # optical center y
	
    	while not rospy.is_shutdown():
		x_world = (x - cx) * z / fx
		y_world = (y - cy) * z / fy
		#print np.float64(x_world),np.float64(y_world),np.float64(z)
        	pub.publish(x_world,y_world,z)
        	r.sleep()

#--------------------------------------------------------------------------------------#
    directory = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
    Objects_pointer = open(directory+'objects.txt', 'r')
    OBJ = {}
    for line in Objects_pointer:
        line = line.strip(',\n')
        if line == 'END':
            break
        fields = line.split(',')
        fields = map(int, fields)
	OBJ[fields[0]] = fields[1:len(fields)]

    x = np.zeros(shape=(len(OBJ)), dtype=int)
    y = np.zeros(shape=(len(OBJ)), dtype=int)
    z = np.zeros(shape=(len(OBJ)), dtype=float)

    rospy.init_node('rosColordetect')
    image_topic = rospy.resolve_name("/cameras/right_hand_camera/image") 
    depth_topic = rospy.resolve_name("/camera/depth_registered/image_raw")      

    rospy.Subscriber(image_topic, sensor_msgs.msg.Image, detect_and_draw)
    rospy.Subscriber(depth_topic, sensor_msgs.msg.Image, depth_calculation)
    print('Object tracker running...  '+str(len(OBJ))+' objects found')
    talker()

    rospy.spin()


