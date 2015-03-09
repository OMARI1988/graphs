#!/usr/bin/env python
# encoding: utf-8

# This code will check the distance between the hand, head and the shoulder and compute QSRs for a short window of time
# problem with this code is that the window size not the same as the actual frame rate (FIXED !!!!)

"""
Module to connect to a kinect through ROS + OpenNI and access
the skeleton postures.
"""
import rospy
import tf

import numpy as np

from espeak import espeak
import os, sys

from qsr.msg import Skeleton

global x,y,z
x = 0
y = 0
z = 0

BASE_FRAME = '/openni_depth_frame'
FRAMES = [
        'head',
        'neck',
        'torso',
        'left_shoulder',
        'left_elbow',
        'left_hand',
        'left_hip',
        'left_knee',
        'left_foot',
        'right_shoulder',
        'right_elbow',
        'right_hand',
        'right_hip',
        'right_knee',
        'right_foot',
        ]
#LAST = rospy.Time(0)
LAST = rospy.Duration()


class Kinect:

    def __init__(self, name='kinect_listener', user=1):
        rospy.init_node(name, anonymous=True)
        self.listener = tf.TransformListener()
        self.user = user

    
    def get_posture(self):
        
	def qsr(A,B):
		C = np.sqrt((A[0]-B[0])**2 + (A[1]-B[1])**2 + (A[2]-B[2])**2)
		if C<=.2:
			var = [0,0,1]
		elif C>.2 and C<.5:
			var = [0,1,0]
		else:
			var = [1,0,0]
		return var
	
	head_trans = [0,0,0]
	RS_trans = [0,0,0]
	RE_trans = [0,0,0]
	RH_trans = [0,0,0]
	LS_trans = [0,0,0]
	LE_trans = [0,0,0]
	LH_trans = [0,0,0]
	torso_trans = [0,0,0]

	flag = 0

	window = 150
	qsr1 = np.zeros(shape=(window, 3), dtype=int)
	qsr2 = np.zeros(shape=(window, 3), dtype=int)
	qsr3 = np.zeros(shape=(window, 3), dtype=int)
	qsr4 = np.zeros(shape=(window, 3), dtype=int)
	qsr5 = np.zeros(shape=(window, 3), dtype=int)
	qsr6 = np.zeros(shape=(window, 3), dtype=int)

	pub = rospy.Publisher('Skeleton_position', Skeleton)

        while not rospy.is_shutdown():

            try:
                frames = []
               	for frame in FRAMES:
                    trans, rot = self.listener.lookupTransform(BASE_FRAME,"/%s_%d" % (frame, self.user), LAST)

                    if frame == 'head':
                        head_trans = trans
                    if frame == 'torso':
                        torso_trans = trans
                    if frame == 'left_hand':
                        LH_trans = trans
                    if frame == 'left_elbow':
                        LE_trans = trans
                    if frame == 'left_shoulder':
                        LS_trans = trans
                    if frame == 'right_shoulder':
                        RS_trans = trans
                    if frame == 'right_elbow':
                        RE_trans = trans
                    if frame == 'right_hand':
                        RH_trans = trans

		    ss = rospy.get_time()
		    #print ss,abs(flag - ss)>.04

		    if abs(flag - ss)>.04:
			flag = ss
			pub.publish(head_trans,RH_trans,LH_trans,RS_trans,LS_trans)
			
			
 
            except (tf.LookupException,
                    tf.ConnectivityException,
                    tf.ExtrapolationException):
                continue


if __name__ == '__main__':

    kin = Kinect()
    print('Skeleton Tracker running...')
    kin.get_posture()
