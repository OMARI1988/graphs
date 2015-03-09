#!/usr/bin/env python
# encoding: utf-8

# This code will say hello at the beggining and then wait until u look at the robot and become near to say another msg

"""
Module to connect to a kinect through ROS + OpenNI and access
the skeleton postures.
"""

import time
import roslib
roslib.load_manifest('skeleton_activity')
import rospy
import tf

import numpy as np

from espeak import espeak
import os, sys

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
LAST = rospy.Time(0)


class Kinect:

    def __init__(self, name='kinect_listener', user=1):
        rospy.init_node(name, anonymous=True)
        self.listener = tf.TransformListener()
        self.user = user

    
    def get_posture(self):
        """Returns a list of frames constituted by a translation matrix
        and a rotation matrix.

        Raises IndexError when a frame can't be found (which happens if
        the requested user is not calibrated).
        """
	#dist = np.zeros(1000, dtype=float)
	#rotation = np.zeros(1, dtype=float)
	
	var3 = 1
	LH_trans = [0,0,0]
        while not rospy.is_shutdown():

            try:
                frames = []
                for frame in FRAMES:
                    trans, rot = self.listener.lookupTransform(BASE_FRAME,"/%s_%d" % (frame, self.user), LAST)
                    #if frame == 'head':
                    #    head_trans = trans
                    #if frame == 'neck':
                    #    neck_trans = trans
                    if frame == 'left_hand':
                        LH_trans = trans
		    # reading torso pose and rotation
                    if frame == 'torso':
                        torso_trans = trans
                        torso_rot = rot

			
			#http://answers.ros.org/question/69754/quaternion-transformations-in-python/
			euler = tf.transformations.euler_from_quaternion(torso_rot)
			roll = euler[0]
			pitch = euler[1]
			yaw = euler[2]
			#rotation = np.append(rotation,yaw)
			
			# QSR2 Rotation
			a=LH_trans[1]-torso_trans[1]
			b=LH_trans[0]-torso_trans[0]
			ta = np.round(yaw,2)-np.round(np.arctan2(torso_trans[1],torso_trans[0]),2)+np.round(np.pi/2,2)
			#if ta>np.pi:
			#	ta = ta-np.pi
			#if ta<0:
			ta = ta-np.pi

			ha = np.round(np.arctan2(a,b),2)

			if np.abs(ha)>np.pi/4*3:
				var1 = 'front'
			elif ha<0 and ha>-np.pi/4*3:
				var1 = 'left'
			elif ha>0 and ha<np.pi/4*3:
				var1 = 'right'

			print np.round(ta,1),np.round(ha,1),var1

			
                    #frames.append((trans, rot))
                    #return frames
            except (tf.LookupException,
                    tf.ConnectivityException,
                    tf.ExtrapolationException):
                continue


if __name__ == '__main__':

    kin = Kinect()
    print('TEST1')


# Get values                                                                                                                                         
     
#    for i in range 10:
    print kin.get_posture()
