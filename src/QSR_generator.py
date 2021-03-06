#!/usr/bin/env python

import rospy
from qsr.msg import Orange,Hand,Skeleton,Objects,QSR
import numpy as np
import cv2
import math

from sklearn.svm import SVC
from sklearn.preprocessing import StandardScaler
from sklearn.cross_validation import StratifiedKFold
from sklearn.grid_search import GridSearchCV
from sklearn.externals import joblib


th = 20			# thickness of qsr lines
n = []
n_qsr = []	# number of qsrs (n is the number of objects + skeleton joints u have)
window = 30
name_window = 200
A = {}
Names_memory = []
qsr1 = []
clf = joblib.load('/home/omari/catkin_ws/src/learning/svm/QSR.pkl')
qsr_old = []
#-------------------------------------------------------------------------------------#
def features_calc(A,B):
	Z = np.sqrt((A[0]-B[0])**2 + (A[1]-B[1])**2 + (A[2]-B[2])**2)
	#print Z
	#Z = clf.predict([distance,-A[0]+B[0]])
	#if Z==0:
	if Z<.1:
		var = [255,0,0]
		b='touch'
	elif Z<.25:
		var = [0,255,0]
		b='near'
	else:
		var = [0,0,255]
		b='far'
	return (var,b)

#-------------------------------------------------------------------------------------#
def Allen(x1,y1,x2,y2):

	temp = []
	if x1==x2 and y1==y2:
		temp='equal'
	elif x1!=x2 and y1==y2:
		temp='finishes'
	elif x1==x2 and y1!=y2:
		temp='starts'
	elif (x1>x2 and y1<y2) or (x1<x2 and y1>y2):
		temp='during'
	elif (x1<x2 and y1<y2 and y1>x2) or (x2<x1 and y2<y1 and y2>x1):
		temp='overlap'
	elif (x1<x2 and y1==x2 and y2>y1) or (x2<x1 and y2==x1 and y1>y2):
		temp='meets'
	elif (x1<x2 and y1<x2) or (x2<x1 and y2<x1):
		temp='before'
	else:
		temp='problem'

	return temp
#-------------------------------------------------------------------------------------#
def callback_orange(data):
    global Ox,Oy,Oz

    Ox = data.z/1000
    Oy = -data.x/1000
    Oz = data.y/1000

#-------------------------------------------------------------------------------------#
def callback_objects(data):
    global A,Names_memory 
   
    A_ob1 = {}
    Names_memory1 = []	
    for i in range(len(data.z)):
    	Ox = data.z[i]/1000
    	Oy = -data.x[i]/1000
    	Oz = data.y[i]/1000
	if Ox != 0: 	
		A_ob1[i] = [Ox,Oy,Oz]	
    		Names_memory1 = np.append(Names_memory1,i)			# names of joints added to Names

    A = A_ob1
    Names_memory = Names_memory1
#-------------------------------------------------------------------------------------#
def callback_skeleton(data):
    global A

    A[n-4] = data.Head 
    A[n-3] = data.RH
    A[n-2] = data.LH
    A[n-1] = data.RS
    A[n] = data.LS		# list of objects to generate the qsr for

#-------------------------------------------------------------------------------------#
def listener():

    rospy.init_node('Object')
    rospy.Subscriber("Objects_position", Objects, callback_objects)
    #rospy.Subscriber('Skeleton_position', Skeleton, callback_skeleton)
    pub = rospy.Publisher('QSRs', QSR)
    r = rospy.Rate(25) # 10hz
    frame_number = 0
    msg = {}
    msg_counter = 0
    spatial_relations = ['spa0','top','bottom','spa1','spa2']
    names = []
    n_old = 0
    while not rospy.is_shutdown():						# QSR generation
	
	n = len(Names_memory)
	if n > 1:

		n_qsr = np.sum(range(n))

   	    	if n_old!=n:
			n_old = n
			qsr1 = np.zeros(shape=(th*n_qsr, window, 3), dtype=int)+255  	# initilize the QSR matrix
			QSR_vector = np.zeros(shape=(n_qsr, window), dtype=int)  	# initilize the QSR matrix

	    	img = np.zeros((th*n_qsr,window+name_window,3), np.uint8)+255			# initializing the image
	    
	    	# Computing the qsr
	    	qsr1[:,0:window-1,:] = qsr1[:,1:window,:]
		QSR_vector[:,0:window-1] = QSR_vector[:,1:window]
	    	counter = 0
	    	for i in range(n):
		    for j in range(i+1,n):
			var1 = Names_memory[i]
			var2 = Names_memory[j]
		     	qsr1[th*counter:th*(counter+1),window-1,:],Q = features_calc(A[var1],A[var2])[0],features_calc(A[var1],A[var2])[1]

			if Q == 'touch':
				QSR_vector[counter,window-1] = 0
			if Q == 'top':
				QSR_vector[counter,window-1] = 1
			if Q == 'bottom':
				QSR_vector[counter,window-1] = 2
			if Q == 'near':
				QSR_vector[counter,window-1] = 3
			if Q == 'far':
				QSR_vector[counter,window-1] = 4

		     	counter = counter+1

	    	# set the qsr on img

		img[:,name_window:window+name_window,:] = qsr1
		img[:,name_window-5:name_window,:] = np.zeros((th*n_qsr,5,3), np.uint8)

	    	# writing on the image
	    	counter = 0
	    	for i in range(n):
		    for j in range(i+1,n): 
		     	cv2.putText(img,'obj'+str(int(Names_memory[i]))+'-'+'obj'+str(int(Names_memory[j])),(10,(th*(counter+1))-5), cv2.FONT_HERSHEY_SIMPLEX, .5,(0,0,0),1)
		     	counter = counter+1
			
	    	# preparing graph msg
		o1 = []
		o2 = []
		spa = []
		spa_interval = []
	    	counter = 0
	    	for i in range(n-1):
		    for j in range(i+1,n):
		    	o1 = np.append(o1,'obj'+str(int(Names_memory[i])))
		    	o2 = np.append(o2,'obj'+str(int(Names_memory[j])))
			spa = np.append(spa,spatial_relations[QSR_vector[counter,0]])
			l = QSR_vector[counter,0]
			spa_interval = np.append(spa_interval,0)
			for k in range(1,window):
				if l != QSR_vector[counter,k]:
					l = QSR_vector[counter,k]
		    			o1 = np.append(o1,'obj'+str(int(Names_memory[i])))
		    			o2 = np.append(o2,'obj'+str(int(Names_memory[j])))
					spa = np.append(spa,spatial_relations[l])
					spa_interval = np.append(spa_interval,k)
					spa_interval = np.append(spa_interval,k)
		     	counter = counter+1
			spa_interval = np.append(spa_interval,window)

		temporal = []
		for i in range(len(spa)-1):
			for j in range(i+1,len(spa)):
				x1 = spa_interval[i*2]
				y1 = spa_interval[i*2+1]
				x2 = spa_interval[j*2]
				y2 = spa_interval[j*2+1]
				temporal = np.append(temporal,Allen(x1,y1,x2,y2))
				#print x1,y1,x2,y2,temporal
		
		pub.publish(o1,spa,o2,temporal,Names)
		r.sleep()
		frame_number = frame_number + 1
	    	# display the qsr
	    	cv2.imshow('QSR',img)
	    	k = cv2.waitKey(5) & 0xFF

    #rospy.spin()

#-------------------------------------------------------------------------------------#
if __name__ == '__main__':
    # check how many object
    Objects_pointer = open('/home/omari/catkin_ws/src/graphs/src/objects.txt', 'r')
    n = 0
    Names = []
    qsr_old = []
    QSR_vector = []
    for line in Objects_pointer:
        line = line.strip(',\n')
        if line == 'END':
            break
	Names = np.append(Names,'obj'+str(n))			# names of objects to generate the qsr for
	n = n+1								# number of objects
	A[n-1]=[0,0,0]

    #for i in range(n,n+5):							# initialize A2
	#A[i] = [0,0,0]

    #n = n+5								# add 5 joints from skeleton	
    n_qsr = np.sum(range(n))						# number of qsrs (n is the number of objects + skeleton joints u have)
    qsr1 = np.zeros(shape=(th*n_qsr, window, 3), dtype=int)+255  	# initilize the QSR matrix
    QSR_vector = np.zeros(shape=(n_qsr, window), dtype=int)  	# initilize the QSR matrix

    for i in range(n_qsr):
  	qsr_old = np.append(qsr_old,'something')			# names of objects to generate the qsr for
	
    #Names = np.append(Names,['Head','RH','LH','RS','LS'])			# names of joints added to Names

    print('QSR generator running...  '+str(n)+' objects found')
    listener()
