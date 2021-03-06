import numpy as np
from networkx import *
from itertools import permutations

def Grounding_function():
	def grapelet(g,obj1,spatial,obj2):
		g.add_edge(spatial,obj1)
		g.add_edge(spatial,obj2)



	G=Graph()		# the graph structure from QSR

	G1=Graph()		# the graph structure from QSR

	L=Graph()		# the graph structure from words

	#--------------------------------------------------------------------------------------#
	Objects_pointer = open('/home/omari/catkin_ws/src/graphs/src/grounding.txt', 'r')
	A = []
	V1 = []
	V2 = []
	V3 = []

	for line in Objects_pointer:
		line = line.strip(',\n')
		if line == 'END':
		    break
		fields = line.split(',')
		A.append(fields[0])
		V1.append(fields[1])
		V2.append(fields[2])
		V3.append(fields[3])

	#--------------------------------------------------------------------------------------#
	GG = dict()

	for i in range(len(A)):
		G1=Graph()		# the graph structure from QSR
		grapelet(G1,V1[i],V2[i],V3[i])
		grapelet(G,V1[i],V2[i],V3[i])
		GG[i] = G1

	#--------------------------------------------------------------------------------------#


	A_splited = []
	A_index = np.zeros(shape=(len(A), 3), dtype=int)		# 3 because every graph has to has a spatial and 2 objects

	for i in list(A):
		A_splited = np.append(A_splited,i.split())

	words = []
	words_matching = []

	for i in list(A_splited):
		if i in words:
			i
		else:
			words = np.append(words,i)

	print 'the words found in sentenses = ',words
	print 'QSR graph nodes = ',G.nodes()

	if len(words)==G.number_of_nodes():
		print 'same number'

	#print G.nodes()[0]

	c1 = 0
	c2 = 0
	for i in list(A):
		for k in i.split():
			A_index[c1][c2] = np.where(words==k)[0]
			c2 = c2+1
		c1 = c1+1
		c2 = 0

	words_f = []
	words_matching_f = []
	for words_matching in permutations(G.nodes()):
	#for words_matching in permutations(['blue','red','near','green','far']):
			result = []
			for i in range(len(A)):
				grapelet(L,words_matching[A_index[i][0]],words_matching[A_index[i][1]],words_matching[A_index[i][2]])
				B = GG[i]
				result = np.append(result,B.edges()==L.edges())
				result = np.append(result,L.nodes()==B.nodes())
				L.clear()
			if np.sum(result)==len(A)*2:
				print words,'=',words_matching,result,'Matching'
				words_f = words
				words_matching_f = words_matching
			else:
				print words,'=',words_matching,result
	print '***************************************************************************'
	print 'Matching results'
	print '***************************************************************************'
	for i in range(len(words_matching_f)):
		print words_matching_f[i],'==',words_f[i]
	print '***************************************************************************'
	print 'Ground Truth'
	print '***************************************************************************'
	print 'obj0 == blue\n','obj1 == green\n','obj2 == yellow\n','spa0 == touch\n','spa1 == near\n','spa2 == far\n'


