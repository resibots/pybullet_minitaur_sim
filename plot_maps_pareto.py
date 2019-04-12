#! /usr/bin/env python
#| This file is a part of the pyite framework.
#| Copyright 2019, INRIA
#| Main contributor(s):
#| Jean-Baptiste Mouret, jean-baptiste.mouret@inria.fr
#| Eloïse Dalin , eloise.dalin@inria.fr
#| Pierre Desreumaux , pierre.desreumaux@inria.fr
#|
#| Antoine Cully, Jeff Clune, Danesh Tarapore, and Jean-Baptiste Mouret.
#|"Robots that can adapt like animals." Nature 521, no. 7553 (2015): 503-507.
#|
#| This software is governed by the CeCILL license under French law
#| and abiding by the rules of distribution of free software.  You
#| can use, modify and/ or redistribute the software under the terms
#| of the CeCILL license as circulated by CEA, CNRS and INRIA at the
#| following URL "http://www.cecill.info".
#|
#| As a counterpart to the access to the source code and rights to
#| copy, modify and redistribute granted by the license, users are
#| provided only with a limited warranty and the software's author,
#| the holder of the economic rights, and the successive licensors
#| have only limited liability.
#|
#| In this respect, the user's attention is drawn to the risks
#| associated with loading, using, modifying and/or developing or
#| reproducing the software by the user in light of its specific
#| status of free software, that may mean that it is complicated to
#| manipulate, and that also therefore means that it is reserved for
#| developers and experienced professionals having in-depth computer
#| knowledge. Users are therefore encouraged to load and test the
#| software's suitability as regards their requirements in conditions
#| enabling the security of their systems and/or data to be ensured
#| and, more generally, to use and operate it in the same conditions
#| as regards security.
#|
#| The fact that you are presently reading this means that you have
#| had knowledge of the CeCILL license and that you accept its terms.
import pybullet as p
import pybullet_data
import os
import time
import math
from timeit import default_timer as timer
import time
from pymap_elites.map_elites import *
from pycontrollers.controller import Controller
import numpy as np
from pylab import *
from pybullet_envs.minitaur.envs import minitaur_derpy
import matplotlib.pyplot as plt

from simulator_map_elites import *


def eval_minitaur2(ctrl):
	simu = MinitaurSimulator(gui=True)
	ctrl2 = ctrl[:-1]
	id = ctrl[-1]
	controller = Controller(ctrl2,1000)
	for i in range(0, 20000):
		error = simu.step(controller)
		if(error):
			desc = [0,0,0,0]
			dist = -1000, np.array(desc), id
			simu.destroyed()
			return dist
		if(not simu.inside_corridor):
			desc = [0,0,0,0]
			dist = -1000, np.array(desc), id
			simu.destroyed()
			return dist
	if(not error):
		if(max(np.mean(simu.currents,axis=0))>16):
			desc = [0,0,0,0]
			dist = -1000, np.array(desc), id
			simu.destroyed()
			return dist
		if(np.sum(np.abs(np.mean(simu.currents,axis=0)))>60):
			desc = [0,0,0,0]
			dist = -1000, np.array(desc), id
			simu.destroyed()
			return dist
		keys = list(simu.descriptor.keys())
		desc=[]
		for k in keys:
			cns = simu.descriptor[k]
			d = round(sum(cns)/len(cns)*100.0)/100.0
			desc.append(d)
	simu.compute_final_reward()
	dist = simu.reward, np.array(desc), id
	simu.destroyed()
	return dist

if __name__ == "__main__":
	import multiprocessing
	num_cores = multiprocessing.cpu_count()
	pool = multiprocessing.Pool(num_cores)

	dist_filter = 1
	torque_filter = 0.7
	dump_period  = 50

	#### LOAD MAP ####################################
	centroids = load_centroids(sys.argv[1])
	dim_x = 24
	fit, beh, x = load_data(sys.argv[2], centroids.shape[1],dim_x)
	index = np.argmax(fit)


	#####################################################
	X = []
	Y = [f[0] for f in fit]
	X_filtered = []
	Y_filtered = []
	filtered_ids = []
	for c in range(0,len(beh)):
		X.append(max(beh[c]))
		if(max(beh[c])<torque_filter):
			if(fit[c] > dist_filter):
				X_filtered.append(max(beh[c]))
				Y_filtered.append(fit[c][0])
				filtered_ids.append(c)

	def pareto_frontier(Xs, Ys, ids, maxX = True, maxY = True):
		myList = sorted([[Xs[i], Ys[i], ids[i]] for i in range(len(Xs))], reverse=maxX)
		p_front = [myList[0]]
		for pair in myList[1:]:
			if maxY:
				if (pair[1]) >= (p_front[-1][1]):
					p_front.append(pair)
			else:
				if (pair[1]) <= p_front[-1][1]:
					p_front.append(pair)
		p_frontX = [pair[0] for pair in p_front]
		p_frontY = [pair[1] for pair in p_front]
		p_ids = [pair[2] for pair in p_front]
		return p_frontX, p_frontY, p_ids

	X_pareto_final = []
	Y_pareto_final = []
	ids_pareto_final = []
	Xcp = copy(X)
	Ycp = copy(Y)
	ids_cp = range(0,len(Xcp))
	while(len(X_pareto_final)< 100):
		X_pareto, Y_pareto, p_ids = pareto_frontier(Xcp,Ycp,range(0,len(Xcp)), False,True)
		X_pareto_final.extend(X_pareto)
		Y_pareto_final.extend(Y_pareto)
		Xcp = np.delete(Xcp,p_ids)
		Ycp = np.delete(Ycp,p_ids)
	fig = figure()
	ax = fig.add_subplot(111)
	ax.plot(X, Y,'+')
	ax.plot(X_filtered, Y_filtered,'r+')
	ax.plot(X_pareto_final, Y_pareto_final,'+',color='black')
	plt.legend()
	fig.savefig('oxo2.pdf')
	plt.show()
	for o in range(0,len(Y_pareto_final)):
		for e in range(0,len(Y)):
			if(Y_pareto_final[o] == Y[e]):
				ids_pareto_final.append(e)
	if(len(ids_pareto_final)!= len(Y_pareto_final)):
		print("Warnin ypareto not unique ")
		ids_pareto_final=[]
	# X_pareto, Y_pareto, p_ids = pareto_frontier(X,Y,range(0,len(X)), False,True)

	# for id in p_ids:
	# 	ctrl = x[id]
	# 	ctrl = np.append(ctrl, id)
	# 	dist = eval_minitaur2(ctrl)
	# 	print(dist)


	# ids = np.argsort(fit,axis=0)
	# ids_max = []
	# for i in ids:
	# 	if(fit[i] > dist_filter):
	# 		ids_max.append(i)
	# print("Number of behaviors with traveled dist >",dist_filter , " : ", len(ids_max))
	#
	# ctrls_to_test = []
	# saved_ids = []
	# saved_fit = []
	# saved_desc = []
	# count = 0
	# for i in range(0,len(ids_max)):
	# 	tmp = x[ids_max[i][0]]
	# 	tmp = np.append(tmp ,ids_max[i][0])
	# 	ctrls_to_test.append(tmp)
	# 	count = count +1
	# 	if(count%dump_period==0):
	# 		print("Evaluating first ",len(ctrls_to_test) ,"ctrls ...")
	# 		results = pool.map(eval_minitaur2,ctrls_to_test)
	# 		print("Results : ")
	# 		for r in results:
	# 			print(r)
	# 			if(r[0]>-1000):
	# 				saved_fit.append(r[0])
	# 				saved_desc.append(r[1])
	# 				saved_ids.append(r[2])
	# 		print("Total number of good behaviors : ", len(saved_ids))
	# 		np.save('saved_ids',np.array(saved_ids))
	# 		np.save('saved_fit',np.array(saved_fit))
	# 		np.save('saved_desc',np.array(saved_desc))
	# 		ctrls_to_test = []
	#
	# print("Total number of good behaviors : ", len(saved_ids))
	# np.save('saved_ids',np.array(saved_ids))
	# np.save('saved_fit',np.array(saved_fit))
	# np.save('saved_desc',np.array(saved_desc))
