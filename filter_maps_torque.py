#! /usr/bin/env python
#| This file is a part of the pyite framework.
#| Copyright 2019, INRIA
#| Main contributor(s):
#| Jean-Baptiste Mouret, jean-baptiste.mouret@inria.fr
#| Eloise Dalin , eloise.dalin@inria.fr
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
#from pybullet_envs.minitaur.envs import minitaur
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

	#### RECOVER BEST DIST BEAHVIORS #################
	saved_ids = []
	saved_fit = []
	saved_desc = []
	ids = np.argsort(fit,axis=0)
	ids_max = []
	for i in ids:
		if(fit[i] > dist_filter):
			ids_max.append(i)
			saved_fit.append(fit[i])
			saved_desc.append(beh[i])
			saved_ids.append(i)
	print("Number of behaviors with traveled dist >",dist_filter , " : ", len(ids_max))
	np.save('saved_ids',np.array(saved_ids))
	np.save('saved_fit',np.array(saved_fit))
	np.save('saved_desc',np.array(saved_desc))

	saved_ids_filtered = []
	saved_fit_filtered  = []
	saved_desc_filtered  = []
	X = [[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[]]
	for id in saved_ids:
		ctrl = beh[id[0]]
		nope = False
		appendd = False
		for c in range(0,len(ctrl)):
			if(ctrl[c] > torque_filter):
				nope = True
		for c in range(0,len(ctrl)):
			if(nope == False):
				X[c].append(ctrl[c])
				appendd = True
		if(appendd):
			saved_ids_filtered.append(id)
			saved_fit_filtered.append(fit[id])
			saved_desc_filtered.append(beh[id])
			np.save('saved_ids_filtered',np.array(saved_ids_filtered))
			np.save('saved_fit_filtered',np.array(saved_fit_filtered))
			np.save('saved_desc_filtered',np.array(saved_desc_filtered))

	print("torque filtered : ", len(X[0]))
	fig = figure()
	ax = fig.add_subplot(111)
	for i in range(0, 16):
		ax.plot(np.arange(0, len(X[i])), np.sort(X[i]),label='torque_'+str(i))
	plt.legend()
	fig.savefig('oxo.pdf')
	plt.show()
	# for k in range(0,10):
	# 	index = np.random.randint(0,len(saved_ids))
	# 	ctrl = x[saved_ids[index][0]]
	# 	# ctrl.append(saved_ids[index])
	# 	ctrl = np.append(ctrl, saved_ids[index][0])
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
