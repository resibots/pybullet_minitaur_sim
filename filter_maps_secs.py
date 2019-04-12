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
	simu = MinitaurSimulator(gui=False)
	ctrl2 = ctrl[:-1]
	id = ctrl[-1]
	controller = Controller(ctrl2,1000)
	for i in range(0, 20000):
		error = simu.step(controller)
		if(error):
			desc = [0,0,0,0]
			dist = -1000, np.array(desc), id, i
			simu.destroyed()
			return dist
		if(not simu.inside_corridor):
			keys = list(simu.descriptor.keys())
			desc=[]
			for k in keys:
				cns = simu.descriptor[k]
				d = round(sum(cns)/len(cns)*100.0)/100.0
				desc.append(d)
			simu.compute_final_reward()
			dist = simu.reward, np.array(desc), id, i
			simu.destroyed()
			return dist
	if(not error):
		# if(max(np.mean(simu.currents,axis=0))>16):
		# 	desc = [0,0,0,0]
		# 	dist = -1000, np.array(desc), id
		# 	simu.destroyed()
		# 	return dist, i
		# if(np.sum(np.abs(np.mean(simu.currents,axis=0)))>60):
		# 	desc = [0,0,0,0]
		# 	dist = -1000, np.array(desc), id
		# 	simu.destroyed()
		# 	return dist, i
		keys = list(simu.descriptor.keys())
		desc=[]
		for k in keys:
			cns = simu.descriptor[k]
			d = round(sum(cns)/len(cns)*100.0)/100.0
			desc.append(d)
	simu.compute_final_reward()
	dist = simu.reward, np.array(desc), id, i
	simu.destroyed()
	return dist

if __name__ == "__main__":
	import multiprocessing
	num_cores = multiprocessing.cpu_count()
	pool = multiprocessing.Pool(num_cores-3)
	dist_filters = [1,2,3,4]
	dist_filters = [1]
	for dist_filter in dist_filters:
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

	# ctrls_to_test=[]
	# for i in range(0,len(ids_max)):
	# 	tmp = x[ids_max[i][0]]
	# 	tmp = np.append(tmp ,ids_max[i][0])
	# 	ctrls_to_test.append(tmp)
		# count = count +1

	# for i in range(0,10):
	# 	index = np.random.randint(0,len(ctrls_to_test))
	# 	dist = eval_minitaur2(ctrls_to_test[index])
	# 	print(dist)
	#
	# ids = np.argsort(fit,axis=0)
	# ids_max = []
	# for i in ids:
	# 	if(fit[i] > dist_filter):
	# 		ids_max.append(i)
	# print("Number of behaviors with traveled dist >",dist_filter , " : ", len(ids_max))

	# ctrls_to_test = []
	# saved_ids = []
	# saved_fit = []
	# saved_desc = []
	# saved_secs = []
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
	# 				saved_secs.append(r[3])
	# 		print("Total number of good behaviors : ", len(saved_ids))
	# 		np.save('saved_ids',np.array(saved_ids))
	# 		np.save('saved_fit',np.array(saved_fit))
	# 		np.save('saved_desc',np.array(saved_desc))
	# 		np.save('saved_secs',np.array(saved_secs))
	# 		ctrls_to_test = []
	#
	# print("Total number of good behaviors : ", len(saved_ids))
	# np.save('saved_ids',np.array(saved_ids))
	# np.save('saved_fit',np.array(saved_fit))
	# np.save('saved_desc',np.array(saved_desc))
	# np.save('saved_secs',np.array(saved_secs))