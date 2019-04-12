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
from simulator_map_elites import *
import sys

if __name__ == "__main__":
	plot = False

	aX = 0.02
	aY = 0.03
	d = 0.5
	ctrl_sync_diagonal = [aX, 0.1, d, aY, 0.0, d,aX, 0.6, d,aY, 0.5, d,aX, 0.6, d,aY, 0.5, d,aX,0.1,d,aY, 0.0, d]

	d = 0.2
	aX = 0.02
	aY = 0.05
	#Same phase along Y will lead to a forward sync
	#Same phase along X will lead to a up/down sync
	#ctrl = ["front_left", "back_left", "front_right", "back_right"]
	ctrl_back_front = [aX, 1, d,aY, 0.0, d, aX, 0.1, d, aY, 0.0, d, aX, 1, d,aY, 0.0, d,aX,0.1,d,aY, 0.0, d]
	ctrl = ctrl_back_front

	aX = 0.02
	aY = 0.04
	d = 0.6
	ctrl_sync_diagonal_jump = [aX, 0.0, d, aY, 1 , d,aX, 0.0, d,aY, 0.1, d,aX, 0.0, d,aY, 0.1, d,aX,0.0,d,aY, 1, d]

	aX = 0.02
	aY = 0.03
	d = 0.5
	ctrl_sync_diagonal_2 = [aX, 0.1, d, aY, 0.0, d,aX, 1, d,aY, 0.5, d,aX, 1, d,aY, 0.5, d,aX,0.1,d,aY, 0.0, d]
	ctrl = ctrl_sync_diagonal_2

	aX = 0.02
	aY = 0.05
	b = 0.9
	ctrl_gallop1 = [aX, b+0.01, d, aY, b-0.01, d,aX, b+0.1, d,aY, b, d,aX, b+0.1, d,aY, b, d,aX,0.1,d,aY, 0.0, d]
	ctrl = ctrl_gallop1

	aX = 0.03
	aY = 0.01
	d = 0.1
	ctrl_jump = [aX, 0.1, d, aY,0.0, d,aX, 0.1, d,aY, 0.0, d,aX, 0.1, d,aY, 0.0, d,aX,0.1,d,aY, 0.0, d]
	ctrl = ctrl_jump

	# aX = 0.02
	# aY = 0.03
	# d = 0.1
	# ctrl = [aX, 0.1, d, aY, 0.0, d,aX, 0.6, d,aY, 0.5, d,aX, 0.6, d,aY, 0.5, d,aX,0.1,d,aY, 0.0, d]
	# ctrl = [aX, 0.1, d, aY, 0.0, d,aX, 0.2, d,aY, 0.1, d,aX,0.1,d,aY, 0.0, d, aX, 0.2, d,aY, 0.1, d]

	if(plot):
		# aX = 0.02
		# aY = 0.02
		controller = Controller(ctrl)
		plt.ion()
		fig = plt.figure()
		ax = fig.add_subplot(111)
		X = []
		Y = []
		X2 = []
		Y2 = []
		X3 = []
		Y3 = []
		X4 = []
		Y4 = []
		line1, = ax.plot(Y, X, 'b-')
		line2, = ax.plot(Y2, X2, 'r-')
		line3, = ax.plot(Y3, X3, 'g-')
		line4, = ax.plot(Y4, X4, 'y-')

		ax.set_xlim(-0.5,0.5)
		ax.set_ylim(0, -0.3)

		Xtmp = controller._control_signal(aX, 0.1, 0.5)
		Ytmp = controller._control_signal(aY, 0.2 , 0.9)

		Xtmp2 = controller._control_signal(aX+0.01, 0.1, 0.5)
		Ytmp2 = controller._control_signal(aY+0.01, 0.1 , 0.5)

		Xtmp3 = controller._control_signal(aX+0.02,0.1, 0.5)
		Ytmp3 = controller._control_signal(aY+0.02,0.1 , 0.5)

		Xtmp4 = controller._control_signal(aX+0.03,0.1, 0.5)
		Ytmp4 = controller._control_signal(aY+0.03,0.1 , 0.5)

		for i in range(0,len(Xtmp)):
			X.append(-Xtmp[i]-0.14)
			Y.append(-Ytmp[i]-0.03)
			X2.append(-Xtmp2[i]-0.14)
			Y2.append(-Ytmp2[i]-0.03)
			X3.append(-Xtmp3[i]-0.14)
			Y3.append(-Ytmp3[i]-0.03)
			X4.append(-Xtmp4[i]-0.14)
			Y4.append(-Ytmp4[i]-0.03)
			line1.set_xdata(Y)
			line1.set_ydata(X)
			line2.set_xdata(Y2)
			line2.set_ydata(X2)
			line3.set_xdata(Y3)
			line3.set_ydata(X3)
			line4.set_xdata(Y4)
			line4.set_ydata(X4)
			fig.canvas.draw()
		plt.show(block = True)

	ctrl = ctrl_back_front
	if len(sys.argv)==3 :
		centroids = load_centroids(sys.argv[1])
		dim_x = 24
		fit, beh, x = load_data(sys.argv[2], centroids.shape[1],dim_x)
		print("Fitness max : ", max(fit))
		index = np.argmax(fit)
		# index = np.random.randint(0,len(x))
		print("Associated desc : " , beh[index] )
		print("Associated ctrl : " , x[index] )
		ctrl = x[index]
		print("Index : ", index)

	start = time.time()
	for k in range(0, 1):
		t0 = time.perf_counter()
		dist = eval_minitaur(ctrl,True)
		print(dist)
	end = time.time()
	print("Time elapsed : ", end-start)

	#
	# ids = np.argsort(fit,axis=0)
	# ids_max = []
	# for i in ids:
	# 	if(fit[i] > 3):
	# 		ids_max.append(i)
	# print("last_id : ", len(ids_max))
	#
	# print(shape(fit))
	# top_2_idx =np.argsort(fit,axis=0)[-10:]
	# top_2_values = [fit[i] for i in top_2_idx]
	# for i in range(0,len(top_2_idx)):
	# 	print(i, " : id ",top_2_idx[i]," fit ", top_2_values[i])
	#
	#
	# start = time.time()
	# for i in range(0,len(top_2_idx)):
	# 	print(" \nSTARTING EPISODE \n")
	# 	index = top_2_idx[len(top_2_idx)-i-1][0]
	# 	ctrl = x[index]
	# 	print(i, " : id ",index," fit ", fit[index])
	# 	print("desc : ", beh[index])
	# 	print("ctrl : ", ctrl)
	# 	print("\n")
	# 	t0 = time.perf_counter()
	# 	simu = MinitaurSimulator(gui=True,urdf="minitaur_derpy.urdf")
	# 	controller = Controller(ctrl,1000)
	# 	for i in range(0, 20000):
	# 		simu.step(controller)
	# 	print("\nTravelled dist \n", simu.covered_distance)
	# 	keys = list(simu.descriptor.keys())
	# 	desc = []
	# 	print(keys)
	# 	for k in keys:
	# 		cns = simu.descriptor[k]
	# 		d = round(sum(cns)/len(cns)*100.0)/100.0
	# 		desc.append(d)
	# 	print(desc)
	# 	simu.destroyed()
	# end = time.time()
	# print(end-start)
