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


def plot_control(hexapod_controller):
	a = 0.01
	d = 0.5
	p1 = 0
	p2 = 0.1
	p3 = 0.2
	p4 = 0.3
	fig = figure()
	ax = fig.add_subplot(111)
	leg1 = hexapod_controller._control_signal(a, p1, d)
	ax.plot(np.arange(0, 100), leg1)
	leg1 = hexapod_controller._control_signal(a, p2, d)
	ax.plot(np.arange(0, 100), leg1)
	fig.savefig('legs.pdf')


#Load the CVT voronoi centroids from input archive
def load_centroids(filename):
	points = np.loadtxt(filename)
	return points

#Load map data from archive
def load_data(filename, dim,dim_ctrl):
	print("Loading ",filename)
	data = np.loadtxt(filename)
	fit = data[:, 0:1]
	desc = data[:,1: dim+1]
	x = data[:,dim+1:dim+1+dim_ctrl]
	return fit, desc, x

#This function is computing a circular trajectory
def circle(omega,radius,t):
	x = -radius*np.cos(omega*t)-0.2
	y = -radius*np.sin(omega*t)
	return x,y

#Switch from cartesian to polar coordinates
def cart_to_pol(x1,x2):
	r = math.sqrt(x1*x1 + x2*x2)
	alpha = math.atan2(x2,x1)
	return r, alpha

#Compute the angle between the first leg link starting at the motor, and the second ending at the tip of the leg
def compute_intermediate_angle(r,l1,l2):
	#Go and check the minitaur.pdf to know how the formula has been calculated
	try:
		interm = math.acos((-l2 * l2 - l1 * l1 + r * r)/(2 * l1 * l2))
		error = False
	except:
		# print("Impossible motion according to leg lenght")
		interm = 0
		error = True
	return interm, error

#Compute the motor angles to give to the motors to put the tip of the leg in (x1,x2)
def compute_theta(interm,r,l1,l2,x1,x2):

	temp1 = l1+l2*math.cos(interm)
	temp2 = l2*math.sin(interm)
	theta1 = 0
	theta2 = 0
	try:
		theta1 = math.acos((temp1*x1+temp2*x2)/(temp1*temp1+temp2*temp2))
		error = False
	except:
		# print("Impossible motion according to leg lenght")
		interm = 0
		error = True
	try:
		theta2 = math.acos((temp1*x1-temp2*x2)/(temp1*temp1+temp2*temp2))
		error = False
	except:
		# print("Impossible motion according to leg lenght")
		interm = 0
		error = True
	return theta1,theta2,error

class MinitaurSimulator:
	def __init__(self, gui=False, urdf='pexod.urdf', dt = 1e-3, control_dt=0.001):
		self.GRAVITY = -9.8
		self.dt = dt
		self.control_dt = control_dt
		self.t = 0
		if gui:
			self.physicsClient = p.connect(p.GUI)
		else:
			self.physicsClient = p.connect(p.DIRECT)
		p.setAdditionalSearchPath(pybullet_data.getDataPath())
		p.resetSimulation()
		p.setGravity(0,0,self.GRAVITY)
		p.setTimeStep(self.dt)

		self.planeId = p.loadURDF("plane.urdf")

		#Size of the leg links
		self.l1 = 0.1
		self.l2 = 0.2
		#self.mini = Minitaur(p,"/nfs/hal01/edalin/bullet3/data/")
		# import pybulletminitaur as minit
		# self.mini = minit.Minitaur(p,"/nfs/hal01/edalin/bullet3/data/",motor_velocity_limit=3.14*2)
		import pybulletminitaur_derpy as minit_derpy
		self.mini = minit_derpy.MinitaurDerpy(p,"bullet3/data/",motor_velocity_limit=3.14*2)
		# self.mini = minitaur_derpy.MinitaurDerpy(p,"/nfs/hal01/edalin/bullet3/data/")
		# self.mini = minitaur_derpy.MinitaurDerpy(p,"/home/eloise/pyinstall/bullet3/data/")
		self.leg_link_ids = [3,9,16,22]
		self.descriptor = {3 : [], 9 : [],16 : [],22 : []}
		self.covered_distance = 0
		self.init_angles = self.mini.GetMotorAngles()
		self.inside_corridor = True
		self.position =  self.mini.GetBasePosition()
		self.euler =self.mini.GetTrueBaseRollPitchYaw()
		self.safety_turnover = True
		self.angle_lim = 30
		self.energy_reward = []
		self.forward_reward = 0
		self.drift_reward = 0
		self.reward = 0
		self.currents = []

	def compute_final_reward(self):
	   # self.reward = self.forward_reward + 0.5*self.drift_reward + 0.05*(sum(self.energy_reward)/len(self.energy_reward))
	   self.reward = self.forward_reward
	   # print("Forward reward : ", self.forward_reward)
	   # print("Drift reward : ",  0.5*self.drift_reward)
	   # print("Energy reward : ", (sum(self.energy_reward)/len(self.energy_reward)), " after weight : ", 0.05*(sum(self.energy_reward)/len(self.energy_reward)))
	   # print("Total reward : ",self.reward)

	def reset(self):
		assert(0), "not working for now"
		self.t = 0
		p.restoreState(self._init_state)
		for joint in self.joint_list:
			p.resetJointState(self.botId, joint, 0)

	def get_pos(self):
		'''
		Returns the position list of 3 floats and orientation as list of 4 floats in [x,y,z,w] order.
		Use p.getEulerFromQuaternion to convert the quaternion to Euler if needed.
		'''
		return p.getBasePositionAndOrientation(self.botId)

	def step(self, controller):
		position = self.mini.GetBasePosition()
		self.position =  self.mini.GetBasePosition()
		self.euler = self.mini.GetTrueBaseRollPitchYaw()
		self.covered_distance = position[0]#sqrt(position[0]*position[0] + position[1]*position[1])
		self.forward_reward = self.position[0]
		self.drift_reward = -abs(self.position[1])
		self.energy_reward.append(-np.abs(np.dot(self.mini.GetTrueMotorTorques(), self.mini.GetTrueMotorVelocities())))
		# print(-np.abs(np.dot(self.mini.GetTrueMotorTorques(), self.mini.GetTrueMotorVelocities())))
		#See if the minitaur center is still in a 1meter large corridor, if not stop the episode
		self.currents.append([a/0.0954 for a in self.mini.GetTrueMotorTorques()])
		if(abs(position[1])>0.3):
			self.inside_corridor = False
			return False

		#Compute the 4  tip of the legs trajectories
		traj = controller.step(self.t)
		j = 0

		#cmd will contain the 8 motor angles to send to the minitaur
		#the order of the angles has to correspond to the function  ApplyAction of the Minitaur class define in pybullet
		cmd = []

		#for each leg
		for i in range(0,4):
			#Here we take x1,x2 <0 according to the x axis defined in mintaur.pdf
			x1 = -traj[j] - 0.14
			# x1 = -0.14 - self.t*0.01
			# x1 = -0.14
			x2 = -(traj[j+1] + 0.025)
			# x2 = 0
			# x2 = 0
			r, alpha = cart_to_pol(x1,x2)
			interm, error = compute_intermediate_angle(r,self.l1,self.l2)


			if(self.safety_turnover):
				if((abs(self.euler[1]) >= math.pi/4) or (abs(self.euler[0]) >= math.pi/4)):
					error = True

			angles = self.mini.GetTrueMotorAngles()
			leg1 = (angles[0]+angles[1])*180/math.pi
			leg2 = (angles[2]+angles[3])*180/math.pi
			leg3 = (angles[4]+angles[5])*180/math.pi
			leg4 = (angles[6]+angles[7])*180/math.pi
			leg_angles = [leg1, leg2, leg3, leg4]
			for l in leg_angles:
				if(l<self.angle_lim):
					error = True
				if(l>(360 - self.angle_lim)):
					error = True

			if(error):
				self.covered_distance = -1000
				return error
			else :
				theta,theta2, error = compute_theta(interm,r,self.l1,self.l2,x1,x2)
				if(error):
					self.covered_distance = -1000
					return error
				else:
					if(i<=1):
						cmd.append(theta2)
						cmd.append(theta)
					else:
						cmd.append(theta)
						cmd.append(theta2)
					j = j+2
		# print(cmd)
		self.mini.ApplyAction(cmd)



		#Get contact points between minitaur and world plane
		contact_points = p.getContactPoints(self.mini.quadruped,self.planeId)
		link_ids = [] #list of links in contact with the ground plane
		if(len(contact_points)>0):
			for cn in contact_points:
				linkid= cn[3] #minitaur link id in contact with world plane
				if linkid not in link_ids:
					link_ids.append(linkid)
		# num_leg_on_ground = 0
		for l in self.leg_link_ids:
			cns = self.descriptor[l]
			if l in link_ids:
				# num_leg_on_ground=num_leg_on_ground+1
				cns.append(1)
			else:
				cns.append(0)
			self.descriptor[l] = cns
		# if(num_leg_on_ground==0):
			# error = True
			# return error

		p.setGravity(0,0,self.GRAVITY)
		p.stepSimulation()
		self.t += self.control_dt
		return error

	def _make_joint_list(self, botId):
		#NOT USED HERE
		joint_names = [b'motor_front_rightR_joint',
		b'motor_front_rightL_joint',
		b'motor_back_rightR_joint',
		b'motor_back_rightL_joint',
		b'motor_front_leftL_joint',
		b'motor_front_leftR_joint',
		b'motor_back_leftL_joint',
		b'motor_back_leftR_joint']
		joint_list = []
		for n in joint_names:
			for joint in range (p.getNumJoints(botId)):
				name = p.getJointInfo(botId, joint)[1]
				if name == n:
					joint_list += [joint]
		return joint_list


	def destroyed(self):
		p.disconnect()

def eval_minitaur(ctrl):
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
	# 	dist = eval_minitaur(ctrl)
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
	# 		results = pool.map(eval_minitaur,ctrls_to_test)
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
