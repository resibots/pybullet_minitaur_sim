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
		interm = 0
		error = True
	try:
		theta2 = math.acos((temp1*x1-temp2*x2)/(temp1*temp1+temp2*temp2))
		error = False
	except:
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

		#import modified minitaur to include motor_velocity_limit and torque_max
		import pybulletminitaur_derpy as minit_derpy
		self.mini = minit_derpy.MinitaurDerpy(p,os.path.dirname(os.path.abspath(__file__))+"/bullet3/data/",motor_velocity_limit=3.14*2,torque_max=3.5)

		#bullet links number corresponding to the legs
		self.leg_link_ids = [3,9,16,22]
		self.descriptor = {3 : [], 9 : [],16 : [],22 : []}

		self.covered_distance = 0
		self.init_angles = self.mini.GetMotorAngles()

		#Corridor boolean value is true if the minitaur is inside a 1meter wide corridor
		self.inside_corridor = True
		self.position =  self.mini.GetBasePosition()
		self.euler =self.mini.GetTrueBaseRollPitchYaw()

		#Enable the turnover safety : stops the simulation if pitch rool angles are too high
		self.safety_turnover = True
		self.angle_lim = 30

		self.energy_reward = []
		self.forward_reward = 0
		self.drift_reward = 0
		self.reward = 0
		self.currents = []
		self.meanTorques = [0,0,0,0,0,0,0,0]
		self.torqueDescriptors = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
		self.tmpTorqueDescriptors = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
		self.descCount = 0
		self.meanCount = 0
		self.meanTorquesDescriptors = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
		self.meanLegTorques = [0,0,0,0]
		self.cycle_index = 0
		self.meanCurrents = []
		self.dtCurrent = 0
		self.safetyCurrentCut = True

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

		self.position =  self.mini.GetBasePosition()
		self.euler = self.mini.GetTrueBaseRollPitchYaw()

		self.covered_distance = self.position[0]
		self.forward_reward = self.position[0]


		#self.drift_reward = -abs(self.position[1])
		#self.energy_reward.append(-np.abs(np.dot(self.mini.GetTrueMotorTorques(), self.mini.GetTrueMotorVelocities())))

		# See if the minitaur is not using too much current, safety limit that is also implementeed on the real robot wit 25A instead of 30A
		self.currents.append([abs(a)/0.0954 for a in self.mini.GetTrueMotorTorques()])
		self.meanCurrents =  np.abs(np.mean(self.currents,axis=0))
		if(self.safetyCurrentCut):
			if(self.dtCurrent>1):
				print(self.meanCurrents)
				for mc in self.meanCurrents:
					if(mc>30):
						error = True
						return error
				self.currents = []
				self.meanCurrents = []
				self.dtCurrent = 0
		# self.currents.append([abs(a) for a in self.mini.GetTrueMotorTorques()])

		# See if the minitaur center is still in a 1meter large corridor, if not stop the episode
		if(abs(self.position[1])>0.3):
			self.inside_corridor = False
			error = False
			return error

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
			interm, error = compute_intermediate_angle(r,self.l1,self.l2) #error here will be to try to reach a point that the leg cannot reach (for ex lenght limit)

			#Check if roll pitch are not too high
			if(self.safety_turnover):
				if((abs(self.euler[1]) >= math.pi/4) or (abs(self.euler[0]) >= math.pi/4)):
					error = True

			#Avoid singularity when the legs are fully retracted (on the real robot we need to put more strenght to exit singularity)
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
		self.mini.ApplyAction(cmd) #send the traj commands

		#### DESCRIPTOR TORQUE COMPUTATION##############################################

		#sum the torques for each motor, at each time step
		torques = self.mini.GetTrueMotorTorques()
		for g in range(0,len(self.meanTorques)):
			self.meanTorques[g] = abs(self.meanTorques[g])+ abs(torques[g])
		self.descCount = self.descCount +1

		# sum the motor torques corresponding to one leg
		k = 0
		for i in range(0,4):
			self.meanLegTorques[i] = (self.meanTorques[k]+self.meanTorques[k+1])/(2*self.descCount)
			k=k+2
		#compute the mean 4 times per 1 second controller cycle
		cycleT = [0.25,0.50,0.75,1]
		k=0
		if(self.t!=0):
			#For every cycle time slice
			if(self.t%cycleT[self.cycle_index]<self.dt):
				#Compute for every leg the mean torque
				k=4*self.cycle_index
				for i in range(0,4):
					self.tmpTorqueDescriptors[k] = (self.tmpTorqueDescriptors[k] + self.meanLegTorques[i])
					k=k+1
				self.meanTorques = [0,0,0,0,0,0,0,0]
				self.meanLegTorques = [0,0,0,0]
				self.descCount = 0
				self.cycle_index = self.cycle_index +1

		# self.tmpTorqueDescriptors = [mean_torque_leg1_slice1, mean_torque_leg2_slice1,mean_torque_leg3_slice1,mean_torque_leg4_slice1 ...... mean_torque_leg4_slice4]
		#If one cycle is completed , compute the means over the cycles
		if(self.t!=0):
			if(self.t%1<self.dt):
				self.meanCount = self.meanCount + 1
				self.cycle_index = 0
				for k in range(0,len(self.tmpTorqueDescriptors)):
					self.torqueDescriptors[k] = self.tmpTorqueDescriptors[k]/self.meanCount

				# Normalize torque vector
				maxT = max(3.5, max(self.torqueDescriptors))
				minT = 0
				maxC1 = max(3.5, max(self.torqueDescriptors[0:4]))
				maxC2 = max(3.5, max(self.torqueDescriptors[4:8]))
				maxC3 = max(0.7*3.5, max(self.torqueDescriptors[8:12])) #From data: the maxC3 and maxC4 where bincluded btwn  0 and 0.7*3.5, so we threshold before to avoid unacessary map-elites exploration
				maxC4 = max(0.5*3.5, max(self.torqueDescriptors[12:16]))
				self.torqueDescriptors[0:4] = [(a-minT)/(maxC1-minT) for a in self.torqueDescriptors[0:4]]
				self.torqueDescriptors[4:8] = [(a-minT)/(maxC2-minT) for a in self.torqueDescriptors[4:8]]
				self.torqueDescriptors[8:12] = [(a-minT)/(maxC3-minT) for a in self.torqueDescriptors[8:12]]
				self.torqueDescriptors[12:16] = [(a-minT)/(maxC4-minT) for a in self.torqueDescriptors[12:16]]


		#### DESCRIPTOR DUTY CYCLE HEXAPOD NOT USED HERE ###################################

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
		######### SIMULATION STEP ##########################################################""
		p.setGravity(0,0,self.GRAVITY)
		p.stepSimulation()
		self.t += self.control_dt
		self.dtCurrent += self.control_dt
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

def eval_minitaur(ctrl,gui_eval = False):
	simu = MinitaurSimulator(gui=gui_eval)
	controller = Controller(ctrl,1000)
	# Run the simulation for 10 seconds
	for i in range(0, 10000):
		error = simu.step(controller)
		if(error):
			#if there is an error stop simulation and penalize the bahavior
			desc = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
			dist = -1000, np.array(desc)
			simu.destroyed()
			return dist
		if(not simu.inside_corridor):
			#if the minitaur leaves the corridor stop simulation and include current desc and reward
			simu.compute_final_reward()
			dist = simu.reward, np.array(simu.torqueDescriptors)
			simu.destroyed()
			return dist
	simu.compute_final_reward()
	dist = simu.reward, np.array(simu.torqueDescriptors)
	simu.destroyed()
	return dist

if __name__ == "__main__":
	p1 = \
	    {
	        "cvt_samples": 400000,
	        "batch_size": 200,
	        "random_init": 1500,
			"random_init_batch": 1000,
	        "sigma_iso": 0.01,
	        "sigma_line": 0.2,
	        "dump_period": 50,
	        "parallel": True,
	        "cvt_use_cache": True,
		    "min": [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	        "max": [0.1,1,1,0.1,1,1,0.1,1,1,0.1,1,1,0.1,1,1,0.1,1,1,0.1,1,1,0.1,1,1]
	    }
	archive = compute(16,24, eval_minitaur, n_niches=40000, n_gen=20000,params = p1)

	# centroids = load_centroids(sys.argv[1])
	# dim_x = 24
	# fit, beh, x = load_data(sys.argv[2], centroids.shape[1],dim_x)

	# indexes = np.load("saved_ids.npy")
	# for id in indexes :
	# 	ctrl = x[int(id)]
	# 	dist = eval_minitaur(ctrl)
	# 	print(dist)

	# print("Fitness max : ", max(fit))
	# index = np.argmax(fit)
	# index = np.random.randint(0,len(x))
	# print("Associated desc : " , beh[index] )
	# print("Associated ctrl : " , x[index] )
	# ctrl = x[index]
	# print("Index : ", index)
	# print("Fitness : ", fit[index])
	# d = 0.2
	# aX = 0.02
	# aY = 0.05
	# #Same phase along Y will lead to a forward sync
	# #Same phase along X will lead to a up/down sync
	# #ctrl = ["front_left", "back_left", "front_right", "back_right"]
	# ctrl_back_front = [aX, 1, d,aY, 0.0, d, aX, 0.1, d, aY, 0.0, d, aX, 1, d,aY, 0.0, d,aX,0.1,d,aY, 0.0, d]
	# ctrl = ctrl_back_front
	#
	# aX = 0.03
	# aY = 0.01
	# d = 0.1
	# ctrl_jump = [aX, 0.1, d, aY,0.0, d,aX, 0.1, d,aY, 0.0, d,aX, 0.1, d,aY, 0.0, d,aX,0.1,d,aY, 0.0, d]
	# ctrl = ctrl_jump

	# aX = 0.02
	# aY = 0.03
	# d = 0.5
	# ctrl_sync_diagonal_2 = [aX, 0.1, d, aY, 0.0, d,aX, 1, d,aY, 0.5, d,aX, 1, d,aY, 0.5, d,aX,0.1,d,aY, 0.0, d]
	# ctrl = ctrl_sync_diagonal_2
	# dist = eval_minitaur(ctrl)
	# print(dist)
	#from minitaur import *
	#import os
	#dir_path = os.path.dirname(os.path.realpath(__file__))
	#print(dir_path)
