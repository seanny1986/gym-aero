import simulation.quadrotor3 as quad
import simulation.config as cfg
import simulation.animation as ani
import simulation.animation_gl as ani_gl
import matplotlib.pyplot as pl
import numpy as np
import random
import time
import gym
from math import pi, sin, cos
from gym import error, spaces, utils
from gym.utils import seeding

#Converts numpy vectors to a list
def npl(np_vec):
	return np_vec.T.tolist()[0]

#Generates a random point in 3 dimensional space from
#-ptSize to ptSize in each dimension
def rndPt(ptSize):
	x = random.uniform(-ptSize,ptSize)
	y = random.uniform(-ptSize,ptSize)
	z = random.uniform(-ptSize,ptSize)
	return np.array([[x], [y], [z]])

#Returns the midpoint between two points
def midPoint(pt1, pt2):
	return (pt2 + pt1) / 2.0

#Performs a quadratic bezier from start to end via the control
#point, t is a value from 0 to 1 which indicates a parametric
#parameter which will indicate what point of the curve is returned
#(i.e 0 will return start & 1 will return end)
def bezier(start, end, control, t):
	#Cache t inverse for calculation
	t_inv = 1.0 - t
	#Quadratic bezier calculation
	return t_inv*(t_inv*start + t*control) + t*(t_inv*control + t*end)

#Calculates the approximate length of a quadratic bezier curve
#from start to end via the control point
def bezier_approx_length(start, end, control):
	#The number of samples taken to approximate the length
	approximationFactor = 5
	dist = 0.0
	cPoint = start
	#Take samples on the curve and sum the distance between
	#them to find the approimate length of the curve
	for i  in range(approximationFactor - 1):
		pt = bezier(start, end, control, (1.0 / approximationFactor) * i)
		dist += np.linalg.norm(pt - cPoint)
		cPoint = pt
	dist += np.linalg.norm(end - cPoint)

	return dist

class TargetFollowingEnv(gym.Env):
	def __init__(self):
		metadata = {'render.modes': ['human']}

		# environment parameters
		self.goal_xyz = np.array([[2.0],
								[1.5],
								[0.0]])
		self.start_pos = np.array([[0.0],
								[0.0],
								[0.0]])

		self.goal_zeta_sin = np.sin(np.array([[0.],
											[0.],
											[0.]]))
		self.goal_zeta_cos = np.cos(np.array([[0.],
											[0.],
											[0.]]))
		self.goal_uvw = np.array([[0.],
								[0.],
								[0.]])
		self.goal_pqr = np.array([[0.],
								[0.],
								[0.]])

		#Sets the xyz dimensions of the environment
		self.x_dim = 4
		self.y_dim = 4
		self.z_dim = 4

		#The current running time (in seconds) of the simulation
		self.t = 0
		
		#The total maximum time of the simulation
		self.T = 25
		
		#The number of action input parameters
		self.num_actions = 4
		
		#The number of input options
		self.action_space = np.zeros((self.num_actions,))
		
		#The number of values that pertain to the current state of the agent
		self.nStateVals = 22
		#The number of goal values
		self.nGoals = 1
		
		#The total number of values that pertain to an observation (given state)
		self.observation_space = np.zeros((self.nStateVals+self.num_actions+self.nGoals,))

		# simulation parameters
		self.params = cfg.params
		self.iris = quad.Quadrotor(self.params)
		self.sim_dt = self.params["dt"]
		self.ctrl_dt = 0.05
		self.steps = range(int(self.ctrl_dt/self.sim_dt))
		self.action_bound = [0, self.iris.max_rpm]
		self.H = int(self.T/self.ctrl_dt)
		self.hov_rpm = self.iris.hov_rpm
		self.trim = [self.hov_rpm, self.hov_rpm,self.hov_rpm, self.hov_rpm]
		self.trim_np = np.array(self.trim)
		self.prev_action + self.trim_np.copy()
		self.bandwidth = 35.

		self.iris.set_state(self.start_pos, np.arcsin(self.goal_zeta_sin), self.goal_uvw, self.goal_pqr)
		xyz, zeta, uvw, pqr = self.iris.get_state()

		self.vec_xyz = xyz-self.start_pos
		self.vec_zeta_cos = np.cos(zeta)-self.goal_zeta_cos

		self.prev_uvw = np.array([[0.],[0.],[0.]])
		self.prev_pqr = np.array([[0.],[0.],[0.]])

		self.v = None
		self.init_rendering = False

	# Initializes a random quadratic bezier path of approximate length maxLen,
	# returns a list of points of length equal to maxTime / timesteps
	def initializeGoalPath(self, maxLen, maxTime, timesteps, ptSize=2.5):       
		points = []
		maxPoints = int(maxTime / timesteps)
		cur_pt = rndPt(ptSize)
		next_pt_1 = rndPt(ptSize)
		next_pt_2 = rndPt(ptSize)
		midpoint_1 = midPoint(cur_pt, next_pt_1)
		midpoint_2 = midPoint(next_pt_1, next_pt_2)

		while(len(points) < maxPoints):
			#Calculate the length of the current curve
			bez_length = bezier_approx_length(midpoint_1, midpoint_2, next_pt_1)
			#Calculate the number of points inside the current curve
			bez_pts = int((bez_length / maxLen) * maxPoints)
			#Calculate how many of the points we need from this curve
			nPoints = min(bez_pts, maxPoints - len(points) + 1)

			#Calculate each of the required points of the curve
			for i in range(nPoints - 1):
				pt = bezier(midpoint_1, midpoint_2, next_pt_1, (1.0 / bez_pts) * i)
				points.append(pt)

			#Randomize new points to generate the next curve
			cur_pt = next_pt_1
			next_pt_1 = next_pt_2
			next_pt_2 = rndPt(ptSize)
			midpoint_1 = midpoint_2
			midpoint_2 = midPoint(next_pt_1, next_pt_2)

		#Return the list of points which define the path of the goal
		return points;      

	def reward(self, state, action, terminal):
		xyz, zeta, uvw, pqr = state

		#Get the distance from the agent to the goal
		curr_dist = xyz-self.goal_xyz
		self.dist_hat = np.linalg.norm(curr_dist)

		#Agent receives reward for maintaining the set distance from the goal
		if(self.dist_hat < self.goal_dist):
			dist_rew = 50
			time_rew = 30 * self.t
		else:
			dist_rew = 0
			#Agent still receives some time reward for statying alive,
			#but less than ormal
			time_rew = 5 * self.t
		
		#Agent receives a bad reward for dying (to prevent dive-bombing close to goal)
		term_rew = -1000 if (terminal and self.t < self.T) else 0

		#Agent gets a negative reward for excessive action inputs
		ctrl_rew = 0.
		ctrl_rew -= np.sum(((action-self.trim_np)/self.action_bound[1])**2)
		ctrl_rew -= np.sum((((action-self.prev_action)/self.action_bound[1])**2))
		ctrl_rew -= 10.*np.sum((uvw-self.prev_uvw)**2)
		ctrl_rew -= 10.*np.sum((pqr-self.prev_pqr)**2)
		return dist_rew, time_rew, term_rew, ctrl_rew

	def terminal(self, pos):
		xyz, zeta = pos
		#If the agent has flipped over 90 degrees
		mask1 = zeta > pi/2
		mask2 = zeta < -pi/2
		#If the agent has left the bounds of the container
		mask3 = (np.abs(xyz[0]) > self.x_dim or
			np.abs(xyz[1]) > self.y_dim or
			np.abs(xyz[2]) > self.z_dim)
		if np.sum(mask1) > 0 or np.sum(mask2) > 0 or np.sum(mask3) > 0:
			return True
		elif self.t >= self.T:
			#Simulation has completed as time has expired
			print("Sim time reached")
			return True

		#Simulation not yet completed
		return False

	def step(self, action):
		for _ in self.steps:
			xyz, zeta, uvw, pqr = self.iris.step(self.trim_np+action*self.bandwidth)
		sin_zeta = np.sin(zeta)
		cos_zeta = np.cos(zeta)
		current_rpm = (self.iris.get_rpm()/self.action_bound[1]).tolist()
		self.goal_xyz = self.move_goal(self.t)
		self.goal_veloc = self.goal_xyz - last_goal_xyz
		vec_to_goal = self.goal_xyz - xyz
		self.dist_to_goal = np.linalg.norm(vec_to_goal)
		self.vec_to_goal = vec_to_goal / self.dist_to_goal

		next_position = xyz.T.tolist()[0]
		next_attitude = sin_zeta.T.tolist()[0]+cos_zeta.T.tolist()[0]
		next_velocity = uvw.T.tolist()[0]+pqr.T.tolist()[0]
		next_state = next_position+next_attitude+next_velocity
		info = self.reward((xyz, zeta, uvw, pqr), action)
		done = self.terminal((xyz, zeta))
		reward = sum(info)
		position_goal = self.vec_xyz.T.tolist()[0] 
		attitude_goal = self.vec_zeta_sin.T.tolist()[0]+self.vec_zeta_cos.T.tolist()[0]
		velocity_goal = self.vec_uvw.T.tolist()[0]+self.vec_pqr.T.tolist()[0]
		goals = position_goal+attitude_goal+velocity_goal
		next_state = next_state+current_rpm+goals
		self.prev_action = rpm_command.copy()
		self.prev_uvw = uvw.copy()
		self.prev_pqr = pqr.copy()
		self.t += 1
		return next_state, reward, done, {"dist_rew": info[0], 
                                        "att_rew": info[1], 
                                        "vel_rew": info[2], 
                                        "ang_rew": info[3], 
                                        "ctrl_rew": info[4], 
                                        "time_rew": info[5]}

	#Retrieves the position of the goal
	def get_goal(self):
		return self.goal_xyz

	#Moves the goal to the position in which it is located at timestep t
	#of the simulation where t <= self.T
	def move_goal(self, t):
		#Calculate the current simulation tick
		timestep = int(self.t / self.ctrl_dt)
		#Maximize simulation tick to end of goal path in case a calculation
		#error occurs in the goal path algorithm
		timestep = min(timestep, len(self.goal_point_path) - 1)
		#Return the position of the goal at the current time step
		return self.goal_point_path[timestep]

	def reset(self):
		self.t = 0.
		self.dist_hat = 0
		self.iris.set_state(self.start_pos, np.sin(self.goal_zeta_sin), self.goal_uvw, self.goal_pqr)
		xyz, zeta, uvw, pqr = self.iris.get_state()
		sin_zeta = np.sin(zeta)
		cos_zeta = np.cos(zeta)
		self.vec_xyz = xyz-self.start_pos
		self.vec_zeta_cos = cos_zeta-self.goal_zeta_cos
		self.goal_path_len = 25.0
		a = [x/self.action_bound[1] for x in self.trim]

		self.goal_point_path = self.initializeGoalPath(self.goal_path_len, self.T, self.ctrl_dt)

		#Initial goal velocity is nothing, as goal hasn't moved yet
		self.goal_veloc = np.array([[0.],
									[0.],
									[0.]])
		#Initial relative intertial velocity is nothing, as neither agent nor goal is moving
		self.rel_inertial_vel = np.array([[0.],
									[0.],
									[0.]])
		#Reset the goal position
		self.goal_xyz = self.move_goal(self.t)
		
		#The distance which the agent should attempt to keep within from the target
		self.goal_dist = 2
		#Calculate the vector to the goal
		vec_to_goal = xyz-self.goal_xyz
		#The distance to the goal
		self.dist_to_goal = np.linalg.norm(vec_to_goal)
		#Normalize vector to goal
		vec_to_goal = vec_to_goal / self.dist_to_goal

		#Get the initial goal state
		goals = [self.goal_dist]
		#Get the initial state of the agent
		state = npl(xyz)+npl(sin_zeta)+npl(cos_zeta)+npl(uvw)+npl(pqr)+npl(vec_to_goal)+[self.dist_to_goal]+npl(self.goal_veloc)
		#Combine the goals, actions and state values to represent the entire agent state
		state = state+a+goals
		return state

	def render(self, mode='human'):
		#If first call to render, initialize rendering
		if(not self.init_rendering):
			self.ani = ani_gl.VisualizationGL(name="Target Following",
				x_dim=self.x_dim, y_dim=self.y_dim, z_dim=self.z_dim)
			self.init_rendering = True

		xyz, zeta, uvw, pqr = self.iris.get_state()
		self.ani.draw_quadrotor(self.iris)
		self.ani.draw_goal(self.goal_xyz, color=(1,1,0))
		self.ani.draw_label("Time: {0:.2f}".format(self.t), 
			(self.ani.window.width // 2, 20.0))
		self.ani.draw_line(self.goal_xyz, xyz)
		self.ani.draw()
		
		#Slow down animation to a reasonable watching speed
		if(mode == 'human'):
			time.sleep(0.05)