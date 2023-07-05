import gym
import rospy
#import roslaunch
import time
import numpy as np
import os

from gym import utils, spaces
from gym_gazebo.envs import gazebo_env
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

from std_msgs.msg import Float32, Int16, Float32MultiArray
from geometry_msgs.msg import Point

from gym.utils import seeding


class GazeboOceanEboatEnvD(gazebo_env.GazeboEnv):

	def __init__(self):
		self.EBOAT_HOME = "/home/lmdc/eboat_ws/src/eboat_gz_1"
		#-->Launch the simulation with the given launchfile name
		gazebo_env.GazeboEnv.__init__(self, os.path.join(self.EBOAT_HOME,"eboat_gazebo/launch/ocean.launch"))

		self.bang_pub    = rospy.Publisher('/eboat/control_interface/sail', Float32, queue_size=5)
		self.rang_pub    = rospy.Publisher('/eboat/control_interface/rudder', Float32, queue_size=5)
		self.pvel_pub    = rospy.Publisher('/eboat/control_interface/propulsion', Int16, queue_size=5)
		self.wind_pub    = rospy.Publisher('/eboat/atmosferic_control/wind', Point, queue_size=5)
		self.unpause     = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
		self.pause       = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
		self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

		# -->GLOBAL VARIABLES
		self.DMAX = 5.0  # --> Threshold for distance. If the boat goes far than that, a done signal is trigged.
		self.D0 = None  # --> The intial distance from the waypoint

		#--> Eletric propulsion [-5,5], Rudder angle [-60,60], Boom angle [0, 90]
		self.action_vec = []
		for pvel in [0, 4, -4]:          #--> eletric propulsion
			for bang in range(0,91,10):          #--> boom angle   (retranca)
				for rang in [-35, -10, 0, 10, 35]:    #--> rudder angle (leme)
					self.action_vec.append([pvel, bang, rang])
		self.action_space = spaces.Discrete(len(self.action_vec))

		self.observation_space = spaces.MultiDiscrete([np.arange(-180, 181, 10).shape[0],  #-->trajectory angle
													   35                               ,  #-->boat linear velocity
													   np.arange(-180, 181, 10).shape[0]]  #-->apparent wind angle
													 )

		self.reward_range = (-np.inf, np.inf)

		self._seed()

		#--> set wind speed initial vector
		self.windSpeed = np.array([0.0, 6.0, 0.0], dtype=float)

		# -->Get the initial distance from the waypoint
		while self.D0 is None:
			try:
				self.D0 = rospy.wait_for_message('/eboat/mission_control/observations',
												 Float32MultiArray, timeout=5).data[0]
				self.DMAX += self.D0  # -->It take the initial distance in consideration
			except:
				pass

	def setWindSpeed(self, windSpeed):
		self.windSpeed = windSpeed

	def getWindSpeed(self):
		return self.windSpeed

	def sampleWindSpeed(self):
		ws = self.np_random.integers(-9, 9, 2, dtype=int)
		self.windSpeed[0] = ws[0]
		self.windSpeed[1] = ws[1]

	def rewardFunction(self, next_state):
		# -->If the distance from the way point is reduced, a reward is earned
		# -->If the distance from the way point is increased, a penalty is earned
		# -->If the boat moves to point toward the waypoint, a reward is earned
		# -->If the eletric propulsion is used, a penalty is earned
		return (2 * (self.D0 - next_state[0]) + 0.01 * (90.0 - abs(next_state[1])) - 4 * (abs(next_state[7])))

	def getObservations(self):
		obsData = None
		while obsData is None:
			try:
				obsData = rospy.wait_for_message('/eboat/mission_control/observations', Float32MultiArray, timeout=5).data
			except:
				pass

		#--> obsData = [distance, trajectory angle, linear velocity, aparent wind speed, aparent wind angle, boom angle, rudder angle, eletric propultion speed]
		#              [   0    ,        1        ,       2        ,         3         ,         4         ,     5     ,      6      ,            7            ]

		return np.array(obsData, dtype=float)

	def _seed(self, seed=None):
		self.np_random, seed = seeding.np_random(seed)
		return [seed]

	def step(self, action):
		#--> Unpause simulation
		rospy.wait_for_service('/gazebo/unpause_physics')
		try:
			self.unpause()
		except( rospy.ServiceException) as e:
			print(("/gazebo/unpause_physics service call failed!"))

		#-->Send the required action to the boat control interface
		# self.action_vec => [pvel, bang, rang]
		self.pvel_pub.publish(self.action_vec[action][0])
		self.bang_pub.publish(self.action_vec[action][1])
		self.rang_pub.publish(self.action_vec[action][2])

		#-->Get Observations (next state)
		observations = self.getObservations()

		#-->Pause simulation
		rospy.wait_for_service('/gazebo/pause_physics')
		try:
			self.pause()
		except( rospy.ServiceException) as e:
			print(("/gazebo/pause_physics service call failed!"))

		#-->Check for a terminal state
		dist = observations[0]
		done = (dist < 3.0) | (dist > self.DMAX)  # --> Considering that the distance is measured in meters

		#-->Computes the reward
		if not done:
			reward = self.rewardFunction(observations)
		elif (dist > self.DMAX):
			reward = -500
		else:
			reward = 500

		#-->Update the current distance
		self.D0 = dist

		return observations, reward, done, False, {}

	def reset(self):
		#--> Resets the state of the environment and returns an initial observation.
		rospy.wait_for_service('/gazebo/reset_simulation')
		try:
			self.reset_proxy()
		except (rospy.ServiceException) as e:
			print(("/gazebo/reset_simulation service call failed!"))

		#--> Unpause simulation to make observation
		rospy.wait_for_service('/gazebo/unpause_physics')
		try:
			self.unpause()
		except( rospy.ServiceException) as e:
			print(("/gazebo/unpause_physics service call failed!"))

		#--> Set random wind speed and direction
		self.wind_pub.publish(Point(self.windSpeed[0], self.windSpeed[1], self.windSpeed[2]))

		#--> Collect observations
		observations = self.getObservations()

		#--> Pause simulation
		rospy.wait_for_service('/gazebo/pause_physics')
		try:
			self.pause()
		except( rospy.ServiceException) as e:
			print(("/gazebo/pause_physics service call failed!"))

		return observations, {}