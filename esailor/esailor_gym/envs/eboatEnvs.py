import gymnasium as gym
import rospy
import numpy as np
import sys, os, signal, subprocess
import time
import glob

from scipy import interpolate

from datetime import datetime

from gymnasium import utils, spaces
from std_srvs.srv import Empty

from std_msgs.msg import Float32, Int16, Float32MultiArray
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from gymnasium.utils import seeding

from tf.transformations import quaternion_from_euler
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelStates

import random
from rosgraph_msgs.msg import Clock

from std_srvs.srv import Empty

from std_msgs.msg import Float32, Int16, Float32MultiArray
from geometry_msgs.msg import Point, Pose

from tf.transformations import quaternion_from_euler
from gazebo_msgs.srv import SetModelState, SpawnModel, DeleteModel
from gazebo_msgs.msg import ModelState

class EboatBase(gym.Env):
    def __init__(self):
        # -->DEFINE A UNIQUE MODEL NAME FOR OUR BOAT
        modelname = f"eboat4tr"

        # -->SERACH FOR THE URDF FILE DESCRIBING THE BOAT
        HOME = os.path.expanduser('~')
        files = glob.glob(os.path.join(HOME, f"**/*Yara_OVE/**/*{modelname}.urdf.xacro"), recursive=True)
        if len(files) > 0:
            urdffilepath = files[0]
            del (files)
        else:
            raise IOError(f"File {modelname}.urdf.xacro does not exist")

        # -->TRANSFORM THE XACRO FILE TO URDF
        # subprocess.run(["xacro", urdffilepath, f"{modelname}.urdf"], capture_output=True)
        os.system(f"xacro {urdffilepath} > {modelname}.urdf")

        # -->SPAWN THE MODEL IN THE GAZEBO SIMULATION
        self.spawn_urdf = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
        ipose = Pose()
        ipose.position.x = 0.0
        ipose.position.y = 0.0
        ipose.position.z = 0.0
        self.model_namespace = f"eboat_{1}"
        count = 0
        spawnflag = "Fail"
        while (spawnflag == "Fail") & (count < 18):
            with open(f"{modelname}.urdf", "r") as f:
                urdffile = f.read()
                try:
                    result = self.spawn_urdf(model_name=self.model_namespace,
                                             model_xml=urdffile,
                                             robot_namespace=self.model_namespace,
                                             initial_pose=ipose,
                                             reference_frame="world")
                    spawnflag = "Sucess"
                except rospy.ServiceException:
                    result = "/gazebo/SpawnModel service call failed"
                    count += 1
                    time.sleep(5)
        print(f"\n\n===========================\n{result}\n===========================\n")

        # -->SERACH FOR THE SDF FILE DESCRIBING THE WAYPOINT
        HOME = os.path.expanduser('~')
        files = glob.glob(os.path.join(HOME, f"**/*Yara_OVE/**/*wayPointMarker/model.sdf"), recursive=True)
        if len(files) > 0:
            sdffilepath = files[0]
            del (files)
        else:
            raise IOError(f"File wayPointMarker/model.sdf does not exist")

        # -->SPAWN THE WAYPOINT MARKER IN THE GAZEBO SIMULATION
        self.spawn_sdf = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        ipose.position.x = 100.0
        ipose.position.y = 0.0
        ipose.position.z = 0.0
        self.waypoint_namespace = f"wayPointMarker"
        with open(sdffilepath, "r") as f:
            sdffile = f.read()
            try:
                result = self.spawn_sdf(model_name=self.waypoint_namespace,
                                        model_xml=sdffile,
                                        robot_namespace=self.waypoint_namespace,
                                        initial_pose=ipose,
                                        reference_frame="world")
            except rospy.ServiceException:
                result = "/gazebo/SpawnModel service call failed"
            print(f"\n\n===========================\n{result}\n===========================\n")

        # -->DEFINE THE NECESSARY ROS TOPICS AND SERVICES
        self.boomAng_pub = rospy.Publisher(f"/{self.model_namespace}/control_interface/sail", Float32, queue_size=1)
        self.rudderAng_pub = rospy.Publisher(f"/{self.model_namespace}/control_interface/rudder", Float32,
                                             queue_size=1)
        self.enginePower_pub = rospy.Publisher(f"/{self.model_namespace}/control_interface/propulsion", Int16,
                                           queue_size=1)
        self.wind_pub = rospy.Publisher(f"/eboat/atmosferic_control/wind", Point, queue_size=1)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    def rot(self, modulus, theta):
        rot = np.array([[np.cos(theta), -np.sin(theta)],[np.sin(theta), np.cos(theta)]], dtype=np.float32)

        return np.dot(np.array([1, 0], dtype=np.float32) * modulus, rot)

    def getObservations(self):
        obsData = None
        while obsData is None:
            try:
                obsData = rospy.wait_for_message(f'/{self.model_namespace}/mission_control/observations', Float32MultiArray,
                                                 timeout=20).data
            except:
                pass

            # --> obsData: 0 distance from the goal,
            #              1 angle between the foward direction and the direction toward the goal
            #              2 surge velocity
            #              3 apparent wind speed,
            #              4 apparent wind angle,
            #              5 boom angle,
            #              6 rudder angle,
            #              7 eletric propulsion power,
            #              8 roll angle
            #              9 boat's current X position
            #             10 boat's current Y position

        return np.array(obsData, dtype=float)

    def rescaleObs(self, observations):
        lobs = len(observations)
        if lobs > 9:
            lobs -= 2
        robs = np.zeros(lobs, dtype=np.float32)
        # --> Distance from the waypoint (m) [0   , DMAX];
        robs[0] = 2 * (observations[0] / self.DMAX) - 1
        # --> Trajectory angle               [-180, 180]
        robs[1] = observations[1] / 180.0
        # --> Boat surge velocity (m/s)     [-10   , 10 ]
        robs[2] = observations[2] / 10
        # --> Aparent wind speed (m/s)       [0   , 30]
        robs[3] = observations[3] / 15 - 1
        # --> Apparent wind angle            [-180, 180]
        robs[4] = observations[4] / 180.0
        # --> Boom angle                     [0   , 90]
        robs[5] = (observations[5] / 45.0) - 1
        # --> Rudder angle                   [-60 , 60 ]
        robs[6] = observations[6] / 60.0
        # --> Electric propulsion speed      [-5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5]
        robs[7] = observations[7] / 5.0
        # --> Roll angle                     [-180, 180]
        robs[8] = observations[8] / 180.0

        return robs

    def setState(self, model_name, pose, theta):
        state = ModelState()
        state.model_name = model_name
        state.reference_frame = "world"
        # pose
        state.pose.position.x = pose[0]
        state.pose.position.y = pose[1]
        state.pose.position.z = pose[2]
        quaternion = quaternion_from_euler(0, 0, theta)
        state.pose.orientation.x = quaternion[0]
        state.pose.orientation.y = quaternion[1]
        state.pose.orientation.z = quaternion[2]
        state.pose.orientation.w = quaternion[3]
        # twist
        state.twist.linear.x = 0
        state.twist.linear.y = 0
        state.twist.linear.z = 0
        state.twist.angular.x = 0
        state.twist.angular.y = 0
        state.twist.angular.z = 0

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = self.set_state
            result = set_state(state)
            assert result.success is True
        except rospy.ServiceException:
            print("/gazebo/get_model_state service call failed")

class Eboat53_v0(EboatBase):
    def __init__(self):

        super().__init__()

        #-->DEFINE OBSERVATION AND ACTION SPACES
        self.action_space = spaces.Box(low   = -1  ,
                                       high  = 1   ,
                                       shape = (3,),
                                       dtype = np.float32)

        self.observation_space = spaces.Box(low   = -1  ,
                                            high  = 1   ,
                                            shape = (5,),
                                            dtype = np.float32)

        #-->SET WIND INITIAL CONDITIONS AND DEFINE ITS HOW IT WILL VARIATE
        self.windVec         = np.array([0, 0, 0], dtype=np.float32)
        # self.wind_speedVec   = np.array([5, 7, 9, 11, 12]) * 0.51444
        self.wind_speedVec = np.array([5, 6, 7, 8, 9, 10, 11, 12]) * 0.51444
        # self.wind_speedVec   = [12 * 0.51444]  #--> 12 knots (~6.17 m/s)
        self.wind_directions = np.concatenate([[-175], np.arange(-150, -5, 15), np.array([-5, 5]), np.arange(15, 151, 15), [175]])

        print(f"\n\n-------------------------------------------\n{self.wind_directions}\n-------------------------------------------\n")

        time.sleep(5)

        #--> UNPAUSE SIMULATION
        rospy.wait_for_service("/gazebo/unpause_physics")
        try:
            self.unpause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/unpause_physics service call failed!"))

        #-->GET OBSERVATIONS AT TIME 0 (INITIAL STATE)
        print("\n\n===========================\nGetting observations on the initial state (t=0)\n===========================\n")
        self.PREVOBS = None
        while (self.PREVOBS is None):
            try:
                self.PREVOBS = rospy.wait_for_message(f"/{self.model_namespace}/mission_control/observations", Float32MultiArray,
                                                      timeout=20).data
            except:
                pass

        #-->PAUSE SIMULATION
        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            self.pause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/pause_physics service call failed!"))

        #-->AUXILIARY VARIABLES
        self.DTOL          = 1.2
        self.D0            = 100.0
        self.DMAX          = self.DTOL * 100.0
        self.DLim          = self.DTOL * 100.0
        # self.S             = 0.0                           #--> TOTAL DISTANCE TRAVELED BY THE BOAT
        # self.preD          = 0.0                           #--> DISTANCE TRAVELED BY THE BOAT IN THE PREVIOUS STEP
        self.d2r           = np.pi / 180.0
        self.step_count    = 0
        self.lateral_limit = 0.6 * 100.0                   #--> DEFINE A TOLERANCE FOR HOW MUCH THE BOAT CAN TRAVEL AWY FROM THE STRAIGHT COURSE
        self.dS            = 0                             #--> TOTAL TRAVELED DISTANCE

    def repositionWayPoint(self, newdistance = None):
        if newdistance != None:
            # -->SERACH FOR THE SDF FILE DESCRIBING THE WAYPOINT
            HOME = os.path.expanduser('~')
            files = glob.glob(os.path.join(HOME, f"**/*Yara_OVE/**/*wayPointMarker/model.sdf"), recursive=True)
            if len(files) > 0:
                sdffilepath = files[0]
                del (files)
            else:
                raise IOError(f"File wayPointMarker/model.sdf does not exist")

            # -->SPAWN THE WAYPOINT MARKER IN THE GAZEBO SIMULATION
            # self.spawn_sdf = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
            ipose = Pose()
            ipose.position.x = newdistance
            ipose.position.y = 0.0
            ipose.position.z = 0.0
            # self.waypoint_namespace = f"wayPointMarker"
            delmodel = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
            rospy.wait_for_service("/gazebo/delete_model")
            result = delmodel(self.waypoint_namespace)
            with open(sdffilepath, "r") as f:
                sdffile = f.read()
                try:
                    result = self.spawn_sdf(model_name=self.waypoint_namespace,
                                            model_xml=sdffile,
                                            robot_namespace=self.waypoint_namespace,
                                            initial_pose=ipose,
                                            reference_frame="world")
                except rospy.ServiceException:
                    result = "/gazebo/SpawnModel service call failed"
                print(f"\n\n===========================\n{result}\n===========================\n")

            self.D0   = newdistance
            self.DMAX = self.DTOL * newdistance
            self.DLim = self.DTOL * newdistance

    def rewardFunction(self, obs):
        dS0  = self.PREVOBS[0] - obs[0]
        epwr = np.min([abs(obs[7]), 5.0])

        if obs[0] < 5.1:
            R = 1.0
        elif obs[0] > self.DMAX:
            R = -1.0
        # elif abs(obs[9]) > self.lateral_limit:
        #     R = -1.0
        # elif abs(obs[1]) > 90:
        #     R = -1.0
        elif dS0 > 0:
            R = (dS0 / self.DMAX) * (1.0 - 0.9 * (epwr / 5.0))
        else:
            R = (2.0 * (dS0 / self.DMAX)) - (0.01 * epwr)

        return R

    def step(self, action):
        act = np.array([((action[0] + 1) * 45.0), (action[1] * 60.0), (action[2] * 5)])
        #--> UNPAUSE SIMULATION
        rospy.wait_for_service("/gazebo/unpause_physics")
        try:
            self.unpause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/unpause_physics service call failed!"))

        #-->PUBLISH THE ACTIONS IN THE ROSTOPIC (SEND COMMANDS TO THE ACTUATORS)
        self.boomAng_pub.publish(act[0])
        self.rudderAng_pub.publish(act[1])
        self.enginePower_pub.publish(int(act[2]))

        #-->GET OBSERVATIONS
        obs = self.getObservations()

        #-->PAUSE SIMULATION
        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            self.pause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/pause_physics service call failed!"))

        #-->RESCALE EACH OBSERVATION TO THE INTERVAL [-1, 1]
        robs = self.rescaleObs(obs)[:5]

        #-->CALCULATES THE REWARD
        reward = self.rewardFunction(obs)

        #-->CHECK FOR A TERMINAL STATE
        done  = bool((obs[0] < 5.1) |
                     (obs[0] > self.DMAX) |
                     # (abs(obs[1]) > 90) |
                     # (abs(obs[9]) > self.lateral_limit) |
                     (np.isnan(obs).any()))
        trunc = bool(self.step_count > 300)

        # -->UPDATE PREVIOUS STATE VARIABLES
        self.PREVOBS     = obs
        self.step_count += 1

        return robs, reward, done, trunc, {}

    def reset(self, seed = None, options = None):
        # -->RESETS THE STATE OF THE ENVIRONMENT.
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print(("/gazebo/reset_simulation service call failed!"))

        # -->SET THE ACTUATORS BACK TO THE DEFAULT SETTING
        self.boomAng_pub.publish(0.0)
        self.rudderAng_pub.publish(0.0)
        self.enginePower_pub.publish(0)

        #-->SET A RANDOM INITIAL STATE FOR THE WIND
        if len(self.wind_directions) > 1:
            theta_wind = np.random.choice(self.wind_directions)
        else:
            theta_wind = self.wind_directions[0]

        if len(self.wind_speedVec) > 1:
            self.wind_speed = np.random.choice(self.wind_speedVec)
        else:
            self.wind_speed = self.wind_speedVec[0]
            
        self.windVec[:2] = self.rot(self.wind_speed, (theta_wind * self.d2r))
        self.wind_pub.publish(Point(self.windVec[0], self.windVec[1], self.windVec[2]))

        #-->UNPAUSE SIMULATION TO MAKE OBSERVATION
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/unpause_physics service call failed!"))

        #-->GET OBSERVATIONS
        obs = self.getObservations()

        #-->PAUSE SIMULATION
        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            self.pause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/pause_physics service call failed!"))

        #-->RESCALE EACH OBSERVATION TO THE INTERVAL [-1, 1]
        robs = self.rescaleObs(obs)[:5]

        #-->RESET INITIAL STATE VALUES
        self.PREVOBS    = obs
        self.step_count = 0
        # self.DLim       = self.DTOL * obs[0]

        return robs, {}

class Eboat103_v0(EboatBase):
    def __init__(self):

        # super().__init__()
        #---------------------------------------------------------------------------------------------------------------
        # -->DEFINE A UNIQUE MODEL NAME FOR OUR BOAT
        modelname = f"eboat4trOD"

        # -->SERACH FOR THE URDF FILE DESCRIBING THE BOAT
        HOME = os.path.expanduser('~')
        files = glob.glob(os.path.join(HOME, f"**/*Yara_OVE/**/*{modelname}.urdf.xacro"), recursive=True)
        if len(files) > 0:
            urdffilepath = files[0]
            del (files)
        else:
            raise IOError(f"File {modelname}.urdf.xacro does not exist")

        # -->TRANSFORM THE XACRO FILE TO URDF
        # subprocess.run(["xacro", urdffilepath, f"{modelname}.urdf"], capture_output=True)
        os.system(f"xacro {urdffilepath} > {modelname}.urdf")

        # -->SPAWN THE MODEL IN THE GAZEBO SIMULATION
        rospy.wait_for_service("gazebo/spawn_urdf_model")
        self.spawn_urdf = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
        ipose = Pose()
        ipose.position.x = 0.0
        ipose.position.y = 0.0
        ipose.position.z = 0.0
        self.model_namespace = f"eboat_{1}"
        count = 0
        spawnflag = "Fail"
        while (spawnflag == "Fail") & (count < 18):
            with open(f"{modelname}.urdf", "r") as f:
                urdffile = f.read()
                try:
                    result = self.spawn_urdf(model_name=self.model_namespace,
                                             model_xml=urdffile,
                                             robot_namespace=self.model_namespace,
                                             initial_pose=ipose,
                                             reference_frame="world")
                    spawnflag = "Sucess"
                except rospy.ServiceException:
                    result = "/gazebo/SpawnModel service call failed"
                    count += 1
                    time.sleep(5)
        print(f"\n\n===========================\n{result}\n===========================\n")

        # -->SERACH FOR THE SDF FILE DESCRIBING THE WAYPOINT
        HOME = os.path.expanduser('~')
        files = glob.glob(os.path.join(HOME, f"**/*Yara_OVE/**/*wayPointMarker/model.sdf"), recursive=True)
        if len(files) > 0:
            sdffilepath = files[0]
            del (files)
        else:
            raise IOError(f"File wayPointMarker/model.sdf does not exist")

        # -->SPAWN THE WAYPOINT MARKER IN THE GAZEBO SIMULATION
        rospy.wait_for_service("gazebo/spawn_sdf_model")
        self.spawn_sdf = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        ipose.position.x = 100.0
        ipose.position.y = 0.0
        ipose.position.z = 0.0
        self.waypoint_namespace = f"wayPointMarker"
        with open(sdffilepath, "r") as f:
            sdffile = f.read()
            try:
                result = self.spawn_sdf(model_name=self.waypoint_namespace,
                                        model_xml=sdffile,
                                        robot_namespace=self.waypoint_namespace,
                                        initial_pose=ipose,
                                        reference_frame="world")
            except rospy.ServiceException:
                result = "/gazebo/SpawnModel service call failed"
            print(f"\n\n===========================\n{result}\n===========================\n")

        # -->DEFINE THE NECESSARY ROS TOPICS AND SERVICES
        self.boomAng_pub = rospy.Publisher(f"/{self.model_namespace}/control_interface/sail", Float32, queue_size=1)
        self.rudderAng_pub = rospy.Publisher(f"/{self.model_namespace}/control_interface/rudder", Float32,
                                             queue_size=1)
        self.enginePower_pub = rospy.Publisher(f"/{self.model_namespace}/control_interface/propulsion", Int16,
                                               queue_size=1)
        self.wind_pub = rospy.Publisher(f"/eboat/atmosferic_control/wind", Point, queue_size=1)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.delmodel = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        #---------------------------------------------------------------------------------------------------------------

        # -->DEFINE OBSERVATION AND ACTION SPACES
        self.action_space = spaces.Box(low=-1,
                                       high=1,
                                       shape=(3,),
                                       dtype=np.float32)

        self.observation_space = spaces.Box(low=-1,
                                            high=1,
                                            shape=(10,),
                                            dtype=np.float32)

        # -->SET WIND INITIAL CONDITIONS AND DEFINE ITS HOW IT WILL VARIATE
        self.windVec         = np.array([0, 0, 0], dtype=np.float32)
        self.wind_speedVec   = np.array([9]) * 0.51444 #np.array([5, 6, 7, 8, 9, 10, 11, 12]) * 0.51444
        self.wind_directions = np.array([-150, -135, -90, -45, -5, 5, 45, 90, 135, 150]) #np.concatenate([np.arange(-150, -5, 15), np.array([-5, 5]), np.arange(15, 151, 15)])

        time.sleep(5)

        # --> UNPAUSE SIMULATION
        rospy.wait_for_service("/gazebo/unpause_physics")
        try:
            self.unpause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/unpause_physics service call failed!"))

        # -->GET OBSERVATIONS AT TIME 0 (INITIAL STATE)
        print(
            "\n\n===========================\nGetting observations on the initial state (t=0)\n===========================\n")
        self.PREVOBS = None
        while (self.PREVOBS is None):
            try:
                self.PREVOBS = rospy.wait_for_message(f"/{self.model_namespace}/mission_control/observations",
                                                      Float32MultiArray,
                                                      timeout=20).data
            except:
                pass

        # -->PAUSE SIMULATION
        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            self.pause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/pause_physics service call failed!"))

        # -->AUXILIARY VARIABLES
        self.DTOL = 25
        self.D0   = self.PREVOBS[0]
        self.DMAX = self.PREVOBS[0] + self.DTOL
        self.DLim = self.PREVOBS[0] + self.DTOL
        self.S    = 0.0  # --> TOTAL DISTANCE TRAVELED BY THE BOAT
        self.preD = 0.0  # --> DISTANCE TRAVELED BY THE BOAT IN THE PREVIOUS STEP
        self.d2r  = np.pi / 180.0
        self.step_count = 0
        self.lateral_limit = 0.6 * 100.0  # --> DEFINE A TOLERANCE FOR HOW MUCH THE BOAT CAN TRAVEL AWY FROM THE STRAIGHT COURSE
        self.dS = 0  # --> TRAVELED DISTANCE
        self.obsidx = [i for i in range(5)]+[i for i in range(9, 14)]

        #--> Auxliary variables for obstacle reposition
        self._opose  = Pose()
        self._opose.position.x = 50.0
        self._opose.position.y = 0.0
        self._opose.position.z = 0.0
        self._ostate = ModelState()
        self._ostate.model_name = "obstacle"
        self._ostate.reference_frame = "world"
        # pose
        self._ostate.pose = self._opose
        # twist
        self._ostate.twist.linear.x = 0
        self._ostate.twist.linear.y = 0
        self._ostate.twist.linear.z = 0
        self._ostate.twist.angular.x = 0
        self._ostate.twist.angular.y = 0
        self._ostate.twist.angular.z = 0

        #--> Find the models representing the obstacles
        HOME = os.path.expanduser('~')
        modelname = "box2"
        files = glob.glob(os.path.join(HOME, f"**/*Yara_OVE/**/{modelname}/model.sdf"), recursive=True)
        self.path2models = ""
        if len(files) > 0:
            self.path2models = files[0]
            del (files)
        else:
            raise IOError(f"File {modelname}/model.sdf does not exist")
        ipose = Pose()
        ipose.position.x = np.random.randint(low=40, high=80, size=1)
        ipose.position.y = np.random.randint(low=-5, high=5, size=1)
        ipose.position.z = 0.0
        self.spawnSDFModel("obstacle", self.path2models, ipose)

        #--> Distance detection rays
        rospy.Subscriber("/eboat/laser/scan", LaserScan, self._laser_scan_callback)

        self.laser_scan = np.zeros(5, dtype=int)
        rospy.logdebug("Waiting for /scan to be READY...")
        while ((self.laser_scan is None) and (not rospy.is_shutdown())):
            try:
                self.laser_scan = rospy.wait_for_message("/eboat/laser/scan", LaserScan, timeout=1.0)
                rospy.logdebug("Current /eboat/laser/scan READY=>")
            except:
                rospy.logerr("Current /eboat/laser/scan not ready yet, retrying for getting laser_scan")
        #-------------------------------------------------------------------------------------------------------

    def _laser_scan_callback(self, data):
        laser_ranges = np.asarray(data.ranges)
        laser_ranges[laser_ranges == np.inf] = data.range_max

        self.laser_scan[4] = np.min(laser_ranges[0:23])
        self.laser_scan[3] = np.min(laser_ranges[24:47])
        self.laser_scan[2] = np.min(laser_ranges[48:72])
        self.laser_scan[1] = np.min(laser_ranges[73:96])
        self.laser_scan[0] = np.min(laser_ranges[97:120])

    def spawnSDFModel(self, model_namespace, descriptor_file_path, ipose=None):
        # -->SPAWN THE WAYPOINT MARKER IN THE GAZEBO SIMULATION
        rospy.wait_for_service("gazebo/spawn_sdf_model")
        spawn_sdf = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        if ipose == None:
            ipose = Pose()
            ipose.position.x = 60.0
            ipose.position.y = 0.0
            ipose.position.z = 0.0

        with open(descriptor_file_path, "r") as f:
            sdffile = f.read()

            try:
                result = spawn_sdf(model_name=model_namespace,
                                   model_xml=sdffile,
                                   robot_namespace=model_namespace,
                                   initial_pose=ipose,
                                   reference_frame="world")
            except rospy.ServiceException:
                result = "/gazebo/SpawnModel service call failed"
            print(f"\n\n----------------------------------------\n{descriptor_file_path}\n{result}\n{result.success}\n----------------------------------------\n")

    def rescaleObs(self, observations, dzones):
        lobs = len(observations)
        llas = len(dzones)
        if lobs > 9:
            lobs -= 2
        robs = np.zeros(lobs + llas, dtype=np.float32)
        # --> Distance from the waypoint (m) [0   , DMAX];
        robs[0] = 2 * (observations[0] / self.DMAX) - 1
        # --> Trajectory angle               [-180, 180]
        robs[1] = observations[1] / 180.0
        # --> Boat surge velocity (m/s)     [-10   , 10 ]
        robs[2] = observations[2] / 10
        # --> Aparent wind speed (m/s)       [0   , 30]
        robs[3] = observations[3] / 15 - 1
        # --> Apparent wind angle            [-180, 180]
        robs[4] = observations[4] / 180.0
        # --> Boom angle                     [0   , 90]
        robs[5] = (observations[5] / 45.0) - 1
        # --> Rudder angle                   [-60 , 60 ]
        robs[6] = observations[6] / 60.0
        # --> Electric propulsion speed      [-5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5]
        robs[7] = observations[7] / 5.0
        # --> Roll angle                     [-180, 180]
        robs[8] = observations[8] / 180.0
        # --> Distance zones for obstacle detection
        robs[9:] = dzones / 30.0 - 1

        return robs

    def rewardFunction(self, obs, dfo):
        dS0 = self.PREVOBS[0] - obs[0]
        epwr = np.min([abs(obs[7]), 5.0])

        if obs[0] < 5.1:
            R = 1.0
        elif obs[0] > self.DMAX:
            R = -1.0
        elif (np.min(dfo[[0, 4]]) < 2) | (np.min(dfo[[1, 2, 3]]) < 5):
            R = -1.0
        elif dS0 > 0:
            R = (dS0 / self.DMAX) * (1.0 - 0.9 * (epwr / 5.0))
        else:
            R = (2.0 * (dS0 / self.DMAX)) - (0.01 * epwr)

        return R

    def step(self, action):
        act = np.array([((action[0] + 1) * 45.0), (action[1] * 60.0), (action[2] * 5)])
        # --> UNPAUSE SIMULATION
        rospy.wait_for_service("/gazebo/unpause_physics")
        try:
            self.unpause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/unpause_physics service call failed!"))

        # -->PUBLISH THE ACTIONS IN THE ROSTOPIC (SEND COMMANDS TO THE ACTUATORS)
        self.boomAng_pub.publish(act[0])
        self.rudderAng_pub.publish(act[1])
        self.enginePower_pub.publish(int(act[2]))

        # -->GET OBSERVATIONS
        obs = self.getObservations()
        dfo = self.laser_scan.copy()

        # -->PAUSE SIMULATION
        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            self.pause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/pause_physics service call failed!"))

        # -->RESCALE EACH OBSERVATION TO THE INTERVAL [-1, 1]
        robs = self.rescaleObs(obs, dfo)[self.obsidx]
        # print("\n----------------------------\n", robs.shape, robs)
        # print("=======\n", self.laser_scan.shape, self.laser_scan)

        # -->CALCULATES THE REWARD
        reward = self.rewardFunction(obs, dfo)

        # -->CHECK FOR A TERMINAL STATE
        done = bool((obs[0] < 5.1) |
                    (obs[0] > self.DMAX) |
                    (np.min(dfo[[0, 4]]) < 2) |
                    (np.min(dfo[[1, 2, 3]]) < 5) |
                    (np.isnan(obs).any()))
        trunc = bool(self.step_count > 300)

        # -->UPDATE PREVIOUS STATE VARIABLES
        self.PREVOBS = obs
        self.step_count += 1

        return robs, reward, done, trunc, {}

    def reset(self, seed=None, options=None):
        # -->RESETS THE STATE OF THE ENVIRONMENT.
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print(("/gazebo/reset_simulation service call failed!"))

        # -->SET THE ACTUATORS BACK TO THE DEFAULT SETTING
        self.boomAng_pub.publish(0.0)
        self.rudderAng_pub.publish(0.0)
        self.enginePower_pub.publish(0)

        # -->SET A RANDOM INITIAL STATE FOR THE WIND
        if len(self.wind_directions) > 1:
            theta_wind = np.random.choice(self.wind_directions)
        else:
            theta_wind = self.wind_directions[0]

        if len(self.wind_speedVec) > 1:
            self.wind_speed = np.random.choice(self.wind_speedVec)
        else:
            self.wind_speed = self.wind_speedVec[0]

        self.windVec[:2] = self.rot(self.wind_speed, (theta_wind * self.d2r))
        self.wind_pub.publish(Point(self.windVec[0], self.windVec[1], self.windVec[2]))

        #--> Spawn obstacle in a random position within the expected trajectory path
        # rospy.wait_for_service("/gazebo/delete_model")
        # result = self.delmodel("obstacle")
        self._opose.position.y = np.random.randint(low=-5, high=7, size=1)
        if self._opose.position.y > 5:
            self._opose.position.x = -300
            self._opose.position.y = 100
        else:
            self._opose.position.x = np.random.randint(low=40, high=81, size=1)
        # print(f"position.x = {self._opose.position.x}")
        # print(f"position.y = {self._opose.position.y}\n----------------------")
        # self.spawnSDFModel("obstacle", self.path2models, ipose)
        self._ostate.pose = self._opose
        rospy.wait_for_service('/gazebo/set_model_state')
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        try:
            result = set_state(self._ostate)
            assert result.success is True
        except rospy.ServiceException:
            print("/gazebo/set_model_state service call failed")

        # -->UNPAUSE SIMULATION TO MAKE OBSERVATION
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/unpause_physics service call failed!"))

        # -->GET OBSERVATIONS
        obs = self.getObservations()
        dfo = self.laser_scan.copy()

        # -->PAUSE SIMULATION
        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            self.pause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/pause_physics service call failed!"))

        # -->RESCALE EACH OBSERVATION TO THE INTERVAL [-1, 1]
        robs = self.rescaleObs(obs, dfo)[self.obsidx]

        # -->RESET INITIAL STATE VALUES
        self.PREVOBS = obs
        self.step_count = 0
        self.DLim = self.D0 + self.DTOL

        return robs, {}

class Eboat101_v0(Eboat103_v0):
    def __init__(self):
        # ---------------------------------------------------------------------------------------------------------------
        # -->DEFINE A UNIQUE MODEL NAME FOR OUR BOAT
        modelname = f"eboat4trOD"

        # -->SERACH FOR THE URDF FILE DESCRIBING THE BOAT
        HOME = os.path.expanduser('~')
        files = glob.glob(os.path.join(HOME, f"**/*Yara_OVE/**/*{modelname}.urdf.xacro"), recursive=True)
        if len(files) > 0:
            urdffilepath = files[0]
            del (files)
        else:
            raise IOError(f"File {modelname}.urdf.xacro does not exist")

        # -->TRANSFORM THE XACRO FILE TO URDF
        os.system(f"xacro {urdffilepath} > {modelname}.urdf")

        # -->SPAWN THE MODEL IN THE GAZEBO SIMULATION
        rospy.wait_for_service("gazebo/spawn_urdf_model")
        self.spawn_urdf = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
        ipose = Pose()
        ipose.position.x = 0.0
        ipose.position.y = 0.0
        ipose.position.z = 0.0
        self.model_namespace = f"eboat_{1}"
        count = 0
        spawnflag = "Fail"
        while (spawnflag == "Fail") & (count < 18):
            with open(f"{modelname}.urdf", "r") as f:
                urdffile = f.read()
                try:
                    result = self.spawn_urdf(model_name=self.model_namespace,
                                             model_xml=urdffile,
                                             robot_namespace=self.model_namespace,
                                             initial_pose=ipose,
                                             reference_frame="world")
                    spawnflag = "Sucess"
                except rospy.ServiceException:
                    result = "/gazebo/SpawnModel service call failed"
                    count += 1
                    time.sleep(5)
        print(f"\n\n===========================\n{result}\n===========================\n")

        # -->SERACH FOR THE SDF FILE DESCRIBING THE WAYPOINT
        HOME = os.path.expanduser('~')
        files = glob.glob(os.path.join(HOME, f"**/*Yara_OVE/**/*wayPointMarker/model.sdf"), recursive=True)
        if len(files) > 0:
            sdffilepath = files[0]
            del (files)
        else:
            raise IOError(f"File wayPointMarker/model.sdf does not exist")

        # -->SPAWN THE WAYPOINT MARKER IN THE GAZEBO SIMULATION
        rospy.wait_for_service("gazebo/spawn_sdf_model")
        self.spawn_sdf = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        ipose.position.x = 100.0
        ipose.position.y = 0.0
        ipose.position.z = 0.0
        self.waypoint_namespace = f"wayPointMarker"
        with open(sdffilepath, "r") as f:
            sdffile = f.read()
            try:
                result = self.spawn_sdf(model_name=self.waypoint_namespace,
                                        model_xml=sdffile,
                                        robot_namespace=self.waypoint_namespace,
                                        initial_pose=ipose,
                                        reference_frame="world")
            except rospy.ServiceException:
                result = "/gazebo/SpawnModel service call failed"
            print(f"\n\n===========================\n{result}\n===========================\n")

        # -->DEFINE THE NECESSARY ROS TOPICS AND SERVICES
        self.boomAng_pub     = rospy.Publisher(f"/{self.model_namespace}/control_interface/sail"      , Float32, queue_size=1)
        self.rudderAng_pub   = rospy.Publisher(f"/{self.model_namespace}/control_interface/rudder"    , Float32, queue_size=1)
        self.enginePower_pub = rospy.Publisher(f"/{self.model_namespace}/control_interface/propulsion", Int16  , queue_size=1)
        self.wind_pub        = rospy.Publisher(f"/eboat/atmosferic_control/wind"                      , Point  , queue_size=1)
        self.unpause         = rospy.ServiceProxy('/gazebo/unpause_physics' , Empty)
        self.pause           = rospy.ServiceProxy('/gazebo/pause_physics'   , Empty)
        self.reset_proxy     = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.set_state       = rospy.ServiceProxy('/gazebo/set_model_state' , SetModelState)
        self.get_state       = rospy.ServiceProxy('/gazebo/get_model_state' , GetModelState)
        self.delmodel        = rospy.ServiceProxy("/gazebo/delete_model"    , DeleteModel)
        # ---------------------------------------------------------------------------------------------------------------

        # -->DEFINE OBSERVATION AND ACTION SPACES
        self.action_space = spaces.Box(low=-1,
                                       high=1,
                                       shape=(1,),
                                       dtype=np.float32)

        self.observation_space = spaces.Box(low=-1,
                                            high=1,
                                            shape=(10,),
                                            dtype=np.float32)

        # -->SET WIND INITIAL CONDITIONS AND DEFINE ITS HOW IT WILL VARIATE
        self.windVec         = np.array([0, 0, 0], dtype=np.float32)
        self.wind_speedVec   = np.array([5, 6, 7, 8, 9, 10, 11, 12]) * 0.51444
        self.wind_directions = np.array([-150, -135, -90, -45, -5, 5, 45, 90, 135, 150])
        # self.wind_directions = np.concatenate([np.arange(-150, -5, 15), np.array([-5, 5]), np.arange(15, 151, 15)])

        time.sleep(5)

        # --> UNPAUSE SIMULATION
        rospy.wait_for_service("/gazebo/unpause_physics")
        try:
            self.unpause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/unpause_physics service call failed!"))

        # -->GET OBSERVATIONS AT TIME 0 (INITIAL STATE)
        print(
            "\n\n===========================\nGetting observations on the initial state (t=0)\n===========================\n")
        self.PREVOBS = None
        while (self.PREVOBS is None):
            try:
                self.PREVOBS = rospy.wait_for_message(f"/{self.model_namespace}/mission_control/observations",
                                                      Float32MultiArray,
                                                      timeout=20).data
            except:
                pass

        # -->PAUSE SIMULATION
        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            self.pause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/pause_physics service call failed!"))

        # -->AUXILIARY VARIABLES
        self.DTOL          = 25
        self.D0            = self.PREVOBS[0]
        self.DMAX          = self.PREVOBS[0] + self.DTOL
        self.DLim          = self.PREVOBS[0] + self.DTOL
        self.S             = 0.0  # --> TOTAL DISTANCE TRAVELED BY THE BOAT
        self.preD          = 0.0  # --> DISTANCE TRAVELED BY THE BOAT IN THE PREVIOUS STEP
        self.d2r           = np.pi / 180.0
        self.step_count    = 0
        self.lateral_limit = 5  # --> DEFINE A TOLERANCE FOR HOW MUCH THE BOAT CAN TRAVEL AWY FROM THE STRAIGHT COURSE
        self.dS            = 0  # --> TRAVELED DISTANCE
        self.obsidx        = [i for i in range(5)] + [i for i in range(9, 14)]

        # --> Auxliary variables for obstacle reposition
        self._opose = Pose()
        self._opose.position.x = 50.0
        self._opose.position.y = 0.0
        self._opose.position.z = 0.0
        self._ostate = ModelState()
        self._ostate.model_name = "obstacle"
        self._ostate.reference_frame = "world"
        # pose
        self._ostate.pose = self._opose
        # twist
        self._ostate.twist.linear.x = 0
        self._ostate.twist.linear.y = 0
        self._ostate.twist.linear.z = 0
        self._ostate.twist.angular.x = 0
        self._ostate.twist.angular.y = 0
        self._ostate.twist.angular.z = 0

        # --> Find the models representing the obstacles
        HOME = os.path.expanduser('~')
        modelname = "box2"
        files = glob.glob(os.path.join(HOME, f"**/*Yara_OVE/**/{modelname}/model.sdf"), recursive=True)
        self.path2models = ""
        if len(files) > 0:
            self.path2models = files[0]
            del (files)
        else:
            raise IOError(f"File {modelname}/model.sdf does not exist")
        ipose = Pose()
        ipose.position.x = np.random.randint(low=40, high=80, size=1)
        ipose.position.y = np.random.randint(low=-5, high=5, size=1)
        ipose.position.z = 0.0
        self.spawnSDFModel("obstacle", self.path2models, ipose)

        # --> Distance detection rays
        rospy.Subscriber("/eboat/laser/scan", LaserScan, self._laser_scan_callback)

        self.laser_scan = np.zeros(5, dtype=int)
        rospy.logdebug("Waiting for /scan to be READY...")
        while ((self.laser_scan is None) and (not rospy.is_shutdown())):
            try:
                self.laser_scan = rospy.wait_for_message("/eboat/laser/scan", LaserScan, timeout=1.0)
                rospy.logdebug("Current /eboat/laser/scan READY=>")
            except:
                rospy.logerr("Current /eboat/laser/scan not ready yet, retrying for getting laser_scan")
        # -------------------------------------------------------------------------------------------------------

    def rewardFunction(self, obs, dfo):
        dS0 = self.PREVOBS[0] - obs[0]
        epwr = np.min([abs(obs[7]), 5.0])

        if obs[0] < 5.1:
            R = 1.0
        elif obs[0] > self.DMAX:
            R = -1.0
        elif (np.min(dfo[[0, 4]]) < 2) | (np.min(dfo[[1, 2, 3]]) < 5):
            R = -1.0
        elif abs(obs[10]) > self.lateral_limit:
            R = -1.0
        elif dS0 > 0:
            R = (dS0 / self.DMAX) #* (1.0 - 0.9 * (epwr / 5.0))
        else:
            R = (2.0 * (dS0 / self.DMAX)) #- (0.01 * epwr)

        return R

    def step(self, action):
        # --> UNPAUSE SIMULATION
        rospy.wait_for_service("/gazebo/unpause_physics")
        try:
            self.unpause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/unpause_physics service call failed!"))

        # -->PUBLISH THE ACTIONS IN THE ROSTOPIC (SEND COMMANDS TO THE ACTUATORS)
        self.boomAng_pub.publish(abs((abs(self.PREVOBS[4]) / 2) - 90.0))
        self.rudderAng_pub.publish(action[0] * 60.0)

        # -->GET OBSERVATIONS
        obs = self.getObservations()
        dfo = self.laser_scan.copy()

        # -->PAUSE SIMULATION
        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            self.pause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/pause_physics service call failed!"))

        # -->RESCALE EACH OBSERVATION TO THE INTERVAL [-1, 1]
        robs = self.rescaleObs(obs, dfo)[self.obsidx]

        # -->CALCULATES THE REWARD
        reward = self.rewardFunction(obs, dfo)

        # -->CHECK FOR A TERMINAL STATE
        done = bool((obs[0] < 5.1) |
                    (obs[0] > self.DMAX) |
                    (np.min(dfo[[0, 4]]) < 2) |
                    (np.min(dfo[[1, 2, 3]]) < 5) |
                    (np.isnan(obs).any()))
        trunc = bool((self.step_count > 300) |
                     (abs(obs[10]) > self.lateral_limit))

        # -->UPDATE PREVIOUS STATE VARIABLES
        self.PREVOBS = obs
        self.step_count += 1

        return robs, reward, done, trunc, {}

    def reset(self, seed=None, options=None):
        # -->RESETS THE STATE OF THE ENVIRONMENT.
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print(("/gazebo/reset_simulation service call failed!"))

        # -->SET THE ACTUATORS BACK TO THE DEFAULT SETTING
        self.boomAng_pub.publish(0.0)
        self.rudderAng_pub.publish(0.0)
        # self.enginePower_pub.publish(0)

        # -->SET A RANDOM INITIAL STATE FOR THE WIND
        if len(self.wind_directions) > 1:
            theta_wind = np.random.choice(self.wind_directions)
        else:
            theta_wind = self.wind_directions[0]

        if len(self.wind_speedVec) > 1:
            wds = np.random.choice(self.wind_speedVec)
        else:
            wds = self.wind_speedVec[0]

        self.windVec[:2] = self.rot(wds, (theta_wind * self.d2r))
        self.wind_pub.publish(Point(self.windVec[0], self.windVec[1], self.windVec[2]))

        #--> Spawn obstacle in a random position within the expected trajectory path
        # rospy.wait_for_service("/gazebo/delete_model")
        # result = self.delmodel("obstacle")
        self._opose.position.y = np.random.randint(low=-5, high=7, size=1)
        if self._opose.position.y > 5:
            self._opose.position.x = -300
            self._opose.position.y = 100
        else:
            self._opose.position.x = np.random.randint(low=40, high=81, size=1)
        # print(f"position.x = {self._opose.position.x}")
        # print(f"position.y = {self._opose.position.y}\n----------------------")
        # self.spawnSDFModel("obstacle", self.path2models, ipose)
        self._ostate.pose = self._opose
        rospy.wait_for_service('/gazebo/set_model_state')
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        try:
            result = set_state(self._ostate)
            assert result.success is True
        except rospy.ServiceException:
            print("/gazebo/set_model_state service call failed")

        # -->UNPAUSE SIMULATION TO MAKE OBSERVATION
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/unpause_physics service call failed!"))

        # -->GET OBSERVATIONS
        obs = self.getObservations()
        dfo = self.laser_scan.copy()

        # -->PAUSE SIMULATION
        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            self.pause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/pause_physics service call failed!"))

        # -->RESCALE EACH OBSERVATION TO THE INTERVAL [-1, 1]
        robs = self.rescaleObs(obs, dfo)[self.obsidx]

        # -->RESET INITIAL STATE VALUES
        self.PREVOBS = obs
        self.step_count = 0
        self.DLim = self.D0 + self.DTOL

        return robs, {}

# if __name__ == "__main__":
#     test = Eboat925SternWindv0()