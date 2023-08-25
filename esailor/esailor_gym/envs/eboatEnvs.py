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
from geometry_msgs.msg import Point
from gymnasium.utils import seeding

from tf.transformations import quaternion_from_euler
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelState

import random
from rosgraph_msgs.msg import Clock

from std_srvs.srv import Empty

from std_msgs.msg import Float32, Int16, Float32MultiArray
from geometry_msgs.msg import Point, Pose

from tf.transformations import quaternion_from_euler
from gazebo_msgs.srv import SetModelState, SpawnModel, DeleteModel
from gazebo_msgs.msg import ModelState

class Eboat925SternWindv0(gym.Env):
    def __init__(self):
        # print(f"\n----------------------\n{ros_port}\n----------------------\n")

        #-->SET ROS CONFIG TO INTERACT WITH THE GAZEBO SIMULATION
        # current_env_port = os.environ["ROS_MASTER_URI"].split(":")[2]
        # if current_env_port != ros_port:
        #     os.environ["ROS_MASTER_URI"] = f"http://localhost:{ros_port}"

        #-->INITIALIZE A ROS NODE FOR THE TRAINING PROCESS
        try:
            rospy.init_node(f"gym", anonymous=False)
        except:
            print("ROSMASTER is not running!")
            print(time.time())
            exit(1)

        #-->DEFINE A UNIQUE MODEL NAME FOR OUR BOAT
        modelname = f"eboat4tr"

        #-->SERACH FOR THE URDF FILE DESCRIBING THE BOAT
        HOME = os.path.expanduser('~')
        files = glob.glob(os.path.join(HOME, f"**/*Yara_OVE/**/*{modelname}.urdf.xacro"), recursive=True)
        if len(files) > 0:
            urdffilepath = files[0]
            del (files)
        else:
            raise IOError(f"File {modelname}.urdf.xacro does not exist")

        #-->TRANSFORM THE XACRO FILE TO URDF
        #subprocess.run(["xacro", urdffilepath, f"{modelname}.urdf"], capture_output=True)
        os.system(f"xacro {urdffilepath} > {modelname}.urdf")


        #-->SPAWN THE MODEL IN THE GAZEBO SIMULATION
        self.spawn_urdf = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
        ipose = Pose()
        ipose.position.x = 0.0
        ipose.position.y = 0.0
        ipose.position.z = 0.0
        self.model_namespace  = f"eboat_{1}"
        count = 0
        spawnflag = "Fail"
        while (spawnflag == "Fail") & (count < 18):
            with open(f"{modelname}.urdf", "r") as f:
                urdffile = f.read()
                try:
                    result = self.spawn_urdf(model_name      = self.model_namespace,
                                             model_xml       = urdffile,
                                             robot_namespace = self.model_namespace,
                                             initial_pose    = ipose,
                                             reference_frame = "world")
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
                result = self.spawn_sdf(model_name      = self.waypoint_namespace,
                                        model_xml       = sdffile,
                                        robot_namespace = self.waypoint_namespace,
                                        initial_pose    = ipose,
                                        reference_frame = "world")
            except rospy.ServiceException:
                result = "/gazebo/SpawnModel service call failed"
            print(f"\n\n===========================\n{result}\n===========================\n")

        #-->DEFINE THE NECESSARY ROS TOPICS AND SERVICES
        self.boomAng_pub   = rospy.Publisher(f"/{self.model_namespace}/control_interface/sail", Float32, queue_size=5)
        self.rudderAng_pub = rospy.Publisher(f"/{self.model_namespace}/control_interface/rudder", Float32, queue_size=5)
        self.propVel_pub   = rospy.Publisher(f"/{self.model_namespace}/control_interface/propulsion", Int16, queue_size=5)
        self.wind_pub      = rospy.Publisher(f"/eboat/atmosferic_control/wind", Point, queue_size=5)
        self.unpause       = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause         = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy   = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.set_state     = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.get_state     = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        #-->DEFINE OBSERVATION AND ACTION SPACES
        self.action_space = spaces.Box(low=-1,
                                       high=1,
                                       shape=(2,),
                                       dtype=np.float32)

        self.observation_space = spaces.Box(low=-1,
                                            high=1,
                                            shape=(9,),
                                            dtype=np.float32)

        #-->SET WIND INITIAL CONDITIONS AND DEFINE ITS HOW IT WILL VARIATE
        self.windVec = np.array([0, 0, 0], dtype=np.float32)
        self.min_windspeed = 3
        self.max_windspeed = 11
        self.wind_directions = np.array([0])

        #--> UNPAUSE SIMULATION
        rospy.wait_for_service("/gazebo/unpause_physics")
        try:
            self.unpause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/unpause_physics service call failed!"))

        #-->GET OBSERVATIONS AT TIME STAMP 0 (INITIAL STATE)
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
        self.DTOL = 25
        self.D0   = self.PREVOBS[0]
        self.DMAX = self.PREVOBS[0] + self.DTOL
        self.d2r  = np.pi / 180.0
        self.step_count = 0
        self.lateral_limit = 5 #-->DEFINE THE HOW MUCH THE BOAT CAN TRAVEL AWY FROM THE STRAIGHT LINE BEFORE A DONE SIGNAL

    def rot(self, modulus, theta):
        R = np.array([[np.cos(theta), -np.sin(theta)],[np.sin(theta), np.cos(theta)]], dtype=np.float32)

        return np.dot(np.array([1, 0], dtype=np.float32) * modulus, R)

    def getObservations(self):
        obsData = None
        while obsData is None:
            try:
                obsData = rospy.wait_for_message(f'/{self.model_namespace}/mission_control/observations', Float32MultiArray,
                                                 timeout=20).data
            except:
                pass
            # --> obsData: 0 distance from the goal,
            #              1 angle between the foward direction and the direction towards the goal
            #              2 surge velocity
            #              3 apparent wind speed,
            #              4 apparent wind angle,
            #              5 boom angle,
            #              6 rudder angle,
            #              7 eletric propultion power,
            #              8 roll angle
            #             10 boat's current X position
            #             11 boat's current Y position

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
        # --> Boat linear velocity (m/s)     [-10   , 10 ]
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

    def rewardFunction(self, obs, actions):
        D = (self.PREVOBS[0] - obs[0]) / self.D0

        return D

    def step(self, action):
        ract = np.array([(action[0]+1)*45, action[1]*60])
        print(self.step_count, ract)
        #--> UNPAUSE SIMULATION
        rospy.wait_for_service("/gazebo/unpause_physics")
        try:
            self.unpause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/unpause_physics service call failed!"))

        #-->PUBLISH THE ACTIONS IN THE ROSTOPIC (SEND COMMANDS TO THE ACTUATORS)
        self.boomAng_pub.publish((action[0] + 1) * 45.0)
        self.rudderAng_pub.publish(action[1] * 60.0)

        #-->GET OBSERVATIONS
        obs = self.getObservations()

        #-->PAUSE SIMULATION
        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            self.pause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/pause_physics service call failed!"))

        #-->RESCALE EACH OBSERVATION TO THE INTERVAL [-1, 1]
        robs = self.rescaleObs(obs)

        #-->CALCULATES THE REWARD
        reward = self.rewardFunction(obs, action)

        #-->CHECK FOR A TERMINAL STATE
        posY    = obs[10]
        windAng = abs(obs[4])
        done    = bool((obs[0] <= 5) |
                       (obs[0] > self.PREVOBS[0]) |
                       (abs(posY) > self.lateral_limit) |
                       ((windAng >= 160) & (windAng <= 200) & (obs[2] < 0.5)) |  # -->A done signal is returned if the wind is blowing from the bow and the surge velocity is smaller than 0.5 m/s
                       (self.step_count > 59) |
                       (np.isnan(obs).any())
                      )

        # -->PROCESS DONE SIGNAL
        if done:
            if (obs[0] <= 5):
                reward = 1
            elif (not (np.isnan(obs).any())):
                reward = -1
            else:
                pass
        else:
            # -->UPDATE PREVIOUS STATE VARIABLES
            self.PREVOBS = obs

        self.step_count += 1

        return robs, reward, done, False, {}

    def reset(self, seed = None, options = None):
        # -->RESETS THE STATE OF THE ENVIRONMENT.
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print(("/gazebo/reset_simulation service call failed!"))

        # -->SET THE ACTUATORS BACK TO THE DEFAULT SETTING
        self.propVel_pub.publish(0)
        self.boomAng_pub.publish(0.0)
        self.rudderAng_pub.publish(0.0)

        #-->SET A RANDOM INITIAL STATE FOR THE WIND
        wind_speed = np.random.randint(low=self.min_windspeed, high=self.max_windspeed)
        if len(self.wind_directions) > 1:
            theta_wind = np.random.choice(self.wind_directions)
        else:
            theta_wind = self.wind_directions[0]
        self.windVec[:2] = self.rot(wind_speed, (theta_wind * self.d2r))
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
        robs = self.rescaleObs(obs)

        #-->RESET INITIAL STATE VALUES
        self.PREVOBS    = obs
        self.step_count = 0

        return robs, {}

# if __name__ == "__main__":
#     test = Eboat925SternWindv0()