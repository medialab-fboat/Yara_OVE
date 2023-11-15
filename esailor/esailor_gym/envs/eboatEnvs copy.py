from raycast_updated import rays
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

class EboatBase(gym.Env):
    def __init__(self):
        # -->DEFINE A UNIQUE MODEL NAME FOR OUR BOAT
        modelname = f"eboat4tr"

         #-->SERACH FOR THE URDF FILE DESCRIBING THE BOAT
        """ HOME = os.path.expanduser('~')
        files = glob.glob(os.path.join(HOME, f"**/*Yara_OVE/**/*{modelname}.urdf.xacro"), recursive=True)
        if len(files) > 0:
            urdffilepath = files[0]
            del (files)
        else:
            raise IOError(f"File {modelname}.urdf.xacro does not exist") """
            
        urdffilepath = "/home/araujo/yara_ws/src/Yara_OVE/eboat_description/urdf/eboat4tr.urdf.xacro"

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
        self.propVel_pub = rospy.Publisher(f"/{self.model_namespace}/control_interface/propulsion", Int16,
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

class Eboat925_v0(EboatBase):
    def __init__(self):
        # print(f"\n----------------------\n{ros_port}\n----------------------\n")

        #-->SET ROS CONFIG TO INTERACT WITH THE GAZEBO SIMULATION
        # current_env_port = os.environ["ROS_MASTER_URI"].split(":")[2]
        # if current_env_port != ros_port:
        #     os.environ["ROS_MASTER_URI"] = f"http://localhost:{ros_port}"

        #-->INITIALIZE A ROS NODE FOR THE TRAINING PROCESS
        # try:
        #     rospy.init_node(f"gym", anonymous=True)
        # except:
        #     print("ROSMASTER is not running!")
        #     print(time.time())
        #     exit(1)

        super().__init__()

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
        self.min_windspeed   = 3
        self.max_windspeed   = 11
        self.wind_directions = np.array([-135, -90, -45, -5, 5, 45, 90, 135])
        self.wind_speed      = 0.0

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
        self.DTOL          = 25
        self.D0            = self.PREVOBS[0]
        self.DMAX          = self.PREVOBS[0] + self.DTOL
        self.PREdS         = 0.0                           #--> DISTANCE TRAVELED BY THE BOAT TOWARDS THE GOAL (D(t = n) - D(t = n-1))
        self.d2r           = np.pi / 180.0
        self.step_count    = 0
        self.lateral_limit = 5                             #-->DEFINE THE HOW MUCH THE BOAT CAN TRAVEL AWY FROM THE STRAIGHT LINE BEFORE A DONE SIGNAL
        self.PREVACT       = np.array([-1, -1])
        self.maxcharge     = 10
        self.battery       = np.full(2, fill_value=self.maxcharge, dtype=int)

    def rewardFunction0(self, obs, actions):
        #-->COMPUTE THE DISTANCE TRAVELED BY THE BOAT TOWARDS THE OBJECTIVE. IF D > 0 THE BOAT WENT NEAR THE OBJECTIVE.
        dS  = self.PREVOBS[0] - obs[0]
        ta  = abs(obs[1])
        pta = abs(self.PREVOBS[1])
        C0  = dS / self.D0;

        if dS > 0:
            if dS > self.PREdS:
                C1 = 0.5 * C0
            elif dS < self.PREdS:
                C1 = -0.6 * C0
            else:
                C1 = 0.0

            if ta <= 45.0:
                if ta < pta:
                # if pta - ta > 1:
                    C2 = 0.5 * C0
                # elif ta > pta:
                elif ta - pta > 1:
                    C2 = -0.6 * C0
                else:
                    if ta < 5:
                        C2 = 0.5 * C0
                    else:
                        C2 = 0.0
            else:
                if ta < pta:
                # if pta - ta > 1:
                    C2 = 0.7 * C0
                # elif ta > pta:
                elif ta - pta > 1:
                    C2 = -1.5 * C0
                else:
                    C2 = -0.5 * C0

            # alpha = 180 - obs[5] - abs(obs[4])
            # print(f"alpha = {alpha}")
            # if ((dS > self.PREdS) & (ta <= 45) & (obs[2] > 0.25 * self.wind_speed)):
            #     C3 = 0.6 * C0
            # else:
            #     C3 = 0.0

            R = C0 + C1 + C2 #+ C3
            # print(f"dS(t)   = {dS} = {self.PREVOBS[0]} - {obs[0]}")
            # print(f"dS(t-1) = {self.PREdS}")
            # print(f"ta      = {ta}")
            # print(f"pta     = {pta}")
            # print(f"return  = {C0} + {C1} + {C2}  = {R}")
            # print(obs[5])
            # print(obs[6])
            # print("-----------------------------------")
        else:
            R = C0
            # print(f"return  = {C0}  = {R}")

        self.PREdS = dS
        return R

    def rewardFunction1(self, obs):
        #-->COMPUTE THE DISTANCE TRAVELED BY THE BOAT TOWARDS THE OBJECTIVE. IF D > 0 THE BOAT WENT NEAR THE OBJECTIVE.
        dS  = self.PREVOBS[0] - obs[0]
        ta  = abs(obs[1])
        pta = abs(self.PREVOBS[1])
        C0  = dS / self.D0;

        if dS > 0:
            if dS > self.PREdS:
                C1 = 0.5 * C0
                self.PREdS = dS
            elif dS < self.PREdS:
                C1 = -0.6 * C0
            else:
                C1 = 0.0

            if ta <= 45.0:
                if ta < pta:
                    C2 = 0.5 * C0
                elif ta - pta > 1:
                    C2 = -0.6 * C0
                else:
                    if ta < 5:
                        C2 = 0.5 * C0
                    else:
                        C2 = 0.0
            else:
                if ta < pta:
                    C2 = 0.7 * C0
                elif ta > pta:
                    C2 = -1.5 * C0
                else:
                    C2 = -0.7 * C0

            alpha = 180 - obs[5] - abs(obs[4])
            if (alpha > 150) & (abs(obs[5] - self.PREVOBS[5]) < 30):
                C3 = (-1) * (((C0 > 0) * C0) + ((C1 > 0) * C1) + ((C2 > 0) * C2) + 0.06)
            else:
                C3 = 0.0

            # if ((dS > self.PREdS) & (ta <= 45) & (obs[2] > 0.25 * self.wind_speed)):
            #     C4 = 0.6 * C0
            # else:
            #     C4 = 0.0

            R = C0 + C1 + C2 + C3
        else:
            R = C0
            # print(f"return  = {C0}  = {R}")

        return R

    def rewardFunctionA(self, obs, action):
        #-->COMPUTE THE DISTANCE TRAVELED BY THE BOAT TOWARDS THE OBJECTIVE. IF D > 0 THE BOAT WENT NEAR THE OBJECTIVE.
        dS    = self.PREVOBS[0] - obs[0]
        ta    = abs(obs[1])
        pta   = abs(self.PREVOBS[1])
        db    = abs((self.PREVACT[0] - action[0]) * 45.0) #abs(self.PREVOBS[5] - (action[0] + 1.0) * 45.0) #--> DIFFERENCE BETWEEN THE POSITION OF THE BOOM AND THE NEW POSITION OF THE BOOM REQUIRED BY THE AGENT
        dr    = abs((self.PREVACT[1] - action[1]) * 60.0) #abs(self.PREVOBS[6] - (action[1] * 60.0))       #--> DIFFERENCE BETWEEN THE POSITION OF THE RUDDER AND THE NEW POSITION OF THE RUDDER REQUIRED BY THE AGENT
        alpha = 180 - obs[5] - abs(obs[4])                      #--> Sail attack angle
        C0  = dS / self.D0;

        if dS > 0:
            C1 = ((dS > self.PREdS) * 0.5 - (dS < self.PREdS) * 0.6) * C0

            if ta <= 45.0:
                if ta < pta:
                    C2 = 0.5 * C0
                elif ta - pta > 1:
                    C2 = -0.6 * C0
                else:
                    if ta < 5:
                        C2 = 0.5 * C0
                    else:
                        C2 = 0.0
            else:
                if ta < pta:
                    C2 = 0.7 * C0
                elif ta > pta:
                    C2 = -1.5 * C0
                else:
                    C2 = -0.7 * C0

            C3 = (alpha > 150) * (((150 - alpha) / 30.0) - 0.5) * C0

            if ((ta <= 45) & (obs[2] > 0.25 * self.wind_speed)):
                C4 = 0.6 * C0
            elif obs[2] < 0.5:
                # C4 = (-1.0) * C0 * ((self.step_count / 60.0) + 0.5)
                C4 = (-0.15) * (self.step_count / 60.0)
                self.step_count += 1
            else:
                C4 = 0.0

            ad = C0 + C1 + C2 + C3 + C4
            if ad > 0:
                C5 = ((((db > 1.0) + (dr > 1.0)) * (-0.15)) + (((db > 28.0) + (dr > 23.0)) * (-0.45))) * ad
            else:
                # C5 = ((db > 28.0) + (dr > 23.0)) * 0.15 * ad
                C5 = (1.0 + (((db > 1.0) + (dr > 1.0)) * 0.1) + (((db > 28.0) + (dr > 23.0)) * 0.2)) * ad

            R = C0 + C1 + C2 + C3 + C4 + C5
            # print(f"db = {self.PREVACT[0]} - {action[0]} = {db}")
            # print(f"dr = {dr}")
            # print("R   = {:7.5f} + {:7.5f} + {:7.5f} + {:7.5f} + {:7.5f} + {:7.5f} = {:7.5f}".format(C0, C1, C2, C3, C4, C5, R))
        else:
            R = C0

        return np.max([R, -1])

    def rewardFunctionB(self, obs, action): #-->rewardFunctionB (INATIVA)
        dS    = self.PREVOBS[0] - obs[0]
        ta    = abs(obs[1])
        pta   = abs(self.PREVOBS[1])
        db    = abs(self.PREVOBS[5] - (action[0] + 1.0) * 45.0) #--> DIFFERENCE BETWEEN THE POSITION OF THE BOOM AND THE NEW POSITION OF THE BOOM REQUIRED BY THE AGENT
        dr    = abs(self.PREVOBS[6] - (action[1] * 60.0))       #--> DIFFERENCE BETWEEN THE POSITION OF THE RUDDER AND THE NEW POSITION OF THE RUDDER REQUIRED BY THE AGENT
        alpha = 180 - obs[5] - abs(obs[4])                      #--> Sail attack angle

        #--> POSITIVE/NEGATIVE RETURN EARNED BY GO NEAR/AWAY THE WAYPOINT
        C0    = dS / self.D0;

        if dS > 0:
            if ta <= 90.0:
                C1 = (np.cos(ta * self.d2r)**3) * C0
                if (obs[2] > 0.4 * self.wind_speed):
                    C2 = 1.5 * C0
                elif (obs[2] > 0.333333333 * self.wind_speed):
                    C2 = 1.2 * C0
                elif (obs[2] > 0.25 * self.wind_speed):
                    C2 = 0.8 * C0
                elif obs[2] < 0.6:
                    C2 = (-0.15) * (self.step_count / 60.0)
                    self.step_count += 1
                else:
                    C2 = 0.0
            elif (ta < pta):
                C1 = 0.7 * C0
                C2 = 0.0
            else:
                C1 = -2.0 * C0
                C2 = 0.0
            #########################################
            if alpha > 150:
                C3 = (((150 - alpha) / 30.0) - 1.0) * C0
            elif alpha < 5:
                C3 = (-1.0) * C0
            else:
                C3 = 0.0
            #########################################
            C4 = ((C0 + C1 + C2 + C3) *
                  ((((db > 0) * np.min([1, db/28.0]) * 0.1) + ((dr > 0) * np.min([1, dr/23.0]) * 0.1)) +
                   (((db > 28) * (db / 90.0) * 0.3) + ((dr > 0) * np.min([1, dr/90.0]) * 0.3))))
            #########################################
            R = C0 + C1 + C2 + C3 + C4
        elif dS == 0:
            R = (-0.15) * (self.step_count / 60.0)
            self.step_count += 1
        else:
            R = np.min([-5.0, ((-0.15) * (self.step_count / 60.0))]) * C0

        return np.max([R, -1])

    def rewardFunctionC(self, obs, action): #-->rewardFunctionC (INATIVA)
        dS    = self.PREVOBS[0] - obs[0]
        ta    = abs(obs[1])
        pta   = abs(self.PREVOBS[1])
        db    = abs(self.PREVOBS[5] - (action[0] + 1.0) * 45.0) #--> DIFFERENCE BETWEEN THE POSITION OF THE BOOM AND THE NEW POSITION OF THE BOOM REQUIRED BY THE AGENT
        dr    = abs(self.PREVOBS[6] - (action[1] * 60.0))       #--> DIFFERENCE BETWEEN THE POSITION OF THE RUDDER AND THE NEW POSITION OF THE RUDDER REQUIRED BY THE AGENT
        alpha = 180 - obs[5] - abs(obs[4])                      #--> Sail attack angle

        #--> POSITIVE/NEGATIVE RETURN EARNED BY GO NEAR/AWAY THE WAYPOINT
        C0    = dS / self.D0;

        if dS > 0:
            if ta <= 90.0:
                C1 = (np.cos(ta * self.d2r)**3) * C0
                if (obs[2] > 0.4 * self.wind_speed):
                    C2 = 1.0 * C0
                elif (obs[2] > 0.333333333 * self.wind_speed):
                    C2 = 0.7 * C0
                elif (obs[2] > 0.25 * self.wind_speed):
                    C2 = 0.4 * C0
                elif obs[2] < 0.6:
                    C2 = (-0.15) * (self.step_count / 60.0)
                    self.step_count += 1
                else:
                    C2 = 0.0

                if dS > self.PREdS:
                    C2 *= 1.5
                    self.PREdS = dS  # --> UPDATE THE PREdS GLOBAL VARIABLE (PREdS stores the maximum dS actived by the boat during an episode)
            elif (ta < pta):
                C1 = 0.7 * C0
                C2 = 0.0
            else:
                C1 = -2.0 * C0
                C2 = 0.0
            #########################################
            if alpha > 150:
                C3 = (((150 - alpha) / 30.0) - 1.0) * C0
            elif alpha < 5:
                C3 = (-1.0) * C0
            else:
                C3 = 0.0
            #########################################
            C4                   = C0 + C1 + C2 + C3
            boom_energy_factor   = ((db > 0) * np.min([1.0, db / 28.0]) * 0.1) + ((db > 28) * (db / 90.0) * 0.3)
            rudder_energy_factor = ((dr > 0) * np.min([1.0, dr / 23.0]) * 0.1) + ((dr > 23) * np.min([1, dr / 90.0]) * 0.3)
            if C4 > 0:
                C4 *= (-1.0)*(boom_energy_factor + rudder_energy_factor)
            else:
                C4 *= boom_energy_factor + rudder_energy_factor
            #########################################
            R = C0 + C1 + C2 + C3 + C4
        elif dS == 0:
            R = (-0.15) * (self.step_count / 60.0)
            self.step_count += 1
        else:
            R = np.min([-5.0, ((-0.15) * (self.step_count / 60.0))]) * C0

        return np.max([R, -1])

    def rewardFunction(self, obs, action): #-->rewardFunctionD (ATIVA)
        # QUais sao as funcoes de truncamento e finalizacao dos episodios?
        dS    = self.PREVOBS[0] - obs[0]
        ta    = abs(obs[1])
        pta   = abs(self.PREVOBS[1])
        db    = abs(self.PREVOBS[5] - (action[0] + 1.0) * 45.0) #--> DIFFERENCE BETWEEN THE POSITION OF THE BOOM AND THE NEW POSITION OF THE BOOM REQUIRED BY THE AGENT
        dr    = abs(self.PREVOBS[6] - (action[1] * 60.0))       #--> DIFFERENCE BETWEEN THE POSITION OF THE RUDDER AND THE NEW POSITION OF THE RUDDER REQUIRED BY THE AGENT
        alpha = 180 - obs[5] - abs(obs[4])                      #--> Sail attack angle

        #--> POSITIVE/NEGATIVE RETURN EARNED BY GO NEAR/AWAY THE WAYPOINT
        C0 = dS / self.D0;

        if dS > 0.5:
            if ta <= 90.0:
                C1 = (np.cos(ta * self.d2r)**3) * C0
                if (obs[2] > 0.4 * self.wind_speed):
                    C2 = 1.0 * C0
                elif (obs[2] > 0.333333333 * self.wind_speed):
                    C2 = 0.7 * C0
                elif (obs[2] > 0.25 * self.wind_speed):
                    C2 = 0.4 * C0
                elif obs[2] < 0.6:
                    C2 = (-0.15) * (self.step_count / 60.0)
                    self.step_count += 1
                else:
                    C2 = 0.0

                if dS > self.PREdS:
                    C2 *= 1.5
                    self.PREdS = dS  # --> UPDATE THE PREdS GLOBAL VARIABLE (PREdS stores the maximum dS actived by the boat during an episode)
            elif (ta < pta):
                C1 = 0.7 * C0
                C2 = 0.0
            else:
                C1 = -2.0 * C0
                C2 = 0.0
            #########################################
            if alpha > 150:
                C3 = (((150 - alpha) / 30.0) - 1.0) * C0
            elif alpha < 5:
                C3 = (-1.0) * C0
            else:
                C3 = 0.0
            #########################################
            C4                   = C0 + C1 + C2 + C3
            # self.battery[0]     -= (db > 0.0) * 1.0 + (db > 28.0) * 1.0
            # self.battery[1]     -= (dr > 0.0) * 1.0 + (dr > 23.0) * 1.0
            boom_position_factor = 0.6 * (((action[0] + 1.0) * 45.0) < 5.0)
            boom_energy_factor   = ((db > 0) * np.min([1.0, db / 28.0]) * 0.1) + ((db > 28) * (db / 90.0) * 0.3)
            rudder_energy_factor = ((dr > 0) * np.min([1.0, dr / 23.0]) * 0.1) + ((dr > 23) * np.min([1, dr / 90.0]) * 0.3)
            if C4 > 0:
                C4 *= (-1.0)*(boom_position_factor + boom_energy_factor + rudder_energy_factor)
            else:
                C4 *= boom_position_factor + boom_energy_factor + rudder_energy_factor
            #########################################
            R = C0 + C1 + C2 + C3 + C4
        elif dS >= 0:
            R = (-0.15) * (self.step_count / 60.0)
            self.step_count += 1
        else:
            R = np.min([-5.0, ((-0.15) * (self.step_count / 60.0))]) * C0

        return np.max([R, -1])

    def step(self, action):
        # ract = np.array([(action[0]+1)*45, action[1]*60])
        # print(self.step_count, ract)
        #--> UNPAUSE SIMULATION
        rospy.wait_for_service("/gazebo/unpause_physics")
        try:
            self.unpause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/unpause_physics service call failed!"))

        #-->PUBLISH THE ACTIONS IN THE ROSTOPIC (SEND COMMANDS TO THE ACTUATORS)
        self.boomAng_pub.publish((action[0] + 1) * 45.0)
        self.rudderAng_pub.publish(action[1] * 60.0)
        # self.propVel_pub.publish(0) values -5 to 5
        # self.propVel_pub.publish(action[2] * 5) transform to integer values -5 to 5
        # self.propVel_pub.publish(int(action[2] * 5)) transform to integer values -5 to 5

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
                       # (self.step_count > 59) |
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
            self.PREVACT = action[0], action[1]

        # self.step_count += 1

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
        self.wind_speed = np.random.randint(low=self.min_windspeed, high=self.max_windspeed)
        if len(self.wind_directions) > 1:
            theta_wind = np.random.choice(self.wind_directions)
        else:
            theta_wind = self.wind_directions[0]
        self.windVec[:2] = self.rot(self.wind_speed, (theta_wind * self.d2r))
        self.wind_pub.publish(Point(self.windVec[0], self.windVec[1], self.windVec[2]))

        #-->TURN BOTA 45o WHEN THE SAILING POINT IS IN "NO GO" ZONE.
        # if theta_wind > 150:
        #     self.setState(self.model_namespace, [0.0, 0.0, 0.0], 0.7854)
        # elif theta_wind < -150:
        #     self.setState(self.model_namespace, [0.0, 0.0, 0.0], -0.7854)

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
        self.PREdS      = 0.0
        self.PREVACT    = 0.0, 0.0
        self.step_count = 0
        self.battery    = self.maxcharge, self.maxcharge

        return robs, {}

