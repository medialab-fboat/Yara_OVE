import gymnasium as gym
import rospy
import numpy as np
import sys, os, signal, subprocess
import time

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

class GazeboEnv(gym.Env):
    """Superclass for all Gazebo environments.
    """
    metadata = {'render.modes': ['human']}

    def __init__(self, launchfile):
        self.last_clock_msg = Clock()

        random_number = random.randint(10000, 15000)
        # self.port = "11311"#str(random_number) #os.environ["ROS_PORT_SIM"]
        # self.port_gazebo = "11345"#str(random_number+1) #os.environ["ROS_PORT_SIM"]
        self.port = str(random_number) #os.environ["ROS_PORT_SIM"]
        self.port_gazebo = str(random_number+1) #os.environ["ROS_PORT_SIM"]

        os.environ["ROS_MASTER_URI"] = "http://localhost:"+self.port
        os.environ["GAZEBO_MASTER_URI"] = "http://localhost:"+self.port_gazebo
        #
        # self.ros_master_uri = os.environ["ROS_MASTER_URI"];

        print("ROS_MASTER_URI=http://localhost:"+self.port + "\n")
        print("GAZEBO_MASTER_URI=http://localhost:"+self.port_gazebo + "\n")

        # self.port = os.environ.get("ROS_PORT_SIM", "11311")
        ros_path = os.path.dirname(subprocess.check_output(["which", "roscore"]))

        # NOTE: It doesn't make sense to launch a roscore because it will be done when spawing Gazebo, which also need
        #   to be the first node in order to initialize the clock.
        # # start roscore with same python version as current script
        # self._roscore = subprocess.Popen([sys.executable, os.path.join(ros_path, b"roscore"), "-p", self.port])
        # time.sleep(1)
        # print ("Roscore launched!")


        if launchfile.startswith("/"):
            fullpath = launchfile
        else:
            fullpath = os.path.join(os.path.dirname(__file__), "assets", "launch", launchfile)
        if not os.path.exists(fullpath):
            raise IOError("File "+fullpath+" does not exist")

        self._roslaunch = subprocess.Popen([sys.executable, os.path.join(ros_path, b"roslaunch"), "-p", self.port, fullpath])
        print ("Gazebo launched!")

        self.gzclient_pid = 0

        # Launch the simulation with the given launchfile name
        rospy.init_node('gym', anonymous=True)

        ################################################################################################################
        # r = rospy.Rate(1)
        # self.clock_sub = rospy.Subscriber('/clock', Clock, self.callback, queue_size=1000000)
        # while not rospy.is_shutdown():
        #     print("initialization: ", rospy.rostime.is_rostime_initialized())
        #     print("Wallclock: ", rospy.rostime.is_wallclock())
        #     print("Time: ", time.time())
        #     print("Rospyclock: ", rospy.rostime.get_rostime().secs)
        #     # print("/clock: ", str(self.last_clock_msg))
        #     last_ros_time_ = self.last_clock_msg
        #     print("Clock:", last_ros_time_)
        #     # print("Waiting for synch with ROS clock")
        #     # if wallclock == False:
        #     #     break
        #     r.sleep()
        ################################################################################################################

    # def callback(self, message):
    #     """
    #     Callback method for the subscriber of the clock topic
    #     :param message:
    #     :return:
    #     """
    #     # self.last_clock_msg = int(str(message.clock.secs) + str(message.clock.nsecs)) / 1e6
    #     # print("Message", message)
    #     self.last_clock_msg = message
    #     # print("Message", message)

    def step(self, action):

        # Implement this method in every subclass
        # Perform a step in gazebo. E.g. move the robot
        raise NotImplementedError

    def reset(self, seed = None):

        # Implemented in subclass
        raise NotImplementedError

    def _render(self, mode="human", close=False):

        if close:
            tmp = os.popen("ps -Af").read()
            proccount = tmp.count('gzclient')
            if proccount > 0:
                if self.gzclient_pid != 0:
                    os.kill(self.gzclient_pid, signal.SIGTERM)
                    os.wait()
            return

        tmp = os.popen("ps -Af").read()
        proccount = tmp.count('gzclient')
        if proccount < 1:
            subprocess.Popen("gzclient")
            self.gzclient_pid = int(subprocess.check_output(["pidof","-s","gzclient"]))
        else:
            self.gzclient_pid = 0

    def close(self):

        ppid = self._roslaunch.pid
        print(f"\n\n===================\nProcess id: {ppid}\n===================\n")
        os.system(f'ps -au eduardo | grep {self._roslaunch.pid}')
        # self._roslaunch.kill()
        # self._roslaunch.terminate()
        os.killpg(os.getpgid(self._roslaunch.pid), signal.SIGTERM)

        print("\n\n\nCLOSE FUNCTION\n\n")

    def _configure(self):

        # TODO
        # From OpenAI API: Provides runtime configuration to the enviroment
        # Maybe set the Real Time Factor?
        pass
    def _seed(self):

        # TODO
        # From OpenAI API: Sets the seed for this env's random number generator(s)
        pass

class EboatSingleWayPointEnvCC35v0(GazeboEnv):
    def __init__(self):
        self.EBOAT_HOME = "/home/eduardo/USVSim/yara_ws/src/Yara_OVE"
        GazeboEnv.__init__(self, os.path.join(self.EBOAT_HOME, "eboat_gazebo/launch/ocean_RL_training.launch"))

        self.boomAng_pub   = rospy.Publisher("/eboat/control_interface/sail", Float32, queue_size=5)
        self.rudderAng_pub = rospy.Publisher("/eboat/control_interface/rudder", Float32, queue_size=5)
        self.propVel_pub   = rospy.Publisher("/eboat/control_interface/propulsion", Int16, queue_size=5)
        self.wind_pub      = rospy.Publisher("/eboat/atmosferic_control/wind", Point, queue_size=5)
        self.unpause       = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause         = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy   = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.set_state     = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        # --> GLOBAL VARIABLES
        self.DTOL  = 25.0  # --> Threshold for distance. If the boat goes far than INITIAL POSITION + DMAX, a done signal is trigged.
        self.D0    = None  # --> The intial distance from the waypoint
        self.DMAX  = None
        self.DPREV = None

        # --> We will use a rescaled action space
        self.action_space = spaces.Box(low=-1,
                                       high=1,
                                       shape=(3,),
                                       dtype=np.float32)

        # --> We will use a rescaled action space
        self.observation_space = spaces.Box(low=-1,
                                            high=1,
                                            shape=(5,),
                                            dtype=np.float32)
        self.reward_range = (-1, 1)

        self._seed()

        # --> SET WIND SPEED INITIAL VECTOR
        self.windSpeed = np.array([0.0, 9.0, 0.0], dtype=np.float32)

        time.sleep(20)

        # --> UNPAUSE SIMULATION
        rospy.wait_for_service("/gazebo/unpause_physics")
        try:
            self.unpause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/unpause_physics service call failed!"))

        # --> GET INITIAL DISTANCE FROM THE WAYPOINT
        while self.D0 is None:
            try:
                self.D0 = \
                rospy.wait_for_message("/eboat/mission_control/observations", Float32MultiArray, timeout=20).data[0]
                self.DMAX = self.D0 + self.DTOL  # --> IT TAKE THE INITIAL DISTANCE IN CONSIDERATION
                self.DPREV = self.D0
            except:
                pass

        # --> AUXILIARY VARS
        self.d2r = np.pi / 180.0
        self.count = 0
        self.step_count = 0

        # --> SUPPORT FOR RANDOM INITIALIZATION OF WIND SPEED AND DIRECTION
        np.random.seed(30)
        self.min_wind_speed  = 3
        self.max_wind_speed  = 12

        # --> LOG FILE TO REGISTER INITIAL CONDITIONS (BOAT ORIENTATION, APPARENT WIND SPEED, APPARENT WIND ANGLE)
        sufix = "_" + datetime.now().strftime("%d%m%Y_%H_%M_%S")
        self.state_log_file = os.path.join(os.getcwd(), states_log, f"STATES_{sufix}.log")
        with open(self.state_log_file, "w") as f:
            f.write("bang,wspeed,wang\n")

    def rot(self, modulus, theta):
        R = np.array([[np.cos(theta), -np.sin(theta)],[np.sin(theta), np.cos(theta)]], dtype=np.float32)

        return np.dot(np.array([1, 0], dtype=np.float32) * modulus, R)

    def setWindSpeed(self, vector):
        self.windSpeed = vector

    def setMaxWindSpeed(self, value):
        self.max_wind_speed = value

    def getWindSpeed(self):
        return self.windSpeed

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

    def sampleInitialState(self, model_name):
        theta_boat = np.random.randint(low=-179, high=180)
        wind_speed = np.random.randint(low=self.min_wind_speed, high=self.max_wind_speed)
        theta_wind = np.random.randint(low=-179, high=180)

        # -->Set the true wind vector
        self.windSpeed[:2] = self.rot(wind_speed, (theta_wind * self.d2r))
        with open(self.state_log_file, "a") as f:
            f.write(f"{theta_boat}, {wind_speed}, {theta_wind}\n")

        # --> Set the boat's position and orientation
        self.setState(model_name, pose=np.zeros(shape=3, dtype=np.float32), theta=(theta_boat * self.d2r))

    def getObservations(self):
        obsData = None
        while obsData is None:
            try:
                obsData = rospy.wait_for_message('/eboat/mission_control/observations', Float32MultiArray,
                                                 timeout=20).data
            except:
                pass
            # --> obsData = [distance, trajectory angle, linear velocity, aparent wind speed, aparent wind angle, boom angle, rudder angle, eletric propultion speed, roll angle]
            #               [   0    ,        1        ,       2        ,         3         ,         4         ,     5     ,      6      ,            7            ,     8     ]

        return np.array(obsData, dtype=float)

    def actionRescale(self, action):
        raction = np.zeros(3, dtype=np.float32)
        # --> Eletric propulsion [-5, 5]
        raction[0] = action[0] * 5.0
        # --> Boom angle [0, 90]
        raction[1] = (action[1] + 1) * 45.0
        # --> Rudder angle [-60, 60]
        raction[2] = action[2] * 60.0
        return raction

    def rescale(self, m, rmin, rmax, tmin, tmax):
        # rmin denote the minimum of the range of your measurement
        # rmax denote the maximum of the range of your measurement
        # tmin denote the minimum of the range of your desired target scaling
        # tmax denote the maximum of the range of your desired target scaling
        # m in [rmin,rmax] denote your measurement to be scaled
        # Then
        # m --> ((m−rmin)/(rmax−rmin))*(tmax-tmin)+tmin
        # will scale m linearly into [tmin,tmax] as desired.
        # To go step by step,
        # m --> m−rmin maps m to [0,rmax−rmin].
        # Next,
        # m --> (m−rmin)/(rmax−rmin)
        # maps m to the interval [0,1], with m=rmin mapped to 0 and m=rmax mapped to 1.
        # Multiplying this by (tmax−tmin) maps m to [0,tmax−tmin].
        # Finally, adding tmin shifts everything and maps m to [tmin,tmax] as desired.
        return (((m - rmin) / (rmax - rmin)) * (tmax - tmin) + tmin)

    def observationRescale(self, observations):
        lobs = len(observations)
        robs = np.zeros(lobs, dtype=np.float32)
        # --> Distance from the waypoint (m) [0   , DMAX];
        robs[0] = 2 * (observations[0] / self.DMAX) - 1
        # --> Trajectory angle               [-180, 180]
        robs[1] = observations[1] / 180.0
        # --> Boat linear velocity (m/s)     [0   , 10 ]
        robs[2] = observations[2] / 5 - 1
        # --> Aparent wind speed (m/s)       [0   , 30]
        robs[3] = observations[3] / 15 - 1
        # --> Apparent wind angle            [-180, 180]
        robs[4] = observations[4] / 180.0
        if lobs > 5:
            # --> Boom angle                     [0   , 90]
            robs[5] = (observations[5] / 45.0) - 1
            # --> Rudder angle                   [-60 , 60 ]
            robs[6] = observations[6] / 60.0
            # --> Electric propulsion speed      [-5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5]
            robs[7] = observations[7] / 5.0
            # --> Roll angle                     [-180, 180]
            robs[8] = observations[8] / 180.0

        return robs

    def rewardFunction(self, obs):
        reward = (self.DPREV - obs[0]) / self.DMAX

        if reward > 0:
            reward *= (1.0 - 0.9 * abs(obs[7]) / 5.0)
        else:
            reward -= 0.01 * abs(obs[7])

        # --> obsData = [distance, trajectory angle, linear velocity, aparent wind speed, aparent wind angle, boom angle, rudder angle, eletric propultion speed, roll angle]
        #               [   0    ,        1        ,       2        ,         3         ,         4         ,     5     ,      6      ,            7            ,     8     ]

        return reward

    def step(self, action):
        # --> UNPAUSE SIMULATION
        rospy.wait_for_service("/gazebo/unpause_physics")
        try:
            self.unpause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/unpause_physics service call failed!"))

        # -->SEND ACTION TO THE BOAT CONTROL INTERFACE
        ract = self.actionRescale(action)
        self.propVel_pub.publish(int(ract[0]))
        self.boomAng_pub.publish(ract[1])
        self.rudderAng_pub.publish(ract[2])

        # -->GET OBSERVATIONS (NEXT STATE)
        observations = self.getObservations()

        # -->PAUSE SIMULATION
        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            self.pause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/pause_physics service call failed!"))

        # -->CALCULATES THE REWARD
        reward = self.rewardFunction(observations)

        # -->UPDATE PREVIOUS STATE VARIABLES
        self.DPREV = observations[0]

        # -->CHECK FOR A TERMINAL STATE
        done = bool((self.DPREV <= 5) |
                    (self.DPREV > self.DMAX) |
                    (np.isnan(observations).any())
                    )

        if np.isnan(observations).any():
            print("\n\n-------------------------------------")
            print(f"distance: {observations[0]}")
            print(f"traj ang: {observations[1]}")
            print(f"boat vel: {observations[2]}")
            print(f"wind vel: {observations[3]}")
            print(f"wind ang: {observations[4]}")
            print(f"boom ang: {observations[5]}")
            print(f"rud ang : {observations[6]}")
            print(f"prop    : {observations[7]}")
            print(f"roll ang: {observations[8]}")
            print("-------------------------------------\n")
            # --> WAIT FOR ACKNOWLEDGEMENT FROM USER
            # _ = input("Unpause: ")

        # -->PROCESS DONE SIGNAL
        if done:
            if (self.DPREV <= 5):
                reward = 1
            else:
                reward = -1

        self.step_count += 1

        return self.observationRescale(observations[:5]), reward, done, False, {}

    def reset(self, seed = None):
        # -->RESETS THE STATE OF THE ENVIRONMENT.
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print(("/gazebo/reset_simulation service call failed!"))

        # -->UNPAUSE SIMULATION TO MAKE OBSERVATION
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/unpause_physics service call failed!"))

        # -->SET THE ACTUATORS BACK TO THE DEFAULT SETTING
        self.propVel_pub.publish(0)
        self.boomAng_pub.publish(0.0)
        self.rudderAng_pub.publish(0.0)

        # -->SET RANDOM INITIAL STATE
        self.sampleInitialState("eboat")
        self.wind_pub.publish(Point(self.windSpeed[0], self.windSpeed[1], self.windSpeed[2]))

        # -->COLLECT OBSERVATIONS
        observations = self.getObservations()

        # -->RESET INITIAL STATE VALUES
        self.DPREV = observations[0]

        # -->PAUSE SIMULATION
        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            self.pause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/pause_physics service call failed!"))

        self.count += 1

        return self.observationRescale(observations[:5]), {}

class EboatSingleWayPointEnvCC25v0(EboatSingleWayPointEnvCC35v0):
    def __init__(self):
        self.EBOAT_HOME = "/home/eduardo/USVSim/yara_ws/src/Yara_OVE"
        GazeboEnv.__init__(self, os.path.join(self.EBOAT_HOME, "eboat_gazebo/launch/ocean_RL_training.launch"))

        self.boomAng_pub   = rospy.Publisher("/eboat/control_interface/sail", Float32, queue_size=5)
        self.rudderAng_pub = rospy.Publisher("/eboat/control_interface/rudder", Float32, queue_size=5)
        self.propVel_pub   = rospy.Publisher("/eboat/control_interface/propulsion", Int16, queue_size=5)
        self.wind_pub      = rospy.Publisher("/eboat/atmosferic_control/wind", Point, queue_size=5)
        self.unpause       = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause         = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy   = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.set_state     = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        # --> GLOBAL VARIABLES
        self.DTOL  = 25.0  # --> Threshold for distance. If the boat goes far than INITIAL POSITION + DMAX, a done signal is trigged.
        self.D0    = None  # --> The intial distance from the waypoint
        self.DMAX  = None
        self.DPREV = None

        # --> We will use a rescaled action space
        self.action_space = spaces.Box(low=-1,
                                       high=1,
                                       shape=(2,),
                                       dtype=np.float32)

        # --> We will use a rescaled action space
        self.observation_space = spaces.Box(low=-1,
                                            high=1,
                                            shape=(5,),
                                            dtype=np.float32)
        self.reward_range = (-1, 1)

        self._seed()

        # --> SET WIND SPEED INITIAL VECTOR
        self.windSpeed = np.array([0.0, 9.0, 0.0], dtype=np.float32)

        # --> UNPAUSE SIMULATION
        rospy.wait_for_service("/gazebo/unpause_physics")
        try:
            self.unpause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/unpause_physics service call failed!"))

        # --> GET INITIAL DISTANCE FROM THE WAYPOINT
        while self.D0 is None:
            try:
                self.D0 = \
                rospy.wait_for_message("/eboat/mission_control/observations", Float32MultiArray, timeout=20).data[0]
                self.DMAX = self.D0 + self.DTOL  # --> IT TAKE THE INITIAL DISTANCE IN CONSIDERATION
                self.DPREV = self.D0
            except:
                pass

        # --> AUXILIARY VARS
        self.d2r = np.pi / 180.0
        self.count = 0
        self.step_count = 0

        # --> SUPPORT FOR RANDOM INITIALIZATION OF WIND SPEED AND DIRECTION
        np.random.seed(30)
        self.min_wind_speed  = 3
        self.max_wind_speed  = 12

        # --> LOG FILE TO REGISTER INITIAL CONDITIONS (BOAT ORIENTATION, APPARENT WIND SPEED, APPARENT WIND ANGLE)
        sufix = "_" + datetime.now().strftime("%d%m%Y_%H_%M_%S")
        self.state_log_file = os.path.join(os.getcwd(), f"STATES_{sufix}.log")
        with open(self.state_log_file, "w") as f:
            f.write("bang,wspeed,wang\n")

    def sampleInitialState(self, model_name):
        wind_speed = np.random.randint(low=self.min_wind_speed, high=self.max_wind_speed)
        val = 180
        count = 0
        while (val > 150) & (val < 210) & (count < 500):
            theta_boat = 0.0 #np.random.randint(low=-179, high=180)
            theta_wind = np.random.randint(low=-179, high=180)
            val = abs(theta_boat + theta_wind)
            if val > 180:
                val = 360 - val
            count += 1
        if count > 499:
            theta_boat = 0.0
            theta_wind = 0.0

        # -->Set the true wind vector
        self.windSpeed[:2] = self.rot(wind_speed, (theta_wind * self.d2r))
        with open(self.state_log_file, "a") as f:
            f.write(f"{theta_boat}, {wind_speed}, {theta_wind}\n")

        # --> Set the boat's position and orientation
        self.setState(model_name, pose=np.zeros(shape=3, dtype=np.float32), theta=(theta_boat * self.d2r))

    def actionRescale(self, action):
        raction = np.zeros(2, dtype=np.float32)
        # --> Boom angle [0, 90]
        raction[0] = (action[0] + 1) * 45.0
        # --> Rudder angle [-60, 60]
        raction[1] = action[1] * 60.0
        return raction

    def expo(self, dist):
        if dist > 0.6 * self.D0:
            return 3
        elif dist > 0.3 * self.D0:
            return 5
        else:
            return 7

    def trajectoryAngleReturn(self, ref, traj, vel, rwd):
        dA = ref - traj
        if dA < 0:
            return (1.0 + 0.2 * dA * self.d2r) * rwd
        else:
            # --> increase the return earned by a factor proportional to 5% of the standard reward given by the variable rwd.
            return (0.05 * (vel + (dA * self.d2r))) * rwd

    def retrunFunc0(self, obs):
        return (self.DPREV - obs[0]) / self.DMAX

    def returnFunc1(self, obs):
        # -->RETURN FUNCTION VERSION 1 (DID NOT WORK WITH PPO)
        dS   = (self.DPREV - obs[0]) / self.DMAX
        tang = obs[1]
        vel  = obs[2]
        wspd = obs[3]
        r1   = dS
        if vel < 0:
            r2 = -5.0
        else:
            r2 = (vel / wspd) * (np.cos(tang) ** self.expo(obs[1]))

        return (0.07 * r1) + (0.03 * r2)

    def returnFunc2(self, obs):
        # -->RETURN FUNCTION VERSION 2 (NOT TESTED YET)
        dS = (self.DPREV - obs[0]) / self.DMAX
        tang = obs[1]
        vel  = obs[2]
        wspd = obs[3]

        r = dS
        w = 1.0
        c = 0.0
        if vel > 0:
            w += 0.2
            if (tang < 30):
                w += 0.4
                if obs[0] < 0.4 * self.D0:
                    w += 0.1
            elif (tang < 60):
                w += 0.2
            else:
                w = 0.0
                c = -0.3
        else:
            w = 0.0
            c = -0.3

        return w * r + c

    def returnFunc3(self, obs):
        # -->RETURN FUNCTION VERSION 3 (DID NOT WORK WITH PPO)
        dist = obs[0]  # -->distance from the goal
        vel = obs[2]  # -->surge velicuty
        traj = abs(obs[1])  # -->trajectory angle
        dS = self.DPREV - dist

        rwd = 0.01
        R = 0.01

        if dist < 0.3 * self.D0:
            if vel > 0:
                R += self.trajectoryAngleReturn(0, traj, vel, rwd)
            else:
                R += (vel - 15.0) * rwd
        elif dist < 0.6 * self.D0:
            if vel > 0:
                R += self.trajectoryAngleReturn(90, traj, vel, rwd)
            else:
                R += (vel - 10.0) * rwd
        elif dist < 0.9 * self.D0:
            if vel > 0:
                R += self.trajectoryAngleReturn(90, traj, vel, rwd)
            else:
                R += (vel - 10.0) * rwd
        else:
            if vel > 0:
                dA = 60 - traj
                if traj > 60:
                    R += (1.0 + 0.2 * dA * self.d2r) * rwd
                else:
                    R += (0.05 * (vel + (
                                dA * self.d2r))) * rwd  # --> increase the return earned by a factor proportional to 5% of the standard reward given by the variable rwd.
            else:
                if dist < 1.1 * self.D0:
                    R += (vel - 2.0) * rwd
                else:
                    R += (vel - 10.0) * rwd
        if (dS > 0) & (R > 0):
            return R + 2 * dS / self.DMAX
        elif dS < 0:
            return R + 3 * dS / self.DMAX
        else:
            return R

    def rewardFunction(self, obs):
        # --> obsData = [distance, trajectory angle, surge velocity, aparent wind speed, aparent wind angle, boom angle, rudder angle, eletric propultion speed, roll angle]
        #               [   0    ,        1        ,       2        ,         3         ,         4         ,     5     ,      6      ,            7            ,     8     ]

        # -->RETURN FUNCTION VERSION 4 (NOT TESTED)
        dist  = obs[0]       # -->distance from the goal
        vel   = obs[2]       # -->surge velocity
        traj  = abs(obs[1])  # -->trajectory angle
        dS    = self.DPREV - dist
        ratio = dist / self.D0
        R     = dS / self.DMAX

        if R > 0:
            if ratio < 0.3:
                if (traj < 6) & (vel > 0):
                    R *= 1.0 + 0.5 * ((5 - traj) / 5)
                elif (vel < 0):
                    R -= R - 0.2 * vel
                else:
                    pass
            elif ratio < 0.6:
                if (traj < 61) & (vel > 0):
                    R *= (1.0 + 0.05 * ((60 - traj) / 6) + 0.1 * vel)
                elif (vel < 0):
                    R -= R - 0.1 * vel
                else:
                    pass
            elif ratio < 0.9:
                if (traj < 91) & (vel > 0):
                    R *= (1.0 + 0.05 * ((90 - traj) / 9) + 0.1 * vel)
                elif (vel < 0):
                    R -= R - 0.1 * vel
                else:
                    pass
            else:
                if (traj < 61) & (vel > 0):
                    R *= (1.0 + 0.05 * ((60 - traj) / 6) + 0.1 * vel)
                elif (vel < 0):
                    R -= R - 0.1 * vel
                else:
                    pass
        elif R < 0:
            if (ratio < 0.9):
                R *= 5
            else:
                pass
        else:
            pass

        return R

    def step(self, action):
        # --> UNPAUSE SIMULATION
        rospy.wait_for_service("/gazebo/unpause_physics")
        try:
            self.unpause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/unpause_physics service call failed!"))

        # -->SEND ACTION TO THE BOAT CONTROL INTERFACE
        ract = self.actionRescale(action)
        self.boomAng_pub.publish(ract[0])
        self.rudderAng_pub.publish(ract[1])

        # -->GET OBSERVATIONS (NEXT STATE)
        observations = self.getObservations()

        # -->PAUSE SIMULATION
        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            self.pause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/pause_physics service call failed!"))

        # -->CALCULATES THE REWARD
        reward = self.rewardFunction(observations)

        # -->UPDATE PREVIOUS STATE VARIABLES
        self.DPREV = observations[0]
        windAng    = abs(observations[4])

        # -->CHECK FOR A TERMINAL STATE
        done = bool((self.DPREV <= 5) |
                    (self.DPREV > self.DMAX) |
                    ((windAng >= 160) & (windAng <= 200) & (observations[2] < 0.5)) |   #-->A done signal is returned if the wind is blowing from the bow and the surge velocity is smaller than 0.5 m/s
                    (np.isnan(observations).any())
                    )

        if np.isnan(observations).any():
            print("\n\n-------------------------------------")
            print(f"distance: {observations[0]}")
            print(f"traj ang: {observations[1]}")
            print(f"boat vel: {observations[2]}")
            print(f"wind vel: {observations[3]}")
            print(f"wind ang: {observations[4]}")
            print(f"boom ang: {observations[5]}")
            print(f"rud ang : {observations[6]}")
            print(f"prop    : {observations[7]}")
            print(f"roll ang: {observations[8]}")
            print("-------------------------------------\n")
            # --> WAIT FOR ACKNOWLEDGEMENT FROM USER
            # _ = input("Unpause: ")

        # -->PROCESS DONE SIGNAL
        if done:
            if (self.DPREV <= 5):
                reward = 1
            elif((self.DPREV > self.DMAX) |
                 ((observations[4] >= 160) & (observations[4] <= 200) & (observations[2] < 0.5))):
                reward = -1

        self.step_count += 1

        return self.observationRescale(observations[:5]), reward, done, False, {}

    def reset(self, seed = None):
        # -->RESETS THE STATE OF THE ENVIRONMENT.
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print(("/gazebo/reset_simulation service call failed!"))

        # -->UNPAUSE SIMULATION TO MAKE OBSERVATION
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/unpause_physics service call failed!"))

        # -->SET THE ACTUATORS BACK TO THE DEFAULT SETTING
        self.propVel_pub.publish(0)
        self.boomAng_pub.publish(0.0)
        self.rudderAng_pub.publish(0.0)

        # -->SET RANDOM INITIAL STATE
        self.sampleInitialState("eboat")
        self.wind_pub.publish(Point(self.windSpeed[0], self.windSpeed[1], self.windSpeed[2]))

        # -->COLLECT OBSERVATIONS
        observations = self.getObservations()

        # -->RESET INITIAL STATE VALUES
        self.DPREV = observations[0]

        # -->PAUSE SIMULATION
        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            self.pause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/pause_physics service call failed!"))

        self.count += 1

        return self.observationRescale(observations[:5]), {}

class EboatStraightLineEnvCC29v0(EboatSingleWayPointEnvCC35v0):
    def __init__(self):
        self.EBOAT_HOME = "/home/eduardo/USVSim/yara_ws/src/Yara_OVE"
        GazeboEnv.__init__(self, os.path.join(self.EBOAT_HOME, "eboat_gazebo/launch/ocean_RL_training.launch"))
        # GazeboEnv.__init__(self, os.path.join(self.EBOAT_HOME, "eboat_gazebo/launch/ocean.launch"))

        self.boomAng_pub   = rospy.Publisher("/eboat/control_interface/sail", Float32, queue_size=5)
        self.rudderAng_pub = rospy.Publisher("/eboat/control_interface/rudder", Float32, queue_size=5)
        self.propVel_pub   = rospy.Publisher("/eboat/control_interface/propulsion", Int16, queue_size=5)
        self.wind_pub      = rospy.Publisher("/eboat/atmosferic_control/wind", Point, queue_size=5)
        self.unpause       = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause         = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy   = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.set_state     = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.get_state     = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        # --> GLOBAL VARIABLES
        self.DTOL   = 25.0  # --> Threshold for distance. If the boat goes far than INITIAL POSITION + DMAX, a done signal is trigged.
        self.STATE0 = None  # --> The boat state at time t0
        self.lateral_limit = 5

        # --> We will use a rescaled action space
        self.action_space = spaces.Box(low=-1,
                                       high=1,
                                       shape=(2,),
                                       dtype=np.float32)

        # --> We will use a rescaled action space
        self.observation_space = spaces.Box(low=-1,
                                            high=1,
                                            shape=(9,),
                                            dtype=np.float32)
        self.reward_range = (-1, 1)

        self._seed()

        # --> SET WIND SPEED INITIAL VECTOR
        self.windSpeed = np.array([0.0, 6.12, 0.0], dtype=np.float32)

        # --> TRUE WIND RESTRICTED ANGLES
        self.windArray = np.array([-135, -90, -45, 0, 45, 90, 135])
        # self.windArray = np.array([0])

        # --> UNPAUSE SIMULATION
        rospy.wait_for_service("/gazebo/unpause_physics")
        try:
            self.unpause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/unpause_physics service call failed!"))

        # --> GET INITIAL DISTANCE FROM THE WAYPOINT AND THE TRAJECTORY ANGLE BETWEEN: (a) THE STRAIGHT LINE CONNECTING THE BOAT TO THE WAY POINT; (b) THE FORWARD VECTOR OF THE BOAT.
        while (self.STATE0 is None):
            try:
                self.STATE0 = rospy.wait_for_message("/eboat/mission_control/observations", Float32MultiArray, timeout=20)
            except:
                pass

        # --> AUXILIARY VARS FOR OBSERVATIONS
        self.D0    = self.STATE0.data[0]
        self.DPREV = self.STATE0.data[0]
        self.DMAX  = self.STATE0.data[0] + self.DTOL  # --> IT TAKE THE INITIAL DISTANCE IN CONSIDERATION
        self.TPREV = self.STATE0.data[1]
        self.RPREV = 0

        # --> AUXILIARY VARS
        self.d2r = np.pi / 180.0
        self.step_count = 0

        # --> SUPPORT FOR RANDOM INITIALIZATION OF WIND SPEED AND DIRECTION
        np.random.seed(30)
        self.wind_speed      = 0.0 #--> intensity of the true wind
        self.min_wind_speed  = 3
        self.max_wind_speed  = 11

        # --> ESTIMATED NUMBER OF STEPS BASED ON WIND VELOCITY (THIS IS VALID ONLY FOR A TIME STEP EQUAL TO 3 SECONDS)
        # wind_speed                     = [3 , 4 , 5 , 6 , 7 , 8 , 9 , 10
        self.max_expected_episode_length = [40, 27, 21, 18, 16, 14, 13, 12]

    def getObservations(self):
        obsData = None
        while obsData is None:
            try:
                obsData = rospy.wait_for_message('/eboat/mission_control/observations', Float32MultiArray,
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

    def observationRescale(self, observations):
        lobs = len(observations) - 2
        robs = np.zeros(lobs, dtype=np.float32)
        # --> Distance from the waypoint (m) [0, DMAX];
        robs[0] = 2 * (observations[0] / self.DMAX) - 1
        # --> Trajectory angle               [-180, 180]
        robs[1] = observations[1] / 180.0
        # --> Boat linear velocity (m/s)     [-10, 10 ]
        robs[2] = observations[2] / 10
        # --> Aparent wind speed (m/s)       [0, 15]
        robs[3] = 2 * (observations[3] / 15) - 1
        # --> Apparent wind angle            [-180, 180]
        robs[4] = observations[4] / 180.0
        # --> Boom angle                     [0   , 90]
        robs[5] = (observations[5] / 45.0) - 1
        # --> Rudder angle                   [-60 , 60 ]
        robs[6] = observations[6] / 60.0
        # --> Electric propulsion speed      [-5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5]
        # robs[7] = observations[7] / 5.0
        robs[7] = 0.0
        # --> Roll angle                     [-180, 180]
        robs[8] = observations[8] / 180.0

        return robs

    def sampleInitialState(self, model_name):
        theta_boat = 0.0
        self.wind_speed = np.random.randint(low=self.min_wind_speed, high=self.max_wind_speed)
        theta_wind      = np.random.choice(self.windArray)
        # val        = 180
        # count      = 0
        # while (val > 150) & (val < 210) & (count < 500):
        #     theta_wind = np.random.randint(low=-179, high=180)
        #     val = abs(theta_boat + theta_wind)
        #     if val > 180:
        #         val = 360 - val
        #     count += 1
        # if count > 499:
        #     theta_boat = 0.0
        #     theta_wind = 0.0

        # -->Set the true wind vector
        self.windSpeed[:2] = self.rot(self.wind_speed, (theta_wind * self.d2r))

    def rewardFunction(self, obs, act):
        # ** Based on experimental results the absolute value of surge velocity is within the interval [0, 3.5] m/s.
        #    So, with a step interval of three seconds, the boat will travel 10.5 meters. As we will use a normalized
        #    traveled distance as the core of our retrun, the maximum distance traveled will be
        #    dS / 10.5 = 10.5 / 10.5 = 1.0 .
        #    we do not desire that the maximum return per step be bigger than 0.1 .
        dS = (self.DPREV - obs[0])/self.D0
        C  = 1.0

        # --> A NEGATIVE RETURN FOR TAKING TO MUCH TIME TO REACH THE GOAL
        if dS > 0:
            # C += (0.2 - 0.008333333 * abs(self.RPREV - act[1])) - 0.5 * (self.step_count / 30)**4
            # if abs(obs[10]) < 2:
            #     C += 0.1
            mexel = self.max_expected_episode_length[self.wind_speed-3] #--> maximum expected episode length base on the true wind velocity
            C -= ((0.15 * abs(self.RPREV - act[1]))                     #--> discount that scales with the variation inthe rudder's angle (max discount = 30% of the reward)
                  + (0.5 * (self.step_count / mexel)**4)                #--> discount that scales with the elapsed time (in numbers of steps)
                 )
            # C += 0.6 * np.cos(obs[1]*self.d2r)**3 #--> bonus/discount as function of the trajectory angle (bonus if the boat is pointing towards de goal and discount otherwise)


        # --> obsData = [distance, trajectory angle, linear velocity, aparent wind speed, aparent wind angle, boom angle, rudder angle, eletric propultion speed, roll angle]
        #               [   0    ,        1        ,       2        ,         3         ,         4         ,     5     ,      6      ,            7            ,     8     ]

        return C * dS

    def step(self, action):
        # --> UNPAUSE SIMULATION
        rospy.wait_for_service("/gazebo/unpause_physics")
        try:
            self.unpause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/unpause_physics service call failed!"))

        # -->SEND ACTION TO THE BOAT CONTROL INTERFACE
        self.boomAng_pub.publish((action[0] + 1) * 45.0) #-->THE ACTION RETURNED BY THE AGENT IS BETWEEN -1 AND 1, BUT THE EBOAT REQUIRES VALUES BETWEEN 0 AND 90
        self.rudderAng_pub.publish(action[1] * 60.0)     #-->THE ACTION RETURNED BY THE AGENT IS BETWEEN -1 AND 1, BUT THE EBOAT REQUIRES VALUES BETWEEN -60 AND 60

        # -->GET OBSERVATIONS (NEXT STATE)
        observations = self.getObservations()

        # -->GET MODEL POSITION
        posY = observations[10]

        # -->PAUSE SIMULATION
        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            self.pause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/pause_physics service call failed!"))

        # -->CALCULATES THE REWARD
        reward = self.rewardFunction(observations, action)

        # -->CHECK FOR A TERMINAL STATE
        windAng = abs(observations[4])
        done = bool((observations[0] <= 5) |
                    (observations[0] > self.DPREV) |
                    (abs(posY) > self.lateral_limit) |
                    ((windAng >= 160) & (windAng <= 200) & (observations[2] < 0.5)) |  # -->A done signal is returned if the wind is blowing from the bow and the surge velocity is smaller than 0.5 m/s
                    (self.step_count > 59) |
                    (np.isnan(observations).any())
                    )

        if np.isnan(observations).any():
            print("\n\n-------------------------------------")
            print(f"distance: {observations[0]}")
            print(f"traj ang: {observations[1]}")
            print(f"boat vel: {observations[2]}")
            print(f"wind vel: {observations[3]}")
            print(f"wind ang: {observations[4]}")
            print(f"boom ang: {observations[5]}")
            print(f"rud ang : {observations[6]}")
            print(f"prop    : {observations[7]}")
            print(f"roll ang: {observations[8]}")
            print("-------------------------------------\n")
            # --> WAIT FOR ACKNOWLEDGEMENT FROM USER
            # _ = input("Unpause: ")

        # -->PROCESS DONE SIGNAL
        if done:
            if (observations[0] <= 5):
                reward = 1
            elif (not(np.isnan(observations).any())):
                reward = -1
            else:
                pass
        else:
            # -->UPDATE PREVIOUS STATE VARIABLES
            self.DPREV = observations[0]
            self.RPREV = action[1]

        self.step_count += 1

        return self.observationRescale(observations), reward, done, False, {}

    def reset(self, seed = None):
        # -->RESETS THE STATE OF THE ENVIRONMENT.
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print(("/gazebo/reset_simulation service call failed!"))

        # -->UNPAUSE SIMULATION TO MAKE OBSERVATION
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/unpause_physics service call failed!"))

        # -->SET THE ACTUATORS BACK TO THE DEFAULT SETTING
        self.propVel_pub.publish(0)
        self.boomAng_pub.publish(0.0)
        self.rudderAng_pub.publish(0.0)

        # -->SET RANDOM INITIAL STATE
        self.sampleInitialState("eboat")
        self.wind_pub.publish(Point(self.windSpeed[0], self.windSpeed[1], self.windSpeed[2]))

        # -->COLLECT OBSERVATIONS
        observations = self.getObservations()

        # -->PAUSE SIMULATION
        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            self.pause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/pause_physics service call failed!"))

        # -->RESET INITIAL STATE VALUES
        self.DPREV      = observations[0]
        self.TPREV      = observations[1]
        self.RPREV      = 0.0
        self.step_count = 0

        return self.observationRescale(observations), {}