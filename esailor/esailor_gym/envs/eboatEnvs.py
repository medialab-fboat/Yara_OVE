
from .laser_scan_processor import LaserScanProcessor

from sensor_msgs.msg import LaserScan
import threading

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

import random
from rosgraph_msgs.msg import Clock

from std_srvs.srv import Empty

from std_msgs.msg import Float32, Int16, Float32MultiArray
from geometry_msgs.msg import Point, Pose

from tf.transformations import quaternion_from_euler
from gazebo_msgs.srv import SetModelState, SpawnModel, DeleteModel, GetModelState
from gazebo_msgs.msg import ModelState

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib
matplotlib.use('Agg')
from matplotlib.animation import FuncAnimation




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
        
        self.laser_processor = LaserScanProcessor()

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
        #self.laser_scan = rospy.Subscriber(f"/eboat/laser/scan", LaserScan, self._laser_scan_callback)         

    """ def _laser_scan_callback(self, laser_data):
        num_groups = 6
        group_size = len(laser_data.ranges) // num_groups
        groups = [laser_data.ranges[i:i+group_size] for i in range(0, len(laser_data.ranges), group_size)]

        group_means = np.zeros(num_groups)

        for i, group in enumerate(groups):
            valid_ranges = [range_val for range_val in group if range_val < float('inf')]
            group_means[i] = np.mean(valid_ranges) if valid_ranges else 0

        self.processed_lidar_data = group_means
        neural_inputs = 1 - np.clip(group_means / 5, 0, 1) # 5 is the maximum range of the lidar, 
                                                           # so we normalize the values between 0 and 1
        self.processed_lidar_data = neural_inputs if neural_inputs.any() else np.zeros(num_groups)

    # Chamar esta função fora do callback para evitar problemas com threads.
    def plot_groups(groups, num_groups):
        import matplotlib.pyplot as plt
        plt.style.use('fivethirtyeight')
        fig, axs = plt.subplots(num_groups, 1, figsize=(10, 8))

        for ax, group in zip(axs, groups):
            ax.plot(group)
            ax.set_title(f'Group {groups.index(group) + 1}')
            
        plt.tight_layout()
        try:
            plt.show(block=False)
        except Exception as e:
            print(f"Plotting error: {e}. Saving plot to file instead.")
            fig.savefig('/home/araujo/yara_ws/src/Yara_OVE/esailor/esailor_gym/envs/laser_scan_visualizer.png')
        plt.tight_layout()
        plt.show()

    # Função para processar os dados em grupos
    def process_scan_in_groups(ranges, group_size=20):
        ranges_array = np.array(ranges)
        if ranges_array.size % group_size == 0:
            groups = np.split(ranges_array, ranges_array.size // group_size)
            group_means = [np.mean(group) for group in groups if np.any(group < float('inf'))]
            return group_means
        else:
            rospy.logerr("The number of rays in the scan does not evenly divide into groups of size %d", group_size)
            return []
 """    
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
            #             11 Grouped laser scan data color 1
            #             12 Grouped laser scan data color 2
            #             13 Grouped laser scan data color 3
            #             14 Grouped laser scan data color 4
            #             15 Grouped laser scan data color 5
            
        #LaserScan groups
        self.laser_scan_data = obsData[11:16]

        print("Length of obsData:", len(obsData))
        return np.array(obsData, dtype=float)

    def rescaleObs(self, observations):
        #lobs = len(observations)
        
        robs = np.zeros(len(observations), dtype=np.float32)
        
        for i in range(len(observations)):
            robs[i] = observations[i]
        
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
        # --> Boat's current X position      [-100, 100]
        robs[9] = observations[9] / 100.0
        # --> Boat's current Y position      [-100, 100]
        robs[10] = observations[10] / 100.0
               

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
        
        #Raycast
        state.raycast = 0

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
        
        self.laser_scan_data = []

        #-->DEFINE OBSERVATION AND ACTION SPACES
        self.action_space = spaces.Box(low=-1,
                                       high=1,
                                       shape=(4,),
                                       dtype=np.float32)

        #self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(9,), dtype=np.float32)
        #self.observation_space = spaces.Box(low=np.array([0]*11), high=np.array([np.inf]*11), shape=(11,), dtype=np.float32)
        self.observation_space = spaces.Box(low=np.array([0]*(11+len(self.laser_scan_data))), high=np.array([np.inf]*(11+len(self.laser_scan_data))), shape=(11+len(self.laser_scan_data),), dtype=np.float32)


        #-->SET WIND INITIAL CONDITIONS AND DEFINE ITS HOW IT WILL VARIATE
        self.windVec = np.array([0, 0, 0], dtype=np.float32)
        self.min_windspeed   = 3
        self.max_windspeed   = 11
        self.wind_directions = np.array([-135, -90, -45, -5, 5, 45, 90, 135])
        self.wind_speed      = 0.0
        #Raycast
        #self.laser_scan = None
        #rospy.Subscriber(f"/{self.model_namespace}/laser/scan", LaserScan, self._laser_scan_callback)
        #self.processed_lidar_data = np.zeros(120)
        #self.raycast = rays()
        #self.raycast_data = None 
        
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
        self.lateral_limit = 35                             #-->DEFINE THE HOW MUCH THE BOAT CAN TRAVEL AWY FROM THE STRAIGHT LINE BEFORE A DONE SIGNAL
        self.PREVACT       = np.array([-1, -1])
        self.maxcharge     = 10
        self.battery       = np.full(2, fill_value=self.maxcharge, dtype=int)
        #Raycast
        #self.raycast_data = 0  
            

    def rewardFunction(self, obs, action): #-->rewardFunctionD (ATIVA)
        # QUais sao as funcoes de truncamento e finalizacao dos episodios?
        dS    = self.PREVOBS[0] - obs[0]
        ta    = abs(obs[1])
        pta   = abs(self.PREVOBS[1])
        db    = abs(self.PREVOBS[5] - (action[0] + 1.0) * 45.0) #--> DIFFERENCE BETWEEN THE POSITION OF THE BOOM AND THE NEW POSITION OF THE BOOM REQUIRED BY THE AGENT
        dr    = abs(self.PREVOBS[6] - (action[1] * 60.0))       #--> DIFFERENCE BETWEEN THE POSITION OF THE RUDDER AND THE NEW POSITION OF THE RUDDER REQUIRED BY THE AGENT
        dp    = abs(self.PREVOBS[7] - (action[2] * 5.0))        #--> DIFFERENCE BETWEEN THE POSITION OF THE PROPELLER AND THE NEW POSITION OF THE PROPELLER REQUIRED BY THE AGENT
       
        alpha = 180 - obs[5] - abs(obs[4])                      #--> Sail attack angle       
       
        # Add logic to rescale the laser scan data within observations
        

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
            propeller_energy_factor = ((dp > 0) * np.min([1.0, dp / 5.0]) * 0.1) + ((dp > 5) * np.min([1, dp / 90.0]) * 0.3)
            if C4 > 0:
                C4 *= (-1.0)*(boom_position_factor + boom_energy_factor + rudder_energy_factor + propeller_energy_factor)
            else:
                C4 *= boom_position_factor + boom_energy_factor + rudder_energy_factor + propeller_energy_factor
            #########################################
            R = C0 + C1 + C2 + C3 + C4 
        elif dS >= 0:
            R = (-0.15) * (self.step_count / 60.0)
            self.step_count += 1
        else:
            R = np.min([-5.0, ((-0.15) * (self.step_count / 60.0))]) * C0

        return np.max([R, -1])

    def step(self, action):
        # Unpause the simulation
        rospy.wait_for_service("/gazebo/unpause_physics")
        try:
            self.unpause()
        except rospy.ServiceException as e:
            print("/gazebo/unpause_physics service call failed!")

        # Publish the actions to the ROS topics
        self.boomAng_pub.publish((action[0] + 1) * 45.0)
        self.rudderAng_pub.publish(action[1] * 60.0)
        self.propVel_pub.publish(int(action[2] * 5))

        # Get observations from the environment
        obs = self.getObservations()

        # Process laser scan data if available
        if len(obs) >= 15:  # Assuming laser data are the last 5 elements in obs
            laser_data = self.laser_processor.process_data()
            max_range = 5
            for i in range(-5, 0):  # Loop over the last 5 elements of obs
                if obs[i] > max_range:
                    obs[i] = max_range
                elif obs[i] < 0:
                    obs[i] = 0
        else:
            print("Laser data not available in observations")

        # Pause the simulation
        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            self.pause()
        except rospy.ServiceException as e:
            print("/gazebo/pause_physics service call failed!")

        # Rescale observations
        robs = self.rescaleObs(obs)

        # Calculate the reward
        reward = self.rewardFunction(obs, action)

        # Check for a terminal state
        posY = obs[10]
        windAng = abs(obs[4])
        done = bool((obs[0] <= 5) or
                    (obs[0] > self.PREVOBS[0]) or
                    (abs(posY) > self.lateral_limit) or
                    ((windAng >= 160) and (windAng <= 200) and (obs[2] < 0.5)) or
                    (np.isnan(obs).any()))

        # Process done signal
        if done:
            if (obs[0] <= 5):
                reward = 1
            elif (not (np.isnan(obs).any())):
                reward = -1

        # Update previous state variables
        self.PREVOBS = obs
        self.PREVACT = action[0], action[1], action[2]

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
        #LaserScan groups
                

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
        #Raycast
        #self.raycast_data = obs[9]

        return robs, {}
