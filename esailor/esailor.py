#!/home/eduardo/miniconda3/envs/esailor/bin/python

#-->PYTHON UTIL
import time
import random
import numpy as np
import glob
import sys
from datetime import datetime
import sys, os, signal, subprocess

import pandas as pd
from matplotlib import pyplot as plt

from simple_pid import PID

#-->ROS
import rospy

#-->GYM
import gymnasium as gym

#-->GAZEBO
import esailor_gym
from std_srvs.srv import Empty
from std_msgs.msg import Float32, Int16, Float32MultiArray
from gazebo_msgs.srv import SetModelState, GetModelState, SpawnModel, DeleteModel, SetPhysicsProperties, GetPhysicsProperties
from geometry_msgs.msg import Point, Pose, Vector3
from gazebo_msgs.msg import ODEPhysics, ModelState
from sensor_msgs.msg import LaserScan
from tf.transformations import quaternion_from_euler

#-->STABLE-BASELINES3
# from gymnasium import wrappers
from stable_baselines3 import PPO, SAC, A2C
from stable_baselines3.common.callbacks import CheckpointCallback

#-->PYTORCH
import torch as th

class esailor():
    def __init__(self):
        random_number           = random.randint(10000, 15000)
        self.port_ros           = str(random_number)                 #--> standard ROS MASTER PORT "11311"
        self.port_gazebo        = str(random_number + 1)             #--> standard GAZEBO PORT "11345"

        self.HOME               = os.path.expanduser('~')

        self._roslaunch              = None
        self._physics_properties     = None
        self._pause                  = None
        self._unpause                = None
        self._holdPhysiscsProperties = False
        self.DMAX                    = 1.0
        self.laser_scan              = np.zeros(5, dtype=int)

    def _autoSarch4EmptyOceanLaunchFile(self):
        # -->SEARCH FOR A LAUNCH FILE
        print("As the user did not provide a viable launch file, I will search for one.\nThis may take a while, pelase wait!")
        files           = glob.glob(os.path.join(self.HOME, "**/*empty_ocean.launch"), recursive=True)
        path2launchfile = None
        if len(files) > 0:
            path2launchfile = files[0]
            del (files)
        else:
            path2launchfile = input("\nI did not find a viable launch file!\nPlease provide an valid path:\n")
            if (((len(path2launchfile) > 0) & (not (os.path.exists(path2launchfile)))) | (
                    len(path2launchfile) == 0)):
                raise IOError("File " + path2launchfile + " does not exist")

        with open(self.logfile, "a") as f:
            f.writelines(f"\nLaunch file used to start the simulation: {path2launchfile}\n")

        return path2launchfile

    def launchGazeboSimulation(self, path2launchfile = None):
        #-->EXPORT ADDRESS FOR THE ROS MASTER NODE AND THE GAZEBO SERVER
        os.environ["ROS_MASTER_URI"]    = "http://localhost:" + self.port_ros
        os.environ["GAZEBO_MASTER_URI"] = "http://localhost:" + self.port_gazebo

        #-->LOG FILE
        self.logfile = "esailor_" + self.port_ros + ".log"
        with open(self.logfile, "w") as f:
            f.writelines("ROS_MASTER_URI = http://localhost:" + self.port_ros + "\n")
            f.writelines("GAZEBO_MASTER_URI = http://localhost:" + self.port_gazebo + "\n")

        #-->SEARCH FOR A LAUNCH FILE
        if path2launchfile == None:
            path2launchfile = self._autoSarch4EmptyOceanLaunchFile()

        #-->LAUNCH THE SIMULATION USING SUBPROCESS
        ros_path = os.path.dirname(subprocess.check_output(["which", "roscore"]))
        self._roslaunch = subprocess.Popen([sys.executable, os.path.join(ros_path, b"roslaunch"), "-p", self.port_ros, path2launchfile])

        #-->INITIALIZE A ROS NODE
        try:
            rospy.init_node(f"esailor", anonymous=True)
        except:
            print("ROSMASTER is not running!")
            print(time.time())
            self.close()
            exit(1)

        #--> ROS SERVICES
        self._pause              = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self._unpause            = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self._physics_properties = rospy.ServiceProxy("/gazebo/get_physics_properties", GetPhysicsProperties)

    def getPhysicsProperties(self):
        if self._roslaunch == None:
            print("There is no simulation runnig!")
            return None
        else:
            rospy.wait_for_service('/gazebo/get_physics_properties')
            print("\n\n-------------------------------------")
            print(self._physics_properties())
            print("-------------------------------------\n")
            return self._physics_properties()

    def setPhysicsProperties(self, time_step = None, max_update_rate = None, gravity = None, ode_config = None):
        if time_step == None      : time_step       = self._physics_properties().time_step
        if max_update_rate == None: max_update_rate = self._physics_properties().max_update_rate
        if gravity == None        : gravity         = self._physics_properties().gravity
        if ode_config == None     : ode_config      = self._physics_properties().ode_config

    def holdPhysicsProperties(self, val = True):
        self._holdPhysiscsProperties = val

    def pauseSim(self, val = True):
        if (val & (self._pause != None)):
            rospy.wait_for_service("/gazebo/pause_physics")
            try:
                self._pause()
            except(rospy.ServiceException) as e:
                print(("/gazebo/pause_physics service call failed!"))
        elif (not(val) & (self._unpause != None)):
            rospy.wait_for_service("/gazebo/unpause_physics")
            try:
                self._unpause()
            except(rospy.ServiceException) as e:
                print(("/gazebo/unpause_physics service call failed!"))

    def _checkPhysicsProperties(self, timestep = 0.01, max_update_rate = 100.0):
        rospy.wait_for_service('/gazebo/get_physics_properties')
        # print("\n\n-------------------------------------\n",self._physics_properties(),"\n-------------------------------------\n")
        if not(self._physics_properties().pause):
            self.pauseSim()

        if ((self._physics_properties().time_step != 0.01) | (self._physics_properties().max_update_rate != 0)):
            rospy.wait_for_service('/gazebo/set_physics_properties')
            set_physics_properties = rospy.ServiceProxy("/gazebo/set_physics_properties", SetPhysicsProperties)
            try:
                result = set_physics_properties(time_step       = np.float64(timestep),
                                                max_update_rate = np.float64(max_update_rate),
                                                gravity         = self._physics_properties().gravity,
                                                ode_config      = self._physics_properties().ode_config
                                               )
            except rospy.ServiceException:
                result = "/gazebo/SetPhysicsProperties service call failed"

        rospy.wait_for_service('/gazebo/get_physics_properties')
        # print("\n\n-------------------------------------\n", self._physics_properties(), "\n-------------------------------------\n")

    def checkEnv(self,
                 envid = "Eboat93-v1"):
        # --> LAUNCH GAZEBO SIMULATION IF IT IS NOT RUNNING YET
        if self._roslaunch == None:
            print("Gazebo simulation was not started by the user.\nStarting simulation with empy ocean world.")
            self.launchGazeboSimulation(
                "/home/eduardo/USVSim/yara_ws/src/Yara_OVE/eboat_gazebo/launch/empty_ocean.launch")
            # self.launchGazeboSimulation()

        # --> ADJUST PHYSICS PROPERTIES
        if not (self._holdPhysiscsProperties):
            self._checkPhysicsProperties()

        # --> START GYMNASIUM ENVIRONMENT
        env = gym.make(envid)
        # env.repositionWayPoint(newdistance = 200)

        for _ in range(3):
            _, _ = env.reset()
            for i in range(10):
                _, _, _, _, _ = env.step([1, 0, 0.2])

    def training(self,
                 rlagent    = "PPO",
                 policy     = "MlpPolicy",
                 envid      = "Eboat93-v0",
                 numofsteps = 500000,
                 refmodel   = None,
                 actor      = [11, 11],
                 critic     = [11, 11],
                 sufix      = "esailor",
                 logdir     = "logs"):

        #--> LAUNCH GAZEBO SIMULATION IF IT IS NOT RUNNING YET
        if self._roslaunch == None:
            print("Gazebo simulation was not started by the user.\nStarting simulation with empy ocean world.")
            self.launchGazeboSimulation("/home/eduardo/USVSim/yara_ws/src/Yara_OVE/eboat_gazebo/launch/empty_ocean.launch")
            # self.launchGazeboSimulation()

        #--> ADJUST PHYSICS PROPERTIES
        if not(self._holdPhysiscsProperties):
            self._checkPhysicsProperties(max_update_rate = 0)

        #--> SET BASE FILE NAME STRUCTURE TO SAVE TENSORBOARD LOGS AND TRAINED MODELS
        sufix += "_A"
        for val in actor:
            sufix += f"{val}"
        sufix += "_C"
        for val in critic:
            sufix += f"{val}"
        sufix += "_" + datetime.now().strftime("%d%m%Y_%H_%M_%S")

        #--> START GYMNASIUM ENVIRONMENT
        env = gym.make(envid)

        if not os.path.exists(logdir):
            os.makedirs(logdir)

        if rlagent == "SAC":
            models_dir = f"models/SAC/{sufix}"
            tb_log_name = f"SAC_{sufix}"

            if not os.path.exists(models_dir):
                os.makedirs(models_dir)

            policy_kwargs = dict(activation_fn=th.nn.ReLU,
                                 net_arch=(dict(pi=actor, qf=critic))
                                 )

            model = SAC(policy                =policy,
                        env                   =env,
                        # learning_rate         = 0.0003,
                        # buffer_size           = 1000000,
                        # learning_starts       = 100,
                        batch_size            = 64, #256,
                        # tau                   = 0.005,
                        # gamma                 = 0.99,
                        # train_freq            = 1,
                        # gradient_steps        = 1,
                        # action_noise          = None,
                        # replay_buffer_class   = None,
                        # replay_buffer_kwargs  = None,
                        # optimize_memory_usage = False,
                        ent_coef              = 0.001, #'auto',
                        # target_update_interval= 1,
                        # target_entropy        = 'auto',
                        # use_sde               = False,
                        # sde_sample_freq       = -1,
                        # use_sde_at_warmup     = False,
                        # stats_window_size     = 100,
                        tensorboard_log       = logdir,
                        policy_kwargs         = policy_kwargs, #None,
                        # verbose               = 0,
                        # seed                  = None,
                        # device                = 'auto',
                        # _init_setup_model     = True
                        )
        else:
            models_dir  = f"models/PPO/{sufix}"
            tb_log_name = f"PPO_{sufix}"

            if not os.path.exists(models_dir):
                os.makedirs(models_dir)

            policy_kwargs = dict(activation_fn=th.nn.ReLU,
                                 net_arch=(dict(pi=actor, vf=critic))
                                 )

            model = PPO(policy            = policy,
                        env               = env,
                        # learning_rate     = 0.0003,
                        # n_steps           = 2048,
                        # batch_size        = 64,
                        # n_epochs          = 10,
                        # gamma             = 0.99,
                        # gae_lambda        = 0.95,
                        # clip_range        = 0.2,
                        # clip_range_vf     = None,
                        # normalize_advantage = True,
                        ent_coef          = 0.001, #0.0
                        # vf_coef           = 0.5,
                        # max_grad_norm     = 0.5,
                        # use_sde           = False,
                        # sde_sample_freq   = -1,
                        # target_kl         = None,
                        tensorboard_log   = logdir,
                        policy_kwargs     = policy_kwargs, #None,
                        # verbose           = 0,
                        # seed              = None,
                        # device            = 'auto',
                        # _init_setup_model = True
                        )

        if refmodel != None:
            # -->SET THE NEURAL NETWORK PARAMETERS EQUAL TO A LOADED PREVIOUS TRAINED NEURAL NETWORK
            model.set_parameters(refmodel.get_parameters())

        checkpoint_callback = CheckpointCallback(save_freq          = 10000,
                                                 save_path          = models_dir,
                                                 name_prefix        = "esailor_model",
                                                 save_replay_buffer = True,
                                                 save_vecnormalize  = True,
                                                )

        model.learn(total_timesteps     = numofsteps,
                    log_interval        = 1,
                    tb_log_name         = tb_log_name,
                    callback            = checkpoint_callback,
                    reset_num_timesteps = False,
                    progress_bar        = True
                    )
        if not(os.path.exists(f"{models_dir}/esailor_model_{numofsteps}_steps.zip")):
            model.save(f"{models_dir}/esailor_model_{numofsteps}_steps")

    def spawnURDFModel(self, model_namespace, descriptor_file, ipose = None):
        # -->SPAWN THE MODEL IN THE GAZEBO SIMULATION
        spawn_urdf = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
        if ipose == None:
            ipose = Pose()
            ipose.position.x = 0.0
            ipose.position.y = 0.0
            ipose.position.z = 0.0
        count = 0
        spawnflag = "Fail"
        while (spawnflag == "Fail") & (count < 18):
            with open(f"{descriptor_file}.urdf", "r") as f:
                urdffile = f.read()
                try:
                    result = spawn_urdf(model_name      = model_namespace,
                                        model_xml       = urdffile,
                                        robot_namespace = model_namespace,
                                        initial_pose    = ipose,
                                        reference_frame = "world")
                    spawnflag = "Sucess"
                except rospy.ServiceException:
                    result = "/gazebo/SpawnModel service call failed"
                    count += 1
                    time.sleep(5)
        print(f"\n\n===========================\n{result}\n===========================\n")

    def spawnSDFModel(self, model_namespace, descriptor_file_path, ipose = None):
        # -->SPAWN THE WAYPOINT MARKER IN THE GAZEBO SIMULATION
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        spawn_sdf = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        if ipose == None:
            ipose = Pose()
            ipose.position.x = 100.0
            ipose.position.y = 0.0
            ipose.position.z = 0.0

        with open(descriptor_file_path, "r") as f:
            sdffile = f.read()
            try:
                result = spawn_sdf(model_name      = model_namespace,
                                   model_xml       = sdffile,
                                   robot_namespace = model_namespace,
                                   initial_pose    = ipose,
                                   reference_frame = "world")
            except rospy.ServiceException:
                result = "/gazebo/SpawnModel service call failed"
            print(f"\n\n===========================\n{result}\n===========================\n")

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

    def setState(self, model_name, ipose):
        state = ModelState()
        state.model_name = model_name
        state.reference_frame = "world"
        # pose
        state.pose = ipose
        # twist
        state.twist.linear.x = 0
        state.twist.linear.y = 0
        state.twist.linear.z = 0
        state.twist.angular.x = 0
        state.twist.angular.y = 0
        state.twist.angular.z = 0

        rospy.wait_for_service('/gazebo/set_model_state')
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        try:
            result = set_state(state)
            assert result.success is True
        except rospy.ServiceException:
            print("/gazebo/set_model_state service call failed")

    def testModel(self, model, baseDistance = 100.0):
        # --> LAUNCH GAZEBO SIMULATION IF IT IS NOT RUNNING YET
        if self._roslaunch == None:
            print("Gazebo simulation was not started by the user.\nStarting simulation with empy ocean world.")
            self.launchGazeboSimulation()

        #--> WAIT FOR PHYSICS
        rospy.wait_for_service('/gazebo/get_physics_properties')

        # -->SERACH FOR THE URDF FILE DESCRIBING THE BOAT
        modelname = f"eboat4"
        files     = glob.glob(os.path.join(self.HOME, f"**/*Yara_OVE/**/*{modelname}.urdf.xacro"), recursive=True)
        if len(files) > 0:
            urdffilepath = files[0]
            del (files)
        else:
            raise IOError(f"File {modelname}.urdf.xacro does not exist")

        # -->TRANSFORM THE XACRO FILE TO URDF
        os.system(f"xacro {urdffilepath} > {modelname}.urdf")

        # -->SPAWN THE MODEL IN THE GAZEBO SIMULATION
        model_namespace = "eboat"
        ipose = None
        self.spawnURDFModel(model_namespace, modelname, ipose=ipose)

        # -->DEFINE THE NECESSARY ROS TOPICS AND SERVICES
        boomAng_pub = rospy.Publisher(f"/{model_namespace}/control_interface/sail", Float32, queue_size=1)
        rudderAng_pub = rospy.Publisher(f"/{model_namespace}/control_interface/rudder", Float32, queue_size=1)
        # propVel_pub = rospy.Publisher(f"/{model_namespace}/control_interface/propulsion", Int16, queue_size=1)
        wind_pub = rospy.Publisher(f"/eboat/atmosferic_control/wind", Point, queue_size=1)

        # -->SERACH FOR THE SDF FILE DESCRIBING THE WAYPOINT
        files = glob.glob(os.path.join(self.HOME, f"**/*Yara_OVE/**/*wayPointMarker/model.sdf"), recursive=True)
        if len(files) > 0:
            sdffilepath = files[0]
            del (files)
        else:
            raise IOError(f"File wayPointMarker/model.sdf does not exist")

        #-->PATH PLANNING FOR THE TEST MISSION
        baseDist    = baseDistance
        baseVec     = np.array([1.0, 0.0])
        path2follow = [baseVec * baseDist]
        # thetaVec    = [-30     , -80     , -140    , 180     , 90          , 40      , -40     , 0       , 0       ]
        # thetaVec     = [-30     , -80    , -140     , 145     , 90          , 40      , -40     , 0       , 0       ]
        # D           = [baseDist, baseDist, baseDist, baseDist, 4 * baseDist, baseDist, baseDist, baseDist, baseDist]
        thetaVec = [0, 90]
        D        = [baseDist, baseDist]
        #========================================================
        # theta_wind = 180
        # theta_model = (theta_wind > 150) * 0.7854 - (theta_wind < -150) * 0.7854
        # --------------------------------
        model_pose               = Pose()
        quaternion               = quaternion_from_euler(0, 0, 0)
        model_pose.position.x    = 0.0
        model_pose.position.y    = 0.0
        model_pose.position.z    = 0.0
        model_pose.orientation.x = quaternion[0]
        model_pose.orientation.y = quaternion[1]
        model_pose.orientation.z = quaternion[2]
        model_pose.orientation.w = quaternion[3]
        self.spawnSDFModel("wayPointMarker", sdffilepath, model_pose)
        time.sleep(5)

        #--> INITIALIZE THE SENSOR HUD
        sensors = subprocess.Popen([sys.executable, os.path.join(
            "/home/eduardo/USVSim/eboat_ws/src/eboat_gz_1/eboat_control/python/projects/ESailor", "sensor_array.py")])
        # camera_raw = subprocess.Popen([sys.executable, "./camera_raw.py"])
        # camera_proc = subprocess.Popen([sys.executable, "./camera_detection.py"])

        for i, theta in enumerate(thetaVec):
            rad = theta * np.pi / 180.0
            rot = np.array([[np.cos(rad), -np.sin(rad)], [np.sin(rad), np.cos(rad)]])
            path2follow.append(path2follow[-1] + np.dot((baseVec * D[i]), rot))

        #--> SET THE WIND
        wind_pub.publish(Point(6.17, 0.0, 0.0))
        # wind_pub.publish(Point(-4.243, -4.234, 0.0))

        if self._physics_properties().pause:
            self.pauseSim(False)

        delmodel = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        rospy.wait_for_service("/gazebo/delete_model")
        result = delmodel("wayPointMarker")

        for i, waypoint in enumerate(path2follow):
            if i > 0:
                self.DMAX = D[i-1] + 25
            else:
                self.DMAX = baseDist + 25
            model_pose.position.x = waypoint[0]
            model_pose.position.y = waypoint[1]
            # self.pauseSim(True)
            # self.setState("wayPointMarker", model_pose)
            self.spawnSDFModel("wayPointMarker", sdffilepath, model_pose)
            # self.pauseSim(False)

            print(f"Going to waypoint {i}: {waypoint}")

            duration  = 0
            mission   = True
            actionVec = []
            while (mission & (duration < 180)):
                obsData = None
                while (obsData is None):
                    try:
                        obsData = rospy.wait_for_message(f"/{model_namespace}/mission_control/observations",
                                                         Float32MultiArray,
                                                         timeout=20).data
                        obs = np.array(obsData)
                    except:
                        pass

                # action, _ = model.predict(self.rescaleObs(obs)[[0, 1, 2, 4, 5 ,6]])
                action, _ = model.predict(self.rescaleObs(obs))
                actionVec.append([((action[0] + 1) * 45.0), (action[1] * 60.0)])
                boomAng_pub.publish(actionVec[-1][0])
                rudderAng_pub.publish(actionVec[-1][1])
                print(f"Action: {actionVec[-1][0]} | {actionVec[-1][1]}")

                if obs[0] <= 5:
                    mission = False
                else:
                    duration += 1

            if i > -1:
                break

            rospy.wait_for_service("/gazebo/delete_model")
            _ = delmodel("wayPointMarker")

        sensors.kill()
        # camera_raw.kill()
        # camera_proc.kill()


        plt.plot(0, 0, 'x', label='0')
        for i, waypoint in enumerate(path2follow):
            print(waypoint)
            plt.plot(-waypoint[1], waypoint[0], 'o', label=f"{i+1}")
        # plt.xlim(-410, 100)
        # plt.ylim(-100, 410)
        plt.legend()
        plt.grid()
        plt.show()

    def nav(self, model, boomAng_pub, rudderAng_pub, propPwr_pub, model_namespace="eboat"):
        duration = 0
        mission = True
        actionVec = []
        states_and_actions = []
        while (mission & (duration < 180)):
            obsData = None
            while (obsData is None):
                try:
                    obsData = rospy.wait_for_message(f"/{model_namespace}/mission_control/observations",
                                                     Float32MultiArray,
                                                     timeout=20).data
                    obs = np.array(obsData)
                except:
                    pass

            action, _ = model.predict(self.rescaleObs(obs))
            actionVec.append([((action[0] + 1) * 45.0), (action[1] * 60.0), np.int16(action[2] * 5)])
            boomAng_pub.publish(actionVec[-1][0])
            rudderAng_pub.publish(actionVec[-1][1])
            propPwr_pub.publish(actionVec[-1][2])
            print(f"Action: {actionVec[-1][0]} | {actionVec[-1][1]} | {actionVec[-1][2]}")
            states_and_actions.append(list(obs) + actionVec)

            if obs[0] <= 5:
                mission = False
            else:
                duration += 1

        return states_and_actions

    def pidController(self, obs):
        abs_wind = abs(obs[4])
        if (abs_wind > 150 and abs_wind < 180):
            epwr = 2
        else:
            epwr = 0
        pid_rudder = PID(0.5, -0.05, 0.0)
        pid_rudder.output_limits = (-60, 60)
        bang = abs((abs_wind / 2) - 90.0)
        rang = pid_rudder(obs[1])

        return [bang, rang, epwr]

    def _laser_scan_callback(self, data):
        laser_ranges = np.asarray(data.ranges)
        laser_ranges[laser_ranges == np.inf] = data.range_max

        self.laser_scan[4] = np.min(laser_ranges[0:23])
        self.laser_scan[3] = np.min(laser_ranges[24:47])
        self.laser_scan[2] = np.min(laser_ranges[48:72])
        self.laser_scan[1] = np.min(laser_ranges[73:96])
        self.laser_scan[0] = np.min(laser_ranges[97:120])

    def createObstacles(self):
        HOME = os.path.expanduser('~')
        sdffilepath = []
        for i in [2]: #[2, 3]:
            files = glob.glob(os.path.join(HOME, f"**/yara_ws/**/*Yara_OVE/**/box{i}/model.sdf"), recursive=True)
            if len(files) > 0:
                sdffilepath = files[0]
                del(files)
            else:
                raise IOError(f"File wayPointMarker/model.sdf does not exist")

            model_namespace = f"box_{i}"
            if i == 2:
                opose = Pose()
                opose.position.x = 70
                opose.position.y = 1
            else:
                opose.position.x = 135
                opose.position.y = -4
            self.spawnSDFModel(model_namespace, sdffilepath, ipose=opose)

    def testModel2(self, model, baseDistance = 100.0, rlagent = "PPO", wind_speed = 9, obstacles = False):
        # --> LAUNCH GAZEBO SIMULATION IF IT IS NOT RUNNING YET
        if self._roslaunch == None:
            print("Gazebo simulation was not started by the user.\nStarting simulation with empy ocean world.")
            self.launchGazeboSimulation()

        # --> WAIT FOR PHYSICS
        rospy.wait_for_service('/gazebo/get_physics_properties')

        # -->SERACH FOR THE URDF FILE DESCRIBING THE BOAT
        modelname = f"eboat4"
        files = glob.glob(os.path.join(self.HOME, f"**/*Yara_OVE/**/*{modelname}.urdf.xacro"), recursive=True)
        if len(files) > 0:
            urdffilepath = files[0]
            del (files)
        else:
            raise IOError(f"File {modelname}.urdf.xacro does not exist")

        # -->TRANSFORM THE XACRO FILE TO URDF
        os.system(f"xacro {urdffilepath} > {modelname}.urdf")

        # -->SPAWN THE MODEL IN THE GAZEBO SIMULATION
        model_namespace = "eboat"
        ipose = None
        self.spawnURDFModel(model_namespace, modelname, ipose=ipose)

        # -->DEFINE THE NECESSARY ROS TOPICS AND SERVICES
        boomAng_pub   = rospy.Publisher(f"/{model_namespace}/control_interface/sail", Float32, queue_size=1)
        rudderAng_pub = rospy.Publisher(f"/{model_namespace}/control_interface/rudder", Float32, queue_size=1)
        propPwr_pub   = rospy.Publisher(f"/{model_namespace}/control_interface/propulsion", Int16, queue_size=1)
        wind_pub      = rospy.Publisher(f"/eboat/atmosferic_control/wind", Point, queue_size=1)

        # -->SERACH FOR THE SDF FILE DESCRIBING THE WAYPOINT
        files = glob.glob(os.path.join(self.HOME, f"**/*Yara_OVE/**/*wayPointMarker/model.sdf"), recursive=True)
        if len(files) > 0:
            sdffilepath = files[0]
            del (files)
        else:
            raise IOError(f"File wayPointMarker/model.sdf does not exist")

        # --> INITIALIZE THE SENSOR HUD
        sensor_hud = None
        # sensors = subprocess.Popen([sys.executable, os.path.join(
        #     "/home/eduardo/USVSim/eboat_ws/src/eboat_gz_1/eboat_control/python/projects/ESailor", "sensor_array.py")])
        # camera_raw = subprocess.Popen([sys.executable, "./camera_raw.py"])
        # camera_proc = subprocess.Popen([sys.executable, "./camera_detection.py"])

        # -->PATH PLANNING FOR THE TEST MISSION
        baseDist    = baseDistance
        baseVec     = np.array([1.0, 0.0])
        path2follow = [baseVec * baseDist]
        laser_scan  = None
        if obstacles:
            thetaVec = [0, 0, 0, 0, 0, 0]
            D        = np.arange(1, (len(thetaVec) + 1)) * baseDist
            # --> Distance detection rays
            rospy.Subscriber("/eboat/laser/scan", LaserScan, self._laser_scan_callback)
            laser_scan = np.zeros(5, dtype=int)
            rospy.logdebug("Waiting for /scan to be READY...")
            while ((laser_scan is None) and (not rospy.is_shutdown())):
                try:
                    laser_scan = rospy.wait_for_message("/eboat/laser/scan", LaserScan, timeout=1.0)
                    rospy.logdebug("Current /eboat/laser/scan READY=>")
                except:
                    rospy.logerr("Current /eboat/laser/scan not ready yet, retrying for getting laser_scan")

            self.createObstacles()
        else:
            thetaVec = [-45     , -90     , -135    , -180    , 45      , 135]
            D        = [baseDist, baseDist, baseDist, baseDist, baseDist, baseDist]
        # ========================================================
        # theta_wind = 180
        # theta_model = (theta_wind > 150) * 0.7854 - (theta_wind < -150) * 0.7854
        # --------------------------------
        model_pose = Pose()
        quaternion = quaternion_from_euler(0, 0, 0)
        model_pose.position.x = 0.0
        model_pose.position.y = 0.0
        model_pose.position.z = 0.0
        model_pose.orientation.x = quaternion[0]
        model_pose.orientation.y = quaternion[1]
        model_pose.orientation.z = quaternion[2]
        model_pose.orientation.w = quaternion[3]
        self.spawnSDFModel("wayPointMarker", sdffilepath, model_pose)
        time.sleep(5)

        for i, theta in enumerate(thetaVec):
            rad = theta * np.pi / 180.0
            rot = np.array([[np.cos(rad), -np.sin(rad)], [np.sin(rad), np.cos(rad)]])
            path2follow.append(path2follow[-1] + np.dot((baseVec * D[i]), rot))

        # --> SET THE WIND
        ws = wind_speed
        wind_pub.publish(Point((ws * 0.51444), 0.0, 0.0))

        if self._physics_properties().pause:
            self.pauseSim(False)

        states_and_actions = []
        for i, waypoint in enumerate(path2follow):
            if i > 0:
                self.DMAX = D[i - 1] + 25
            else:
                self.DMAX = baseDist + 25
            model_pose.position.x = waypoint[0]
            model_pose.position.y = waypoint[1]
            # self.pauseSim(True)
            self.setState("wayPointMarker", model_pose)
            # time.sleep(5)
            # self.pauseSim(False)

            print(f"Going to waypoint {i}/{len(path2follow)}: {waypoint}")
            duration = 0
            mission = True
            while (mission & (duration < 90)):
                obsData = None
                while (obsData is None):
                    try:
                        obsData = rospy.wait_for_message(f"/{model_namespace}/mission_control/observations",
                                                         Float32MultiArray,
                                                         timeout=20).data
                        obs = np.array(obsData)
                    except:
                        pass
                dfo = self.laser_scan

                if model != None:
                    print("\n------------\n", self.laser_scan)
                    robs      = np.concatenate((self.rescaleObs(obs)[:5], ((dfo  / 30.0) - 1)), axis = 0)
                    action, _ = model.predict(robs)
                    actionROS = [((action[0] + 1) * 45.0), (action[1] * 60.0), np.int16(action[2] * 5)]
                else:
                    actionROS = self.pidController(obs)

                boomAng_pub.publish(actionROS[0])
                rudderAng_pub.publish(actionROS[1])
                propPwr_pub.publish(actionROS[2])
                print(f"Observations: {obs}")
                print(f"              {dfo}")
                print("----------------------------------------")
                print(f"Action      : {actionROS[0]} | {actionROS[1]} | {actionROS[2]}")
                print("========================================")
                states_and_actions.append(list(obs) + actionROS)

                if (obs[0] <= 5):
                    mission = False
                    print(f"D = {obs[0]} / {duration}")
                else:
                    duration += 1

            if i > 0:
                break



        if sensor_hud != None:
            sensor_hud.kill()
        else:
            pass

        # print(states_and_actions)
        df = pd.DataFrame(states_and_actions, columns=["distance","dirang","surge","apwindSpd","apwindAng","boomAng",
                                                       "rudderAng", "propPwr", "roll", "X", "Y", "boomAct", "rudderAct",
                                                       "propPwrAct"])
        print(df)
        if model == None:
            df.to_csv(f"pid_mission_2_{ws}.csv", sep=";", index=False)
        else:
            df.to_csv(f"{rlagent}_mission_2_{ws}.csv", sep=";", index=False)

        y = df.X.values
        x = df.Y.values * (-1)
        plt.plot(x, y, color="red")
        plt.plot(0, 0, 'x', label='0')
        for i, waypoint in enumerate(path2follow):
            # print(waypoint)
            plt.plot(-waypoint[1], waypoint[0], 'o', label=f"{i + 1}")
        # plt.xlim(-410, 100)
        # plt.ylim(-100, 410)
        plt.legend()
        plt.grid()
        plt.show()

    def rescaleObs2(self, observations, DMAX):
        lobs = len(observations)
        if lobs > 9:
            lobs -= 2
        robs = np.zeros(lobs, dtype=np.float32)
        # --> Distance from the waypoint (m) [-1, 1] --> [0   , DMAX];
        robs[0] = (observations[0] + 1) * 0.5 * DMAX
        # --> Trajectory angle               [-1, 1] --> [-180, 180]
        robs[1] = observations[1] * 180.0
        # --> Boat surge velocity (m/s)      [-1, 1] --> [-10   , 10 ]
        robs[2] = observations[2] * 10
        # --> Aparent wind speed (m/s)       [-1, 1] --> [0   , 30]
        robs[3] = (observations[3] + 1) * 15.0
        # --> Apparent wind angle            [-1, 1] --> [-180, 180]
        robs[4] = observations[4] * 180.0
        # --> Boom angle                     [-1, 1] --> [0   , 90]
        robs[5] = (observations[5] + 1) * 45.0
        # --> Rudder angle                   [-1, 1] --> [-60 , 60 ]
        robs[6] = observations[6] * 60.0
        # --> Electric propulsion speed      [-1, 1] --> [-5, 5]
        robs[7] = observations[7] * 5.0
        # --> Roll angle                     [-1, 1] --> [-180, 180]
        robs[8] = observations[8] * 180.0

        return robs

    def humanPolicy(self,
                 envid      = "Eboat92-v0",
                 numofsteps = 120,
                 ):

        #--> LAUNCH GAZEBO SIMULATION IF IT IS NOT RUNNING YET
        if self._roslaunch == None:
            print("Gazebo simulation was not started by the user.\nStarting simulation with empy ocean world.")
            self.launchGazeboSimulation("/home/eduardo/USVSim/yara_ws/src/Yara_OVE/eboat_gazebo/launch/empty_ocean.launch")
            # self.launchGazeboSimulation()

        #--> ADJUST PHYSICS PROPERTIES
        # if not(self._holdPhysiscsProperties):
        #     self._checkPhysicsProperties()

        #--> START GYMNASIUM ENVIRONMENT
        env = gym.make(envid)

        obs, _ = env.reset()
        robs   = self.rescaleObs2(obs, env.DMAX)
        print("Distance to the waypoint: {:6.3f}".format(robs[0]))
        print("Trajectory angle        : {:6.3f}".format(robs[1]))
        print("Surge velocity          : {:6.3f}".format(robs[2]))
        print("Apparent wind speed     : {:6.3f}".format(robs[3]))
        print("Apparent wind angle     : {:6.3f}".format(robs[4]))
        print("Boom angle              : {:6.3f}".format(robs[5]))
        print("Rudder angle            : {:6.3f}".format(robs[6]))
        print("Eletric propulsion power: {:6.3f}".format(robs[7]))
        print("Roll angle              : {:6.3f}".format(robs[8]))
        print("Attack angle            : {:6.3f}".format(180 - robs[5] - abs(robs[4])))
        print("============================================")

        step   = 0
        cumu   = 0
        action = np.zeros(2, dtype=np.float32)
        while True:
            input_action = input("\nChose an action in the format [boom angle rudder angle]:").split(" ")
            if len(input_action) > 1:
                action[0] = (np.float32(input_action[0]) / 45.0) - 1
                action[1] = np.float32(input_action[1]) / 60.0
            #
            obs, reward, done, trunc, _ = env.step(action)
            robs  = self.rescaleObs2(obs, env.DMAX)
            cumu += reward
            print("Action                  : [{:6.3f}, {:6.3f}]".format((action[0]+1.0)*45.0, action[1]*60.0))
            print("Distance to the waypoint: {:6.3f}".format(robs[0]))
            print("Trajectory angle        : {:6.3f}".format(robs[1]))
            print("Surge velocity          : {:6.3f}".format(robs[2]))
            print("Apparent wind speed     : {:6.3f}".format(robs[3]))
            print("Apparent wind angle     : {:6.3f}".format(robs[4]))
            print("Boom angle              : {:6.3f}".format(robs[5]))
            print("Rudder angle            : {:6.3f}".format(robs[6]))
            print("Eletric propulsion power: {:6.3f}".format(robs[7]))
            print("Roll angle              : {:6.3f}".format(robs[8]))
            print("Attack angle            : {:6.3f}".format(180 - robs[5] - abs(robs[4])))
            print("-----------------------------------")
            print("Step return             : {:6.3f}".format(reward))
            print("Cumulated retrun        : {:6.3f}".format(cumu))
            print("Number of steps         : {:d}".format(step+1))
            print("============================================")

            if (done | trunc | (step >= numofsteps)):
                break
            else:
                step += 1

    def pidTest(self, baseDistance = 100.0):
        # --> LAUNCH GAZEBO SIMULATION IF IT IS NOT RUNNING YET
        if self._roslaunch == None:
            print("Gazebo simulation was not started by the user.\nStarting simulation with empy ocean world.")
            self.launchGazeboSimulation()

        # --> WAIT FOR PHYSICS
        rospy.wait_for_service('/gazebo/get_physics_properties')

        # -->SERACH FOR THE URDF FILE DESCRIBING THE BOAT
        modelname = f"eboat4"
        files = glob.glob(os.path.join(self.HOME, f"**/*Yara_OVE/**/*{modelname}.urdf.xacro"), recursive=True)
        if len(files) > 0:
            urdffilepath = files[0]
            del (files)
        else:
            raise IOError(f"File {modelname}.urdf.xacro does not exist")

        # -->TRANSFORM THE XACRO FILE TO URDF
        os.system(f"xacro {urdffilepath} > {modelname}.urdf")

        # -->SPAWN THE MODEL IN THE GAZEBO SIMULATION
        model_namespace = "eboat"
        ipose = None
        self.spawnURDFModel(model_namespace, modelname, ipose=ipose)

        # -->DEFINE THE NECESSARY ROS TOPICS AND SERVICES
        boomAng_pub = rospy.Publisher(f"/{model_namespace}/control_interface/sail", Float32, queue_size=1)
        rudderAng_pub = rospy.Publisher(f"/{model_namespace}/control_interface/rudder", Float32, queue_size=1)
        propPwr_pub = rospy.Publisher(f"/{model_namespace}/control_interface/propulsion", Int16, queue_size=1)
        wind_pub = rospy.Publisher(f"/eboat/atmosferic_control/wind", Point, queue_size=1)

        # -->SERACH FOR THE SDF FILE DESCRIBING THE WAYPOINT
        files = glob.glob(os.path.join(self.HOME, f"**/*Yara_OVE/**/*wayPointMarker/model.sdf"), recursive=True)
        if len(files) > 0:
            sdffilepath = files[0]
            del (files)
        else:
            raise IOError(f"File wayPointMarker/model.sdf does not exist")

        # -->PATH PLANNING FOR THE TEST MISSION
        baseDist = baseDistance
        baseVec = np.array([1.0, 0.0])
        path2follow = [baseVec * baseDist]
        thetaVec = [-45, -90, -135, -180, 45, 135]
        D = [baseDist, baseDist, baseDist, baseDist, baseDist, baseDist]
        # ========================================================
        # theta_wind = 180
        # theta_model = (theta_wind > 150) * 0.7854 - (theta_wind < -150) * 0.7854
        # --------------------------------
        model_pose = Pose()
        quaternion = quaternion_from_euler(0, 0, 0)
        model_pose.position.x = 0.0
        model_pose.position.y = 0.0
        model_pose.position.z = 0.0
        model_pose.orientation.x = quaternion[0]
        model_pose.orientation.y = quaternion[1]
        model_pose.orientation.z = quaternion[2]
        model_pose.orientation.w = quaternion[3]
        self.spawnSDFModel("wayPointMarker", sdffilepath, model_pose)
        time.sleep(5)

        # --> INITIALIZE THE SENSOR HUD
        # sensors = subprocess.Popen([sys.executable, os.path.join("/home/eduardo/USVSim/eboat_ws/src/eboat_gz_1/eboat_control/python/projects/ESailor", "sensor_array.py")])
        # camera_raw = subprocess.Popen([sys.executable, "./camera_raw.py"])
        # camera_proc = subprocess.Popen([sys.executable, "./camera_detection.py"])

        for i, theta in enumerate(thetaVec):
            rad = theta * np.pi / 180.0
            rot = np.array([[np.cos(rad), -np.sin(rad)], [np.sin(rad), np.cos(rad)]])
            path2follow.append(path2follow[-1] + np.dot((baseVec * D[i]), rot))

        # --> SET THE WIND
        wind_pub.publish(Point(6.17, 0.0, 0.0))
        # wind_pub.publish(Point(-4.243, -4.234, 0.0))

        if self._physics_properties().pause:
            self.pauseSim(False)

        delmodel = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        rospy.wait_for_service("/gazebo/delete_model")
        result = delmodel("wayPointMarker")

        states_and_actions = []
        for i, waypoint in enumerate(path2follow):
            if i > 0:
                self.DMAX = D[i - 1] + 25
            else:
                self.DMAX = baseDist + 25
            model_pose.position.x = waypoint[0]
            model_pose.position.y = waypoint[1]
            # self.pauseSim(True)
            # self.setState("wayPointMarker", model_pose)
            self.spawnSDFModel("wayPointMarker", sdffilepath, model_pose)
            # self.pauseSim(False)

            print(f"Going to waypoint {i}: {waypoint}")
            duration = 0
            mission = True
            while (mission & (duration < 20)):
                obsData = None
                while (obsData is None):
                    try:
                        obsData = rospy.wait_for_message(f"/{model_namespace}/mission_control/observations",
                                                         Float32MultiArray,
                                                         timeout=20).data
                        obs = np.array(obsData)
                    except:
                        pass

                abs_wind = abs(obs[4])
                if (abs_wind > 150 and abs_wind < 180):
                    epwr = 2
                else:
                    epwr = 0
                pid_rudder = PID(0.5, -0.05, 0.0)
                pid_rudder.output_limits = (-60, 60)
                bang      = abs((abs_wind / 2) - 90.0)
                rang      = pid_rudder(obs[1])
                actionROS = [bang, rang, epwr]

                print("----------------------------------------")
                print(f"{duration} - {obs[1]} / {rang} | {bang} | {epwr}")

                boomAng_pub.publish(actionROS[0])
                rudderAng_pub.publish(actionROS[1])
                propPwr_pub.publish(actionROS[2])
                # print(f"Observations: ", obs)
                # print("----------------------------------------")
                # print(f"Action      : {actionROS[0]} | {actionROS[1]} | {actionROS[2]}")
                states_and_actions.append(list(obs) + actionROS)

                if obs[0] <= 5:
                    mission = False
                else:
                    duration += 1

            if i > -1:
                break

            rospy.wait_for_service("/gazebo/delete_model")
            _ = delmodel("wayPointMarker")

        # sensors.kill()
        print(states_and_actions)
        # df = pd.DataFrame(states_and_actions, columns=["distance","dirang","surge","apwindSpd","apwindAng","boomAng",
        #                                                "rudderAng", "propPwr", "roll", "X", "Y", "boomAct", "rudderAct",
        #                                                "propPwrAct"])
        # print(df)
        #
        # x = df.X.values
        # y = df.Y.values
        # plt.plot(x, y, color="red")
        # plt.plot(0, 0, 'x', label='0')
        # for i, waypoint in enumerate(path2follow):
        #     # print(waypoint)
        #     plt.plot(-waypoint[1], waypoint[0], 'o', label=f"{i + 1}")
        # # plt.xlim(-410, 100)
        # # plt.ylim(-100, 410)
        # plt.legend()
        # plt.grid()
        # plt.show()

    def close(self):
        if self._roslaunch.pid != None:
            ppid = self._roslaunch.pid
            print(f"\n\n===================\nProcess id: {ppid}\n===================\n")
            os.system(f'ps -au eduardo | grep {self._roslaunch.pid}')
            os.killpg(os.getpgid(self._roslaunch.pid), signal.SIGTERM)

            print("\n\n\nCLOSE FUNCTION\n\n")

def main(argv):
    if len(argv) < 1:
        rlagent = 'PPO'
    else:
        rlagent = argv[0]
    print("RL agent :", rlagent)
    
    # refmodel = PPO.load("./models/PPO/esailor_93_A116_C116_29022024_18_28_51/esailor_model_1001472_steps.zip")
    # if rlagent == "PPO":
    #     refmodel = PPO.load(f"./models/PPO/esailor_93_A3232_C3232_03032024_19_58_50/esailor_model_501760_steps.zip")
    # elif rlagent == "SAC":
    #     refmodel = SAC.load(f"./policy/esailor_53_{rlagent}_A3232_C3232_03032024/esailor_model_501760_steps.zip")
    # else:
    #     refmodel = None
    refmodel = PPO.load(f"./models/PPO/esailor_103_A3232_C3232_11032024_23_02_16/esailor_model_501760_steps.zip")
    agent = esailor()
    # agent.training(rlagent=rlagent,
    #                policy="MlpPolicy",
    #                envid="Eboat103-v1", #"Eboat53-v0",
    #                numofsteps=(245 * 2048), #489 * 2048,
    #                refmodel=refmodel,
    #                actor=[32, 32],
    #                critic=[32, 32],
    #                sufix="esailor_103",
    #                logdir="logs")
    agent.testModel2(refmodel, rlagent = rlagent, wind_speed = 9, obstacles = True)
    # agent.humanPolicy(envid="Eboat92-v0", numofsteps=120)
    # agent.checkEnv(envid="Eboat103-v1")
    # agent.pidTest()
    agent.close()

if __name__ == "__main__":
    main(sys.argv[1:])