#!/home/eduardo/miniconda3/envs/esailor/bin/python

#-->PYTHON UTIL
import time
import random
import numpy as np
import glob
from datetime import datetime
import sys, os, signal, subprocess
from matplotlib import pyplot as plt

#-->ROS
import rospy

#-->GYM
import gymnasium as gym

#-->GAZEBO
import esailor_gym
from std_srvs.srv import Empty
from std_msgs.msg import Float32, Int16, Float32MultiArray
from gazebo_msgs.srv import SetModelState, GetModelState, SpawnModel, DeleteModel, SetPhysicsProperties
from geometry_msgs.msg import Point, Pose, Vector3
from gazebo_msgs.msg import ODEPhysics, ModelState

#-->STABLE-BASELINES3
from gymnasium import wrappers
from stable_baselines3 import PPO, SAC, A2C
from stable_baselines3.common.callbacks import CheckpointCallback

#-->PYTORCH
import torch as th

class esailor():
    def __init__(self, path2launchfile = None):
        random_number    = random.randint(10000, 15000)
        self.port_ros    = str(random_number)
        self.port_gazebo = str(random_number + 1)
        # self.port_ros = "11311"
        # self.port_gazebo = "11345"

        #-->EXPORT ADDRESS FOR THE ROS MASTER NODE AND THE GAZEBO SERVER
        os.environ["ROS_MASTER_URI"]    = "http://localhost:" + self.port_ros
        os.environ["GAZEBO_MASTER_URI"] = "http://localhost:" + self.port_gazebo

        #-->LOG FILE
        logfile = "esailor_" + self.port_ros + ".log"
        with open(logfile, "w") as f:
            f.writelines("ROS_MASTER_URI = http://localhost:" + self.port_ros + "\n")
            f.writelines("GAZEBO_MASTER_URI = http://localhost:" + self.port_gazebo + "\n")

        #-->SEARCH FOR A LAUNCH FILE
        HOME = os.path.expanduser('~')
        if path2launchfile == None:
            print("As the user did not provide a viable launch file, I will search for one.\nThis may take a while, pelase wait!")
            files = glob.glob(os.path.join(HOME,"**/*empty_ocean.launch"), recursive=True)
            if len(files) > 0:
                path2launchfile = files[0]
                del(files)
            else:
                path2launchfile = input("\nI did not find a viable launch file!\nPlease provide an valid path:\n")
                if (((len(path2launchfile) > 0) & (not(os.path.exists(path2launchfile)))) | (len(path2launchfile) == 0)):
                    raise IOError("File " + path2launchfile + " does not exist")

        with open(logfile, "a") as f:
            f.writelines(f"\nLaunch file used to start the simulation: {path2launchfile}\n")

        #-->LAUNCH THE SIMULATION USING SUBPROCESS
        ros_path = os.path.dirname(subprocess.check_output(["which", "roscore"]))
        self._roslaunch = subprocess.Popen([sys.executable, os.path.join(ros_path, b"roslaunch"), "-p", self.port_ros, path2launchfile])

        #-->AUXILIARY VARIABLES
        self.DMAX = 1.0

    def training(self, rlagent = "PPO", policy = "MlpPolicy", numofsteps=500000, refmodel = None):

        env    = gym.make("Eboat92_5_SternWind-v0")

        # sufix  = "esailor_8_winds_" + datetime.now().strftime("%d%m%Y_%H_%M_%S")
        sufix = "esailor_2c_winds_" + datetime.now().strftime("%d%m%Y_%H_%M_%S")
        logdir = "logs"

        if not os.path.exists(logdir):
            os.makedirs(logdir)

        actor = [11, 11]
        critic = [9, 9]

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

    def envTest(self, numofsteps=24):

        env = gym.make("Eboat92_5_SternWind-v0")

        env.reset()
        print(f"\n\n=======================\nPublicando valor vetor de vento\n=======================\n")
        env.wind_pub.publish(Point(6.17, 0, 0))
        # env.propVel_pub.publish(2)
        cumurwd = 0
        for i in range(numofsteps):
            input_action = input("Enter an action in the format (boom angle , rudder angle):").split(" ")
            # if i < numofsteps - 3:
            #     input_action = [89, 0]
            # else:
            #     input_action = [89, 5]
            if len(input_action) > 1:
                action = np.array(input_action, dtype=np.float32)
                action[0] = (action[0] / 45.0) - 1
                action[1] = action[1] / 60.0

            robs, rwd, _, _, _ = env.step(action)
            cumurwd += rwd
            print(f"surge  : {robs[2]*10} m/s ({robs[2]*10*1.94384} knots")
            print(f"reward : {rwd}")
            print(f"cumurwd: {cumurwd}")
            print("--------------------------------------------------")

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

        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            result = set_state(state)
            assert result.success is True
        except rospy.ServiceException:
            print("/gazebo/get_model_state service call failed")

    def testModel(self, model):
        # -->INITIALIZE A ROS NODE FOR THE TRAINING PROCESS
        try:
            rospy.init_node(f"gym", anonymous=True)
        except:
            print("ROSMASTER is not running!")
            print(time.time())
            exit(1)

        # -->DEFINE A UNIQUE MODEL NAME FOR OUR BOAT
        modelname = f"eboat4"

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
        model_namespace = "eboat"
        self.spawnURDFModel(model_namespace, modelname, ipose=None)

        # -->SERACH FOR THE SDF FILE DESCRIBING THE WAYPOINT
        HOME = os.path.expanduser('~')
        files = glob.glob(os.path.join(HOME, f"**/*Yara_OVE/**/*wayPointMarker/model.sdf"), recursive=True)
        if len(files) > 0:
            sdffilepath = files[0]
            del (files)
        else:
            raise IOError(f"File wayPointMarker/model.sdf does not exist")

        # -->SPAWN THE WAYPOINT MARKER IN THE GAZEBO SIMULATION
        waypoint_namespace = f"wayPointMarker"
        ipose = Pose()
        ipose.position.x = 100.0
        ipose.position.y = 0.0
        ipose.position.z = 0.0
        self.spawnSDFModel(waypoint_namespace, sdffilepath, ipose)

        # -->DEFINE THE NECESSARY ROS TOPICS AND SERVICES
        boomAng_pub = rospy.Publisher(f"/{model_namespace}/control_interface/sail", Float32, queue_size=1)
        rudderAng_pub = rospy.Publisher(f"/{model_namespace}/control_interface/rudder", Float32, queue_size=1)
        # propVel_pub = rospy.Publisher(f"/{model_namespace}/control_interface/propulsion", Int16, queue_size=1)
        wind_pub = rospy.Publisher(f"/eboat/atmosferic_control/wind", Point, queue_size=1)
        unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        # self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        # self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        # self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        # self.get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        time.sleep(5)

        # set_physics_properties = rospy.ServiceProxy("/gazebo/set_physics_properties", SetPhysicsProperties)
        # try:
        #     result = set_physics_properties(time_step       = np.float64(0.01),
        #                                     max_update_rate = np.float64(100.0),
        #                                     gravity         = Vector3(0.0, 0.0, -9.81)
        #                                     # gazebo_msgs/ODEPhysics ode_config
        #                                    )
        # except rospy.ServiceException:
        #     result = "/gazebo/SetPhysicsProperties service call failed"

        # -->UNPAUSE SIMULATION
        rospy.wait_for_service("/gazebo/unpause_physics")
        try:
            unpause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/unpause_physics service call failed!"))

        # -->SERACH FOR THE SDF FILE DESCRIBING THE DESTINATION POINT
        HOME = os.path.expanduser('~')
        files = glob.glob(os.path.join(HOME, f"**/*Yara_OVE/**/*destinationMarker/model.sdf"), recursive=True)
        if len(files) > 0:
            sdffilepath = files[0]
            del (files)
        else:
            raise IOError(f"File wayPointMarker/model.sdf does not exist")

        # -->GET OBSERVATIONS AT TIME STAMP 0 (INITIAL STATE)
        obsData = None
        while (obsData is None):
            try:
                obsData = rospy.wait_for_message(f"/{model_namespace}/mission_control/observations",
                                                 Float32MultiArray,
                                                 timeout=20).data
                obs  = np.array(obsData)
            except:
                pass

        self.DMAX = obs[0] + 25
        # wind_pub.publish(Point(6.17, 0.0, 0.0))
        wind_pub.publish(Point(-4.243, -4.234, 0.0))

        ipose.position.x = 0.0
        ipose.position.y = 0.0
        ipose.position.z = 0.0
        #for i, theta in enumerate([90, 30, -30, -130, 180, -90]):
        # thetaVec = [90, 30, -30, -130, 180, -90]
        thetaVec = [90]
        for i, theta in enumerate(thetaVec):
            rad = theta * (np.pi / 180.0)
            ipose.position.x += 100 * np.sin(rad)
            ipose.position.y += 100 * np.cos(rad)
            self.spawnSDFModel(f"destination_{i}", sdffilepath, ipose)

        ipose.position.x = 0.0
        ipose.position.y = 0.0
        ipose.position.z = 0.0
        actionVec = []
        for i, theta in enumerate(thetaVec):
            rad = theta * (np.pi / 180.0)
            ipose.position.x += 100 * np.sin(rad)
            ipose.position.y += 100 * np.cos(rad)

            self.setState("wayPointMarker", ipose)

            obsData = None
            while (obsData is None):
                try:
                    obsData = rospy.wait_for_message(f"/{model_namespace}/mission_control/observations",
                                                     Float32MultiArray,
                                                     timeout=20).data
                    obs = np.array(obsData)
                except:
                    pass

            j = 0
            print("\n\n---------------------------------------\nENTRANDO NO LOOPING\n----------------------------------------------\n")
            while ((obs[0] > 5) & (j < 180)):
                action, _ = model.predict(self.rescaleObs(obs))
                actionVec.append([((action[0] + 1) * 45.0), (action[1] * 60.0)])
                boomAng_pub.publish(actionVec[-1][0])
                rudderAng_pub.publish(actionVec[-1][1])
                print(f"Action: {actionVec[-1][0]} | {actionVec[-1][1]}")


                obsData = None
                while (obsData is None):
                    try:
                        obsData = rospy.wait_for_message(f"/{model_namespace}/mission_control/observations",
                                                              Float32MultiArray,
                                                              timeout=20).data
                        obs  = np.array(obsData)
                    except:
                        pass

                j += 1
        # time.sleep(10)
        fig = plt.figure(figsize=[12,8])
        ax1 = fig.add_subplot(211)
        ax2 = fig.add_subplot(212)
        actionVec = np.array(actionVec)
        ax1.plot(actionVec[:, 0], color="tab:blue", label="Boom")
        ax2.plot(actionVec[:, 1], color="tab:red", label="Rudder")
        ax1.legend()
        ax2.legend()
        plt.show()

    def testLoadParams(self):
        actor         = [11, 11]
        critic        = [9, 9]
        policy_kwargs = dict(activation_fn=th.nn.ReLU,
                                 net_arch=(dict(pi=actor, vf=critic))
                                 )
        env   = gym.make("Eboat92_5_SternWind-v0")

        model = PPO(policy            = "MlpPolicy",
                    env               = env,
                    ent_coef          = 0.001, #0.0
                    # tensorboard_log   = logdir,
                    policy_kwargs     = policy_kwargs,
                    )

        modelref = PPO.load("./models/PPO/esailor_01092023_12_52_49/esailor_model_1000000_steps.zip")
        params   = modelref.get_parameters()

        print(f"\n\n===================\nParâmetros carregados!\n===================\n")

        model.set_parameters(params)

        print(f"\n\n===================\nParâmetros setados!\n===================\n")

    def close(self):
        ppid = self._roslaunch.pid
        print(f"\n\n===================\nProcess id: {ppid}\n===================\n")
        os.system(f'ps -au eduardo | grep {self._roslaunch.pid}')
        os.killpg(os.getpgid(self._roslaunch.pid), signal.SIGTERM)

        print("\n\n\nCLOSE FUNCTION\n\n")

if __name__ == "__main__":
    # refmodel = PPO.load("./models/PPO/esailor_01092023_12_52_49/esailor_model_1000000_steps.zip")
    # refmodel = PPO.load("./models/PPO/esailor_6_winds_05092023_11_22_52/esailor_model_1001472_steps.zip")
    # refmodel = PPO.load("./models/PPO/esailor_2_winds_06092023_23_18_14/esailor_model_1001472_steps.zip")
    # refmodel = PPO.load("./models/PPO/esailor_6_winds_07092023_09_37_10/esailor_model_1001472_steps.zip")
    # refmodel = PPO.load("./models/PPO/esailor_8_winds_11092023_21_55_00/esailor_model_1001472_steps.zip")
    # refmodel = PPO.load("./models/PPO/test_esailor_2f_winds_12092023_12_56_39/esailor_model_680000_steps")
    # refmodel = PPO.load("./models/PPO/esailor_2a_winds_14092023_12_24_47/esailor_model_970000_steps.zip")
    # refmodel = PPO.load("./models/PPO/esailor_2b_winds_15092023_11_53_43/esailor_model_1001472_steps.zip")
    agent = esailor()
    agent.training(numofsteps = 489 * 2048,
                   # refmodel   = refmodel
                  )
    # agent.envTest(numofsteps=40)
    # agent.testLoadParams()
    # agent.testModel(refmodel)
    agent.close()
    # delta = 1.0
    # for i in range(121):
    #     print(-60+i*delta)