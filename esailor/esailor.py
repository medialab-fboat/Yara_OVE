#!/home/eduardo/miniconda3/envs/esailor/bin/python

#-->PYTHON UTIL
import time
import random
import numpy as np
import glob
from datetime import datetime
import sys, os, signal, subprocess

#-->ROS
import rospy

#-->GYM
import gymnasium as gym

#-->GAZEBO GYM
import esailor_gym
from geometry_msgs.msg import Point, Pose

#-->STABLE-BASELINES3
from gymnasium import wrappers
from stable_baselines3 import PPO, SAC, A2C
from stable_baselines3.common.callbacks import CheckpointCallback

#-->PYTORCH
import torch as th

class esailor():
    def __init__(self, path2launchfile = None):
        random_number    = random.randint(10000, 15000)
        # self.port_ros    = str(random_number)
        # self.port_gazebo = str(random_number + 1)
        self.port_ros = "11311"
        self.port_gazebo = "11345"

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

    def training(self, rlagent = "PPO", policy = "MlpPolicy", numofsteps=500000, refmodel = None):

        env    = gym.make("Eboat92_5_SternWind-v0")

        sufix  = "trash_esailor_6_winds_" + datetime.now().strftime("%d%m%Y_%H_%M_%S")
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
            model.set_parameters(refmodel.get_parameters)

        checkpoint_callback = CheckpointCallback(save_freq          = 5*2048,
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

    def envTest(self, numofsteps=24, refmodel = None):

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

    def testLoadParams(self):
        policy_kwargs = dict(activation_fn=th.nn.ReLU,
                                 net_arch=(dict(pi=actor, vf=critic))
                                 )

        model = PPO(policy            = policy,
                    env               = env,
                    ent_coef          = 0.001, #0.0
                    # tensorboard_log   = logdir,
                    policy_kwargs     = policy_kwargs,
                    )

        modelref = PPO.load("./models/PPO/esailor_01092023_12_52_49/esailor_model_1000000_steps.zip")
        params   = modelref.get_parameters()

    def close(self):
        ppid = self._roslaunch.pid
        print(f"\n\n===================\nProcess id: {ppid}\n===================\n")
        os.system(f'ps -au eduardo | grep {self._roslaunch.pid}')
        os.killpg(os.getpgid(self._roslaunch.pid), signal.SIGTERM)

        print("\n\n\nCLOSE FUNCTION\n\n")

if __name__ == "__main__":
    # refmodel = PPO.load("/home/eduardo/USVSim/yara_ws/src/Yara_OVE/esailor/models/PPO/ensign29_7_winds_10m_straight_09082023_10_34_00/eboat_ocean_50.zip")
    # params = refmodel.get_parameters()
    agent = esailor()
    # agent.training(numofsteps = 489 * 2048,
                   # refmodel   = refmodel
                  # )
    # agent.envTest(numofsteps=40)
    agent.testLoadParams()
    agent.close()