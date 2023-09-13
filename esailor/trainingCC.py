#!/home/araujo/miniconda3/envs/esailor/bin/python

#-->PYTHON UTIL
import os
import time
import random
import numpy as np
from datetime import datetime

#-->GYM
import gymnasium as gym

#-->GAZEBO GYM
import esailor_gym

#-->STABLE-BASELINES3
from gymnasium import wrappers
from stable_baselines3 import PPO, SAC

#-->PYTORCH
import torch as th


def truncate(value):
    ival = int(value)
    if ((value - ival) < 0.5):
        return ival
    else:
        return (ival + 1)

def runPPO(policy, env, learning_rate=0.0003, n_steps=2048, batch_size=64, n_epochs=10, gamma=0.99, gae_lambda=0.95,
           clip_range=0.2, clip_range_vf=None, normalize_advantage=True, ent_coef=0.0, vf_coef=0.5, max_grad_norm=0.5,
           use_sde=False, sde_sample_freq=-1, target_kl=None, tensorboard_log=None, policy_kwargs=None, verbose=0,
           seed=None, device='auto', init_setup_model=True, sufix = ""):

    sufix      = sufix + "_" + datetime.now().strftime("%d%m%Y_%H_%M_%S")
    models_dir = f"models/PPO/{sufix}"

    # if not os.path.exists(models_dir):
    #     os.makedirs(models_dir)

    model = PPO(policy              = policy,
                env                 = env,
                learning_rate       = learning_rate,
                n_steps             = n_steps,
                batch_size          = batch_size,
                n_epochs            = n_epochs,
                gamma               = gamma,
                gae_lambda          = gae_lambda,
                clip_range          = clip_range,
                clip_range_vf       = clip_range_vf,
                # normalize_advantage = normalize_advantage,
                ent_coef            = ent_coef,
                vf_coef             = vf_coef,
                max_grad_norm       = max_grad_norm,
                use_sde             = use_sde,
                sde_sample_freq     = sde_sample_freq,
                target_kl           = target_kl,
                tensorboard_log     = tensorboard_log,
                policy_kwargs       = policy_kwargs,
                verbose             = verbose,
                seed                = seed,
                device              = device,
                _init_setup_model   = init_setup_model
                )

    tb_log_name = f"PPO_{sufix}"

    return model, models_dir, tb_log_name

def htime(input):
    if input >= 3600:
        h = int(input // 3600)
        t = input % 3600
        m = int(t // 60)
        s = t % 60
        return "{:02d}h {:02d}m {:4.2f}s".format(h, m, s)
    elif input >= 60:
        m = int(input // 60)
        s = input % 60
        return "{:02d}m {:4.2f}s".format(m, s)
    else:
        return "{:4.2f}s".format(input)

def actionRescale(action):
    raction = np.zeros(3, dtype = np.float32)
    #--> Eletric propulsion [-5, 5]
    raction[0] = action[0] * 5.0
    #--> Boom angle [0, 90]
    raction[1] = (action[1] + 1) * 45.0
    #--> Rudder angle [-60, 60]
    raction[2] = action[2] * 60.0
    return raction

def runTraining25(env, logdir, sufix="model35"):
    policy_kwargs = dict(activation_fn=th.nn.ReLU,
                         net_arch=(dict(pi=[32, 32], vf=[32, 32]))
                         )

    model, models_dir, TB_LOG_NAME = runPPO(policy          = "MlpPolicy",
                                            env             = env,
                                            tensorboard_log = logdir,
                                            ent_coef        = 0.001,
                                            verbose         = 0,
                                            policy_kwargs   = policy_kwargs,
                                            sufix           = sufix)

    SAVESTEPS = 50+1
    TIMESTEPS = 2048*5
    start     = time.time()
    model.save(f"{models_dir}/eboat_ocean_0")
    for i in range(1, SAVESTEPS):
        print("\n\n---------------------------------------------------------")
        print(f"iteration                   : {i}")
        timeA = time.time()

        model.learn(total_timesteps     = TIMESTEPS  ,
                    log_interval        = 1          ,
                    tb_log_name         = TB_LOG_NAME,
                    reset_num_timesteps = False      ,
                    progress_bar        = False
                    )
        model.save(f"{models_dir}/eboat_ocean_{i}")

        timeB  = time.time()
        avtime = (timeB - start) / i
        print(f"Time spent in this iteration: {htime(timeB - timeA)}")
        print(f"Average time per iteration  : {htime(avtime)}")
        print(f"Elapsed time                : {htime(timeB - start)}")
        print(f"Remaining time              : {htime((SAVESTEPS - 1 - i)*avtime)}")

def main():
    logdir = "logs"
    if not os.path.exists(logdir):
        os.makedirs(logdir)

    env = gym.make(f'GazeboOceanEboatEnvCC35-v0')
    runTraining25(env, logdir)
   

    print("---------------------------------------------------------\n")

    env.close()
    os.system('/home/araujo/yara_ws/src/Yara_OVE/esailor/kill_gaz.sh')

if __name__ == '__main__':
    main()