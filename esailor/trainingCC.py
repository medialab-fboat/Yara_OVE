#!/home/araujo/miniconda3/envs/esailor2/bin/python

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
from stable_baselines3 import PPO, SAC, A2C

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

def runSAC(policy, env, learning_rate=0.0003, buffer_size=1000000, learning_starts=100, batch_size=256, tau=0.005,
           gamma=0.99, train_freq=1, gradient_steps=1, action_noise=None, replay_buffer_class=None,
           replay_buffer_kwargs=None, optimize_memory_usage=False, ent_coef='auto', target_update_interval=1,
           target_entropy='auto', use_sde=False, sde_sample_freq=-1, use_sde_at_warmup=False, stats_window_size=100,
           tensorboard_log=None, policy_kwargs=None, verbose=0, seed=None, device='auto', _init_setup_model=True,
           sufix = ""):
    sufix = sufix + "_" + datetime.now().strftime("%d%m%Y_%H_%M_%S")
    models_dir = f"models/SAC/{sufix}"

    if not os.path.exists(models_dir):
        os.makedirs(models_dir)

    model = SAC(policy                 = policy,
                env                    = env,
                learning_rate          = learning_rate,
                buffer_size            = buffer_size,
                learning_starts        = learning_starts,
                batch_size             = batch_size,
                tau                    = tau,
                gamma                  = gamma,
                train_freq             = train_freq,
                gradient_steps         = gradient_steps,
                action_noise           = action_noise,
                replay_buffer_class    = replay_buffer_class,
                replay_buffer_kwargs   = replay_buffer_kwargs,
                optimize_memory_usage  = optimize_memory_usage,
                ent_coef               = ent_coef,
                target_update_interval = target_update_interval,
                target_entropy         = target_entropy,
                use_sde                = use_sde,
                sde_sample_freq        = sde_sample_freq,
                use_sde_at_warmup      = use_sde_at_warmup,
                stats_window_size      = stats_window_size,
                tensorboard_log        = tensorboard_log,
                policy_kwargs          = policy_kwargs,
                verbose                = verbose,
                seed                   = seed,
                device                 = device,
                _init_setup_model      = _init_setup_model
                )

    tb_log_name = f"SAC_{sufix}"

    return model, models_dir, tb_log_name

def runA2C(policy, env,learning_rate=0.0007, n_steps=5, gamma=0.99, gae_lambda=1.0, ent_coef=0.001, vf_coef=0.5,
           max_grad_norm=0.5, rms_prop_eps=1e-05, use_rms_prop=True, use_sde=False, sde_sample_freq=-1,
           normalize_advantage=False, tensorboard_log=None, policy_kwargs=None, verbose=0, seed=None, device='auto',
           init_setup_model=True, sufix = ""):

    sufix      = sufix + "_" + datetime.now().strftime("%d%m%Y_%H_%M_%S")
    models_dir = f"models/A2C/{sufix}"

    if not os.path.exists(models_dir):
        os.makedirs(models_dir)

    model = A2C(policy              = policy,
                env                 = env,
                learning_rate       = learning_rate,
                n_steps             = n_steps,
                gamma               = gamma,
                gae_lambda          = gae_lambda,
                ent_coef            = ent_coef,
                vf_coef             = vf_coef,
                max_grad_norm       = max_grad_norm,
                rms_prop_eps        = rms_prop_eps,
                use_rms_prop        = use_rms_prop,
                use_sde             = use_sde,
                sde_sample_freq     = sde_sample_freq,
                normalize_advantage = normalize_advantage,
                tensorboard_log     = tensorboard_log,
                policy_kwargs       = policy_kwargs,
                verbose             = verbose,
                seed                = seed,
                device              = device,
                _init_setup_model   = init_setup_model
                )

    tb_log_name = f"A2C_{sufix}"

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

def runTraining(env, logdir, agent = "PPO", sufix="model", refmodel=None):
    actor  = [11, 11]
    critic = [9, 9]
    if agent == "SAC":
        policy_kwargs = dict(activation_fn=th.nn.ReLU,
                             net_arch=(dict(pi=actor, qf=critic))
                             )
        model, models_dir, TB_LOG_NAME = runSAC(policy="MlpPolicy",
                                                env=env,
                                                tensorboard_log=logdir,
                                                batch_size=64,
                                                ent_coef=0.001,
                                                verbose=0,
                                                policy_kwargs=policy_kwargs,
                                                sufix=sufix)
    else:
        policy_kwargs = dict(activation_fn=th.nn.ReLU,
                             net_arch=(dict(pi=actor, vf=critic))
                             )
        model, models_dir, TB_LOG_NAME = runPPO(policy="MlpPolicy",
                                                env=env,
                                                tensorboard_log=logdir,
                                                ent_coef=0.001,
                                                verbose=0,
                                                policy_kwargs=policy_kwargs,
                                                sufix=sufix)

    if refmodel != None:
        # -->SET THE NEURAL NETWORK PARAMETERS EQUAL TO A LOADED PREVIOUS TRAINED NEURAL NETWORK
        model.set_parameters(refmodel.get_parameters)

    SAVESTEPS = 100+1
    TIMESTEPS = 2048*5
    start     = time.time()
    reftime   = start
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
        if (i % 10) > 0:
            avtime = (timeB - reftime) / (i % 10)
        else:
            avtime  = (timeB - reftime) / 10
            reftime = timeB
        print(f"Time spent in this iteration: {htime(timeB - timeA)}")
        print(f"Average time per iteration  : {htime(avtime)}")
        print(f"Elapsed time                : {htime(timeB - start)}")
        print(f"Remaining time              : {htime((SAVESTEPS - 1 - i)*avtime)}")

def main():
    logdir = "logs"
    if not os.path.exists(logdir):
        os.makedirs(logdir)

    env = gym.make('EboatStraightLineEnvCC29-v0')
    # runTraining(env, logdir, "PPO", sufix="ensign29_7_winds_10m_straight")
    runTraining(env, logdir, "SAC", sufix="ensign29_7_winds_5m_straight")

    # refmodel = PPO.load("/home/araujo/yara_ws/src/Yara_OVE/esailor/models/PPO/ensign29_7_winds_10m_straight_09082023_10_34_00/eboat_ocean_50.zip")
    # runTraining(env, "PPO", sufix="sailor29_7_winds_5m_straight", refmodel=refmodel)

    print("---------------------------------------------------------\n")

    env.close()
    os.system('./kill_gaz.sh')

def rot(self, modulus, theta):
    R = np.array([[np.cos(theta), -np.sin(theta)],[np.sin(theta), np.cos(theta)]], dtype=np.float32)

    return np.dot(np.array([1, 0], dtype=np.float32) * modulus, R)

if __name__ == '__main__':
    main()
    # count = 0
    # for _ in range(50000):
    #     theta_boat = np.random.randint(low=-179, high=180)
    #     wind_speed = np.random.randint(low=3, high=12)
    #     theta_wind = np.random.randint(low=-179, high=180)
    #
    #     val = abs(theta_boat) + abs(theta_wind)
    #     if (val > 150) & (val < 210):
    #         print(f"FALHA {count} -- {val} / theta_boat = {theta_boat} / theta_wind = {theta_wind}")
    #         count += 1
    #
    # env = gym.make(f'GazeboOceanEboatEnvCC25-v0')
    # for _ in range(5):
    #     env.reset()
    #     # time.sleep(5)
    #
    # env.close()
    # os.system('./kill_gaz.sh')