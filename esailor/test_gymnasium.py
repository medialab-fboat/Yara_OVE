#!/home/eduardo/miniconda3/envs/esailor/bin/python

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
from stable_baselines3 import A2C, PPO, DQN, SAC

#-->PYTORCH
import torch as th

if __name__ == "__main__":
    env = gym.make('GazeboOceanEboatEnvCC-v25')

    time.sleep(20)

    print("\n\n\nPASSANDO ADIANTE\n\n")

    env.close()
    os.system('/home/eduardo/USVSim/eboat_ws/kill_gaz.sh')