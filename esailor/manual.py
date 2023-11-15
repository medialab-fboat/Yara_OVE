#!/home/araujo/miniconda3/envs/esailor2/bin/python

#-->PYTHON UTIL
import os
import time
import random
import numpy as np
from datetime import datetime
import sys, os, signal, subprocess

#-->ROS
import rospy

#-->GYM
import gymnasium as gym

#-->GAZEBO GYM
import esailor_gym
from std_srvs.srv import Empty
from std_msgs.msg import Float32, Int16, Float32MultiArray
from geometry_msgs.msg import Point
from gazebo_msgs.srv import SetModelState, SpawnModel, DeleteModel

#-->STABLE-BASELINES3
from gymnasium import wrappers
from stable_baselines3 import PPO, SAC, A2C

#-->PYTORCH
import torch as th

#-->MY IMPORTS
from test_model import rot, getObservations, spawnModel, resetGazEnv

def observationRescale(observations):
    lo   = len(observations)
    robs = np.zeros(lo, dtype = np.float32)
    #--> Distance from the waypoint (m) [0   , 200];
    robs[0] = (observations[0] + 1) * 125 * 0.5
    #--> Trajectory angle               [-180, 180];
    robs[1] = observations[1] * 180.0
    #--> Boat linear velocity (m/s)     [0   , 10 ];
    robs[2] = (observations[2] + 1) * 5
    #--> Aparent wind speed (m/s)       [0   , 30];
    robs[3] = (observations[3] + 1) * 15
    #--> Apparent wind angle            [-180, 180]
    robs[4] = observations[4] * 180.0
    if lo > 5:
        # --> Boom angle                     [0   , 90]
        robs[5] = (observations[5] + 1) * 45.0
        # --> Rudder angle                   [-60 , 60 ]
        robs[6] = observations[6] * 60.0
        # --> Electric propulsion speed      [-5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5]
        robs[7] = observations[7] * 5.0
        # --> Roll angle                     [-180, 180]
        robs[8] = observations[8] * 180.0

    return robs

def manualControlUsingGymEnv():
    obsvarname = ["distance", "trajectory angle", "surge velocity", "aparent wind speed", "aparent wind angle",
                  "boom angle", "rudder angle", "eletric propultion speed", "roll angle"]

    env = gym.make('EboatStraightLineEnvCC29-v0')

    time.sleep(5)

    sensors = subprocess.Popen([sys.executable, os.path.join(
        "/home/araujo/eboat_ws/src/eboat_gz_1/eboat_control/python/projects/ESailor", "sensor_array.py")])

    obs, _ = env.reset()

    print("\n\n-------------------------------------\nRESET ENV CONCLUÍDO\n-------------------------------------\n")

    num_of_steps = 60
    acureward = 0
    reward = 0
    for step in range(num_of_steps):
        print("\n------------------------------------------")
        print(f"Step {step}")
        obs = observationRescale(obs)
        for i, value in enumerate(obs):
            print("{:s} : {:f}".format(obsvarname[i], value))
        print(f"Return earned in the step         : {reward}")
        print(f"Total return earned in the episode: {acureward}")

        input_actions = input("Enter an action in the format (boom angle , rudder angle):").split(" ")

        if len(input_actions) > 1:
            actions = np.array(input_actions, dtype=np.float32)
            actions[0] = (actions[0] / 45.0) - 1
            actions[1] = actions[1] / 60.0
        obs, reward, done, _, _ = env.step(actions)
        acureward += reward
        if done:
            print(f"SINAL DE TÉRMINO DO EPISÓDIO: {done}")
            print(f"Total return earned in the episode: {acureward}")
            break

    sensors.kill()
    env.close()

def manualControlExperiment(wind_speed = 7):
    port = "11311"
    port_gazebo = "11345"
    os.environ["ROS_MASTER_URI"] = "http://localhost:" + port
    os.environ["GAZEBO_MASTER_URI"] = "http://localhost:" + port_gazebo

    ros_path = os.path.dirname(subprocess.check_output(["which", "roscore"]))

    print(ros_path)

    obsvarname = ["distance", "trajectory angle", "surge velocity", "aparent wind speed", "aparent wind angle",
                  "boom angle", "rudder angle", "eletric propultion speed", "roll angle", "Pos.X", "Pos.Y"]

    EBOAT_HOME = "/home/araujo/yara_ws/src/Yara_OVE"
    launch_file_path = os.path.join(EBOAT_HOME, "eboat_gazebo/launch/ocean.launch")

    roslaunch = subprocess.Popen([sys.executable, os.path.join(ros_path, b"roslaunch"), "-p", port, launch_file_path])
    print("Gazebo launched!")

    gzclient_pid = 0
    rospy.init_node('model_testing', anonymous=True)

    boomAng_pub   = rospy.Publisher("/eboat/control_interface/sail", Float32, queue_size=5)
    rudderAng_pub = rospy.Publisher("/eboat/control_interface/rudder", Float32, queue_size=5)
    propVel_pub   = rospy.Publisher("/eboat/control_interface/propulsion", Int16, queue_size=5)
    wind_pub      = rospy.Publisher("/eboat/atmosferic_control/wind", Point, queue_size=5)
    unpause       = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    pause         = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
    reset_proxy   = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
    # set_state     = rospy.ServiceProxy('/gazebo/set_model_state' , SetModelState)
    spawn_model   = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    windList = [np.array([wind_speed, 0, 0]),
                np.concatenate((rot(wind_speed, 45), np.zeros(1)), dtype=np.float32),
                # np.concatenate((rot(wind_speed, -45), np.zeros(1)), dtype=np.float32),
                np.concatenate((rot(wind_speed, 90), np.zeros(1)), dtype=np.float32),
                # np.concatenate((rot(wind_speed, -90), np.zeros(1)), dtype=np.float32),
                # np.concatenate((rot(wind_speed, 135), np.zeros(1)), dtype=np.float32),
                np.concatenate((rot(wind_speed, -135), np.zeros(1)), dtype=np.float32)
               ]

    _ = getObservations()

    spawnModel(spawn_model, model_name="wayPointMarker", model_id="wayPointMarker", waypoint=[100, 0, 0])

    sensors = subprocess.Popen([sys.executable, os.path.join("/home/araujo/eboat_ws/src/eboat_gz_1/eboat_control/python/projects/ESailor", "sensor_array.py")])

    time.sleep(10)

    _ = getObservations()
    resetGazEnv(pause, unpause, reset_proxy, propVel_pub, boomAng_pub, rudderAng_pub)

    for windSpeed in windList:
        avg_speed  = 0.0
        wind_pub.publish(Point(windSpeed[0], windSpeed[1], windSpeed[2]))

        # --> GET OBSERVATIONS
        obs = getObservations()
        j = 0
        action = np.zeros(2, dtype=np.float32)
        while ((obs[0] > 5) & (j < 100)):
            # -->PAUSE SIMULATION
            rospy.wait_for_service("/gazebo/pause_physics")
            try:
                pause()
            except(rospy.ServiceException) as e:
                print(("/gazebo/pause_physics service call failed!"))

            for i, value in enumerate(obs):
                print("{:s} : {:f}".format(obsvarname[i], value))

            input_action = input("Enter an action in the format (boom angle , rudder angle):").split(" ")
            if len(input_action) > 1:
                action = np.array(input_action, dtype=np.float32)

            # -->SEND ACTION TO THE BOAT CONTROL INTERFACE
            boomAng_pub.publish(action[0])
            rudderAng_pub.publish(action[1])

            # --> UNPAUSE SIMULATION
            rospy.wait_for_service("/gazebo/unpause_physics")
            try:
                unpause()
            except(rospy.ServiceException) as e:
                print(("/gazebo/unpause_physics service call failed!"))

            obs = getObservations()
            j += 1
            #-------------
            avg_speed += obs[2]
            print(f"---------------------\nNÚMERO DE EPISÓDEOS TRANSCORRIDOS: {j}\n"
                  f"VELOCIDADE MÉDIA                 : {avg_speed/j}\n---------------------")

        wind_pub.publish(Point(0.0, 0.0, 0.0))
        resetGazEnv(pause, unpause, reset_proxy, propVel_pub, boomAng_pub, rudderAng_pub)
        # print(f"---------------------\nNÚMERO DE EPISÓDEOS TRANSCORRIDOS: {j}\n---------------------")

    # --> END SIMULATION
    sensors.kill()
    ppid = roslaunch.pid
    print(f"\n\n===================\nProcess id: {ppid}\n===================\n")
    os.system(f'ps -au araujo | grep {roslaunch.pid}')
    os.killpg(os.getpgid(roslaunch.pid), signal.SIGTERM)

    print("\n\n\nCLOSE FUNCTION\n\n")

if __name__ == "__main__":
    manualControlExperiment(wind_speed=6)