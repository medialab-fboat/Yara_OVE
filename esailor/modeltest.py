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
from gazebo_msgs.srv import SetModelState, GetModelState, SpawnModel, DeleteModel, SetPhysicsProperties, GetPhysicsProperties
from geometry_msgs.msg import Point, Pose, Vector3
from gazebo_msgs.msg import ODEPhysics, ModelState
from tf.transformations import quaternion_from_euler

#-->STABLE-BASELINES3
# from gymnasium import wrappers
from stable_baselines3 import PPO, SAC, A2C
from stable_baselines3.common.callbacks import CheckpointCallback

#-->PYTORCH
import torch as th

import esailor

def main():
    es = esailor.esailor()
    home = os.path.expanduser('~')
    # path2launch = "/home/eduardo/USVSim/yara_ws/src/Yara_OVE/eboat_gazebo/launch/ocean_fixed_cam.launch"
    path2launch = "/home/eduardo/USVSim/yara_ws/src/Yara_OVE/eboat_gazebo/launch/ocean.launch"
    es.launchGazeboSimulation(path2launch)
    # -->EXPORT ADDRESS FOR THE ROS MASTER NODE AND THE GAZEBO SERVER
    # os.environ["ROS_MASTER_URI"] = "http://localhost:" + es.port_ros
    # os.environ["GAZEBO_MASTER_URI"] = "http://localhost:" + es.port_gazebo

    # agent = PPO.load("./models/PPO/esailor_92_cr_A12123_C12123_27092023_22_33_51/esailor_model_1001472_steps.zip")
    # agent = PPO.load("./models/PPO/esailor_a_cr_A12123_C12123_26092023_22_41_36/esailor_model_1001472_steps.zip")
    agent = PPO.load("./models/PPO/esailor_8d_winds_19092023_22_14_05/esailor_model_1001472_steps.zip")

    # --> WAIT FOR PHYSICS
    rospy.wait_for_service('/gazebo/get_physics_properties')

    # -->ROS TOPICS AND SERVICES
    rospy.init_node('esailor', anonymous=True)
    boomAng_pub   = rospy.Publisher(f"/eboat/control_interface/sail"      , Float32, queue_size=1)
    rudderAng_pub = rospy.Publisher(f"/eboat/control_interface/rudder"    , Float32, queue_size=1)
    propVel_pub   = rospy.Publisher(f"/eboat/control_interface/propulsion", Int16  , queue_size=1)
    wind_pub      = rospy.Publisher(f"/eboat/atmosferic_control/wind"     , Point  , queue_size=1)
    delmodel      = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)

    # -->SERACH FOR THE SDF FILE DESCRIBING THE WAYPOINT
    files = glob.glob(os.path.join(home, f"**/*Yara_OVE/**/*wayPointMarker/model.sdf"), recursive=True)
    if len(files) > 0:
        sdffilepath = files[0]
        del (files)
    else:
        raise IOError(f"File wayPointMarker/model.sdf does not exist")

    #-->PATH PLANNING FOR THE TEST MISSION
    baseDist    = 100.0
    baseVec     = np.array([1.0, 0.0])
    path2follow = [] #[baseVec * baseDist]
    thetaVec    = [-30     , 45      , 90      , 140     ]
    D           = [baseDist, baseDist, baseDist, baseDist]
    for i, theta in enumerate(thetaVec):
        rad = theta * np.pi / 180.0
        rot = np.array([[np.cos(rad), -np.sin(rad)], [np.sin(rad), np.cos(rad)]])
        if i == 0:
            path2follow.append(np.array([0.0, 0.0]) + np.dot((baseVec * D[i]), rot))
        path2follow.append(path2follow[-1] + np.dot((baseVec * D[i]), rot))

    model_pose = Pose()
    quaternion = quaternion_from_euler(0, 0, 0)
    model_pose.position.x = path2follow[0][0]
    model_pose.position.y = path2follow[0][1]
    model_pose.position.z = 0.0
    model_pose.orientation.x = quaternion[0]
    model_pose.orientation.y = quaternion[1]
    model_pose.orientation.z = quaternion[2]
    model_pose.orientation.w = quaternion[3]
    es.spawnSDFModel("wayPointMarker", sdffilepath, model_pose)
    time.sleep(5)

    # --> SET THE WIND
    wind_pub.publish(Point(6.17, 0.0, 0.0))
    # wind_pub.publish(Point(-4.243, -4.234, 0.0))

    rospy.wait_for_service("/gazebo/delete_model")
    _ = delmodel("wayPointMarker")

    # --> INITIALIZE THE SENSOR HUD
    sensors = subprocess.Popen([sys.executable, os.path.join("/home/eduardo/USVSim/eboat_ws/src/eboat_gz_1/eboat_control/python/projects/ESailor", "sensor_array.py")])
    # camera_raw = subprocess.Popen([sys.executable, "./camera_raw.py"])

    for i, waypoint in enumerate(path2follow):
        if i > 0:
            DMAX = D[i - 1] + 25
        else:
            DMAX = baseDist + 25

        model_pose.position.x = waypoint[0]
        model_pose.position.y = waypoint[1]
        es.spawnSDFModel("wayPointMarker", sdffilepath, model_pose)

        print(f"Going to waypoint {i}: {waypoint}")

        duration = 0
        mission = True
        actionVec = []
        es.DMAX   = DMAX
        while (mission & (duration < 180)):
            obsData = None
            while (obsData is None):
                try:
                    obsData = rospy.wait_for_message(f"/eboat/mission_control/observations", Float32MultiArray,
                                                     timeout=20).data
                    obs = np.array(obsData)
                except:
                    pass

            action, _ = agent.predict(es.rescaleObs(obs))
            actionVec.append([((action[0] + 1) * 45.0), (action[1] * 60.0)])
            boomAng_pub.publish(actionVec[-1][0])
            rudderAng_pub.publish(actionVec[-1][1])
            # print(f"{duration} - Action: {actionVec[-1][0]} | {actionVec[-1][1]}")

            if obs[0] <= 5:
                mission = False
            else:
                duration += 1

        # if i > 0:
        #     break
        rospy.wait_for_service("/gazebo/delete_model")
        _ = delmodel("wayPointMarker")
        time.sleep(1)

    _ = input("Pressione qualquer tecla para concluir")

    sensors.kill()
    # camera_raw.kill()
    es.close()

if __name__ == "__main__":
    main()