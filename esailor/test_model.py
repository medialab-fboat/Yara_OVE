#!/home/eduardo/miniconda3/envs/esailor2/bin/python

import numpy as np
import rospy
import rosnode
import rosgraph
import argparse
import sys, os, signal
import subprocess
import time
import random

from std_srvs.srv import Empty

from std_msgs.msg import Float32, Int16, Float32MultiArray
from geometry_msgs.msg import Point

from tf.transformations import quaternion_from_euler
from gazebo_msgs.srv import SetModelState, SpawnModel, DeleteModel
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose

from stable_baselines3 import PPO

import matplotlib.pyplot as plt

DMAX = 125

def rot(modulus, angle):
    theta = angle * np.pi / 180.0
    R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]], dtype=float)

    return np.dot(np.array([1, 0], dtype=float) * modulus, R)

# def actionRescale(action):
#     raction = np.zeros(3, dtype=np.float32)
#     # --> Eletric propulsion [-5, 5]
#     raction[0] = action[0] * 5.0
#     # --> Boom angle [0, 90]
#     raction[1] = (action[1] + 1) * 45.0
#     # --> Rudder angle [-60, 60]
#     raction[2] = action[2] * 60.0
#     return raction

def actionRescale(action):
    raction = np.zeros(2, dtype=np.float32)
    # --> Boom angle [0, 90]
    raction[0] = (action[0] + 1) * 45.0
    # --> Rudder angle [-60, 60]
    raction[1] = action[1] * 60.0
    return raction

def observationRescale(observations):
    robs = np.zeros(len(observations), dtype=np.float32)
    # --> Distance from the waypoint (m) [0   , DMAX];
    robs[0] = 2 * (observations[0] / DMAX) - 1
    # --> Trajectory angle               [-180, 180]
    robs[1] = observations[1] / 180.0
    # --> Boat linear velocity (m/s)     [0   , 10 ]
    robs[2] = observations[2] / 5 - 1
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

def getObservations():
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

def resetGazEnv(pause, unpause, reset_proxy, propVel_pub, boomAng_pub, rudderAng_pub):
    # -->PAUSE SIMULATION
    rospy.wait_for_service("/gazebo/pause_physics")
    try:
        pause()
    except(rospy.ServiceException) as e:
        print(("/gazebo/pause_physics service call failed!"))

    # -->RESETS THE STATE OF THE ENVIRONMENT.
    rospy.wait_for_service('/gazebo/reset_simulation')
    try:
        reset_proxy()
    except (rospy.ServiceException) as e:
        print(("/gazebo/reset_simulation service call failed!"))

    # -->SET THE ACTUATORS BACK TO THE DEFAULT SETTING
    propVel_pub.publish(0)
    boomAng_pub.publish(0.0)
    rudderAng_pub.publish(0.0)

    # -->UNPAUSE SIMULATION TO MAKE OBSERVATION
    rospy.wait_for_service('/gazebo/unpause_physics')
    try:
        unpause()
    except(rospy.ServiceException) as e:
        print(("/gazebo/unpause_physics service call failed!"))

def closeGazEnv(port, launch_process):
    nodelist = ['/gazebo_gui', '/gazebo', '/rosout']

    try:
        from xmlrpc.client import ServerProxy
    except ImportError:
        from xmlrpclib import ServerProxy

    ID     = "/rosnode"
    master = rosgraph.Master(ID, master_uri=f"http://localhost:{port}")
    print(master.getUri())
    for node in nodelist:
        print("  " + node)
        node_api = rosnode.get_api_uri(master, node)
        if not node_api:
            print("    API URI: error (unknown node: {}?)".format(node))
            continue
        print("    API URI: " + node_api)
        node = ServerProxy(node_api)
        pid = rosnode._succeed(node.getPid(ID))
        print("    PID    : {}".format(pid))
        os.kill(pid, signal.SIGKILL)

    launch_process.terminate()
    return 0

def spawnModel(rosserviseproxy, model_name="wayPointMarker", model_id="wayPointMarker", waypoint=[0, 100, 0]):
    ipose = Pose()
    ipose.position.x = waypoint[0]
    ipose.position.y = waypoint[1]
    ipose.position.z = waypoint[2]
    print(f"\n\n===========================\nspawnModel\n===========================\n")
    with open(
            f"/home/eduardo/USVSim/eboat_ws/src/eboat_gz_1/eboat_description/models/{model_name}/model.sdf") as f:
        sdffile = f.read()
        try:
            result = rosserviseproxy(f"{model_id}",
                                 sdffile,
                                 f"{model_id}",
                                 ipose, "world")
            return 0
        except rospy.ServiceException:
            print("/gazebo/SpawnModel service call failed")
            return 1
        print(f"\n\n===========================\n{result}\n===========================\n")

def singleWayPoint(model, wind_speed = 10):
    port        = "11311"
    port_gazebo = "11345"
    os.environ["ROS_MASTER_URI"]    = "http://localhost:"+port
    os.environ["GAZEBO_MASTER_URI"] = "http://localhost:"+port_gazebo

    ros_path = os.path.dirname(subprocess.check_output(["which", "roscore"]))

    print(ros_path)

    EBOAT_HOME       = "/home/eduardo/USVSim/yara_ws/src/Yara_OVE"
    launch_file_path = os.path.join(EBOAT_HOME, "eboat_gazebo/launch/ocean.launch")

    roslaunch = subprocess.Popen([sys.executable, os.path.join(ros_path, b"roslaunch"), "-p", port, launch_file_path])
    print ("Gazebo launched!")

    gzclient_pid = 0
    rospy.init_node('model_testing', anonymous=True)

    boomAng_pub   = rospy.Publisher("/eboat/control_interface/sail"      , Float32, queue_size=5)
    rudderAng_pub = rospy.Publisher("/eboat/control_interface/rudder"    , Float32, queue_size=5)
    propVel_pub   = rospy.Publisher("/eboat/control_interface/propulsion", Int16  , queue_size=5)
    wind_pub      = rospy.Publisher("/eboat/atmosferic_control/wind"     , Point  , queue_size=5)
    unpause       = rospy.ServiceProxy('/gazebo/unpause_physics' , Empty)
    pause         = rospy.ServiceProxy('/gazebo/pause_physics'   , Empty)
    reset_proxy   = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
    # set_state     = rospy.ServiceProxy('/gazebo/set_model_state' , SetModelState)
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    windList = [np.array([wind_speed, 0, 0]),
                np.concatenate((rot(wind_speed, 45)  ,np.zeros(1)), dtype=np.float32),
                np.concatenate((rot(wind_speed, -45) ,np.zeros(1)), dtype=np.float32),
                np.concatenate((rot(wind_speed, 90)  ,np.zeros(1)), dtype=np.float32),
                np.concatenate((rot(wind_speed, -90) ,np.zeros(1)), dtype=np.float32),
                np.concatenate((rot(wind_speed, 235) ,np.zeros(1)), dtype=np.float32),
                np.concatenate((rot(wind_speed, -235),np.zeros(1)), dtype=np.float32),
                np.concatenate((rot(wind_speed, 180) ,np.zeros(1)), dtype=np.float32)
                ]

    _ = getObservations()

    spawnModel(spawn_model, model_name="wayPointMarker", model_id="wayPointMarker", waypoint=[100, 0, 0])
    time.sleep(10)

    _ = getObservations()
    resetGazEnv(pause, unpause, reset_proxy, propVel_pub, boomAng_pub, rudderAng_pub)

    for windSpeed in windList[:1]:
        wind_pub.publish(Point(windSpeed[0], windSpeed[1], windSpeed[2]))

        #--> GET OBSERVATIONS
        obs = getObservations()
        distance, boat_orientation, linear_velocity, aparent_wind_speed,\
            aparent_wind_angle, boom_angle, rudder_angle, propeller_speed, roll_angle = obs

        count = 0

        while distance > 5:
            # --> ESAILOR
            robs           = observationRescale(obs)[:5]
            action, _state = model.predict(robs)

            #--> SEND ACTION TO THE BOAT CONTROL INTERFACE
            # action = np.array([0.0,0.0,40.0])
            ract = actionRescale(action)
            # propVel_pub.publish(int(ract[0]))
            boomAng_pub.publish(ract[0])
            rudderAng_pub.publish(ract[1])

            #--> GET OBSERVATIONS
            obs  = getObservations()
            distance, boat_orientation, linear_velocity, aparent_wind_speed, \
                aparent_wind_angle, boom_angle, rudder_angle, propeller_speed, roll_angle = obs

            if count > 150:
                break
            else:
                count += 1

        resetGazEnv(pause, unpause, reset_proxy, propVel_pub, boomAng_pub, rudderAng_pub)

    # closeGazEnv()

def main():
    # --> RL agent
    model = PPO.load("/home/eduardo/USVSim/yara_ws/src/Yara_OVE/esailor/models/PPO/model25v1_05072023_22_52_16/eboat_ocean_50")

    singleWayPoint(model, wind_speed = 10)

    os.system('./kill_gaz.sh')

if __name__ == "__main__":
    main()