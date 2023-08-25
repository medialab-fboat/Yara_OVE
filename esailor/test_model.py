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

from stable_baselines3 import PPO, SAC

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
    L = len(observations)
    if L > 9:
        L = 9
    robs = np.zeros(L, dtype=np.float32)
    # --> Distance from the waypoint (m) [0   , DMAX];
    robs[0] = 2 * (observations[0] / DMAX) - 1
    # --> Trajectory angle               [-180, 180]
    robs[1] = observations[1] / 180.0
    # --> Boat linear velocity (m/s)     [-10 , 10 ]
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
    print(f"\n===========================\nspawnModel\n===========================\n")
    with open(
            f"/home/eduardo/USVSim/yara_ws/src/Yara_OVE/eboat_description/models/{model_name}/model.sdf") as f:
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

def setState(model_name, pose, theta):
    state = ModelState()
    state.model_name = model_name
    state.reference_frame = "world"
    # pose
    state.pose.position.x = pose[0]
    state.pose.position.y = pose[1]
    state.pose.position.z = pose[2]
    quaternion = quaternion_from_euler(0, 0, theta * np.pi / 180.0)
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

    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        result = set_state(state)
        assert result.success is True
    except rospy.ServiceException:
        print("/gazebo/get_model_state service call failed")

def singleWayPoint(model, wind_speed = 7):
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
                np.concatenate((rot(wind_speed, 135) ,np.zeros(1)), dtype=np.float32),
                np.concatenate((rot(wind_speed, -135),np.zeros(1)), dtype=np.float32)#,
                # np.concatenate((rot(wind_speed, 180) ,np.zeros(1)), dtype=np.float32)
                ]

    _ = getObservations()

    spawnModel(spawn_model, model_name="wayPointMarker", model_id="wayPointMarker", waypoint=[100, 0, 0])

    sensors = subprocess.Popen([sys.executable, os.path.join("/home/eduardo/USVSim/eboat_ws/src/eboat_gz_1/eboat_control/python/projects/ESailor", "sensor_array.py")])

    time.sleep(10)

    _ = getObservations()
    resetGazEnv(pause, unpause, reset_proxy, propVel_pub, boomAng_pub, rudderAng_pub)

    for windSpeed in windList:
        wind_pub.publish(Point(windSpeed[0], windSpeed[1], windSpeed[2]))

        #--> GET OBSERVATIONS
        obs = getObservations()
        j = 0
        while ((obs[0] > 5) & (j < 100)):
            ## --> ESAILOR <-- ##
            nobs = observationRescale(obs)
            action, _ = model.predict(nobs)
            boomAng_pub.publish((action[0] + 1) * 45.0)
            rudderAng_pub.publish(action[1] * 60.0)
            #####################
            obs = getObservations()
            j += 1

        wind_pub.publish(Point(0.0, 0.0, 0.0))
        resetGazEnv(pause, unpause, reset_proxy, propVel_pub, boomAng_pub, rudderAng_pub)

    # --> END SIMULATION
    sensors.kill()
    ppid = roslaunch.pid
    print(f"\n\n===================\nProcess id: {ppid}\n===================\n")
    os.system(f'ps -au eduardo | grep {roslaunch.pid}')
    os.killpg(os.getpgid(roslaunch.pid), signal.SIGTERM)

    print("\n\n\nCLOSE FUNCTION\n\n")

def multiWayPoints(model, wind_speed = [7, 0, 0]):
    waypointlist = []
    x = 0
    y = 0
    for i, theta in enumerate([90, 30, -30, -130, 180, -90]):
        rad = theta * (np.pi / 180.0)
        x += 100 * np.sin(rad)
        y += 100 * np.cos(rad)
        waypointlist.append([x, -y, 0])

    port = "11311"
    port_gazebo = "11345"
    os.environ["ROS_MASTER_URI"] = "http://localhost:" + port
    os.environ["GAZEBO_MASTER_URI"] = "http://localhost:" + port_gazebo

    ros_path = os.path.dirname(subprocess.check_output(["which", "roscore"]))

    print(ros_path)

    EBOAT_HOME = "/home/eduardo/USVSim/yara_ws/src/Yara_OVE"
    launch_file_path = os.path.join(EBOAT_HOME, "eboat_gazebo/launch/ocean.launch")

    roslaunch = subprocess.Popen([sys.executable, os.path.join(ros_path, b"roslaunch"), "-p", port, launch_file_path])
    print("Gazebo launched!")

    gzclient_pid = 0
    rospy.init_node('model_testing', anonymous=True)

    boomAng_pub = rospy.Publisher("/eboat/control_interface/sail", Float32, queue_size=5)
    rudderAng_pub = rospy.Publisher("/eboat/control_interface/rudder", Float32, queue_size=5)
    propVel_pub = rospy.Publisher("/eboat/control_interface/propulsion", Int16, queue_size=5)
    wind_pub = rospy.Publisher("/eboat/atmosferic_control/wind", Point, queue_size=5)
    unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
    reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
    # set_state     = rospy.ServiceProxy('/gazebo/set_model_state' , SetModelState)
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    _ = getObservations()

    # --> SPAW THE WAYPOINTS IN THE VIRTUAL WORLD
    for i, wp in enumerate(waypointlist):
        spawnModel(spawn_model, model_name="destinationMarker", model_id=f"wayPoint{i}", waypoint=wp)
        if i == 0:
            spawnModel(spawn_model, model_name="wayPointMarker", model_id="wayPointMarker", waypoint=wp)

    sensors     = subprocess.Popen([sys.executable, os.path.join("/home/eduardo/USVSim/eboat_ws/src/eboat_gz_1/eboat_control/python/projects/ESailor", "sensor_array.py")])
    camera_raw  = subprocess.Popen([sys.executable, "./camera_raw.py"])
    camera_proc = subprocess.Popen([sys.executable, "./camera_detection.py"])

    time.sleep(10)

    wind_pub.publish(Point(wind_speed[0], wind_speed[1], wind_speed[2]))

    _ = getObservations()
    resetGazEnv(pause, unpause, reset_proxy, propVel_pub, boomAng_pub, rudderAng_pub)

    i = 0
    L = len(waypointlist)
    while L > 0:
        if i > 0:
            setState("wayPointMarker", pose=waypointlist[0], theta=0)
            time.sleep(3)
        obs = getObservations()
        j   = 0
        while ((obs[0] > 5) & (j < 100)):
            ## --> ESAILOR <-- ##
            nobs      = observationRescale(obs)
            action, _ = model.predict(nobs)
            boomAng_pub.publish((action[0] + 1) * 45.0)
            rudderAng_pub.publish(action[1] * 60.0)
            #####################
            obs = getObservations()
            j += 1
        i += 1
        del(waypointlist[0])
        L = len(waypointlist)
        break

    # --> END SIMULATION
    sensors.kill()
    camera_raw.kill()
    camera_proc.kill()
    ppid = roslaunch.pid
    print(f"\n\n===================\nProcess id: {ppid}\n===================\n")
    os.system(f'ps -au eduardo | grep {roslaunch.pid}')
    os.killpg(os.getpgid(roslaunch.pid), signal.SIGTERM)

    print("\n\n\nCLOSE FUNCTION\n\n")

def main():
    # --> RL agent
    # model = PPO.load("/home/eduardo/USVSim/yara_ws/src/Yara_OVE/esailor/models/PPO/ensign29_7_winds_10m_straight_09082023_10_34_00/eboat_ocean_50")
    model = SAC.load("/home/eduardo/USVSim/yara_ws/src/Yara_OVE/esailor/models/SAC/ensign29_7_winds_10m_straight_10082023_10_05_47/eboat_ocean_50")

    # model = PPO.load("/home/eduardo/USVSim/yara_ws/src/Yara_OVE/esailor/models/PPO/ensign29_7_winds_5m_straight_17082023_10_23_35/eboat_ocean_50")
    # model = SAC.load("/home/eduardo/USVSim/yara_ws/src/Yara_OVE/esailor/models/SAC/ensign29_7_winds_5m_straight_18082023_00_22_05/eboat_ocean_50")

    # singleWayPoint(model, wind_speed = 7)
    # os.system('./kill_gaz.sh')
    multiWayPoints(model, wind_speed=[7, 0, 0])

if __name__ == "__main__":
    main()