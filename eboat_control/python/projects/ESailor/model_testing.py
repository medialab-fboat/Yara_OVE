#!/home/araujo/miniconda3/envs/esailor/bin/python

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

def actionRescale(action):
    raction = np.zeros(3, dtype=np.float32)
    # --> Eletric propulsion [-5, 5]
    raction[0] = action[0] * 5.0
    # --> Boom angle [0, 90]
    raction[1] = (action[1] + 1) * 45.0
    # --> Rudder angle [-60, 60]
    raction[2] = action[2] * 60.0
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

def setState(model_name, pose, theta, set_state):
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

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        result = set_state(state)
        assert result.success is True
    except rospy.ServiceException:
        print("/gazebo/get_model_state service call failed")

def spawnModel(rosserviseproxy, model_name="wayPointMarker", model_id="wayPointMarker", waypoint=[0, 100, 0]):
    ipose = Pose()
    ipose.position.x = waypoint[0]
    ipose.position.y = waypoint[1]
    ipose.position.z = waypoint[2]
    with open(
            f"/home/araujo/yara_ws/src/Yara_OVE/eboat_description/models/{model_name}/model.sdf") as f:
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

def closeGazEnv():
    # --> CLOSE
    tmp = os.popen("ps -Af").read()
    gzclient_count = tmp.count('gzclient')
    gzserver_count = tmp.count('gzserver')
    roscore_count = tmp.count('roscore')
    rosmaster_count = tmp.count('rosmaster')

    if gzclient_count > 0:
        os.system("killall -9 gzclient")
    if gzserver_count > 0:
        os.system("killall -9 gzserver")
    if rosmaster_count > 0:
        os.system("killall -9 rosmaster")
    if roscore_count > 0:
        os.system("killall -9 roscore")

    if (gzclient_count or gzserver_count or roscore_count or rosmaster_count > 0):
        os.wait()

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

def singleWayPoint(model, wind_speed = 10):
    port        = "11311"
    port_gazebo = "11345"
    os.environ["ROS_MASTER_URI"]    = "http://localhost:"+port
    os.environ["GAZEBO_MASTER_URI"] = "http://localhost:"+port_gazebo

    ros_path = os.path.dirname(subprocess.check_output(["which", "roscore"]))

    print(ros_path)

    EBOAT_HOME       = "/home/araujo/yara_ws/src/Yara_OVE"
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

    for windSpeed in windList:
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
            propVel_pub.publish(int(ract[0]))
            boomAng_pub.publish(ract[1])
            rudderAng_pub.publish(ract[2])

            #--> GET OBSERVATIONS
            obs  = getObservations()
            distance, boat_orientation, linear_velocity, aparent_wind_speed, \
                aparent_wind_angle, boom_angle, rudder_angle, propeller_speed, roll_angle = obs

            if count > 150:
                break
            else:
                count += 1

        resetGazEnv(pause, unpause, reset_proxy, propVel_pub, boomAng_pub, rudderAng_pub)

    closeGazEnv()

def regattaCourse(model, wind_speed = 10, wind_angle = 0, obs_space_size = 5):
    port        = "11311"
    port_gazebo = "11345"
    os.environ["ROS_MASTER_URI"]    = "http://localhost:"+port
    os.environ["GAZEBO_MASTER_URI"] = "http://localhost:"+port_gazebo

    ros_path = os.path.dirname(subprocess.check_output(["which", "roscore"]))

    print(ros_path)

    EBOAT_HOME       = "/home/araujo/yara_ws/src/Yara_OVE"
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
    # reset_proxy   = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
    set_state     = rospy.ServiceProxy('/gazebo/set_model_state' , SetModelState)
    spawn_model   = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    dist = 100
    navpoint0 = np.concatenate((rot(dist, 45)  ,np.zeros(1)), dtype=np.float32)
    navpoint1 = navpoint0 + np.concatenate((rot(dist, -45)  ,np.zeros(1)), dtype=np.float32)
    navpoint2 = navpoint1 + np.concatenate((rot(dist, -135)  ,np.zeros(1)), dtype=np.float32)
    navpoint3 = np.array([0, 0, 0])
    navpoints = [navpoint0,
                 navpoint1,
                 navpoint2,
                 navpoint3,
                 navpoint1,
                 navpoint3
                ]

    _ = getObservations()

    #########################################################################
    for i, waypoint in enumerate(navpoints[:4]):
        ipose = Pose()
        ipose.position.x = waypoint[0]
        ipose.position.y = waypoint[1]
        ipose.position.z = waypoint[2]
        with open(
                "/home/araujo/yara_ws/src/Yara_OVE/eboat_description/models/wayPointMarker/model.sdf") as f:
            sdffile = f.read()
            try:
                result = spawn_model(f"wayPointMarker{i}",
                                     sdffile,
                                     f"wayPointMarker{i}",
                                     ipose, "world")
            except rospy.ServiceException:
                print("/gazebo/SpawnModel service call failed")
    #########################################################################

    windSpeed = np.concatenate((rot(wind_speed, wind_angle), np.zeros(1)), dtype=np.float32)
    wind_pub.publish(Point(windSpeed[0], windSpeed[1], windSpeed[2]))

    sailingPoints = ["B", "C", "D", "A", "C", "A"]

    for i, waypoint in enumerate(navpoints):
        print(f"Current Way Point: {sailingPoints[i]}")
        
        #--> PAUSE SIMULATION
        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            pause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/pause_physics service call failed!"))

        setState("wayPointMarker", pose=waypoint, theta=0, set_state=set_state)

        #--> UNPAUSE SIMULATION
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            unpause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/unpause_physics service call failed!"))

        #--> GET OBSERVATIONS
        obs = getObservations()
        distance, boat_orientation, linear_velocity, aparent_wind_speed,\
            aparent_wind_angle, boom_angle, rudder_angle, propeller_speed, roll_angle = obs

        count = 0

        while distance > 5:
            # --> ESAILOR
            robs           = observationRescale(obs)[:obs_space_size]
            action, _state = model.predict(robs)

            #--> SEND ACTION TO THE BOAT CONTROL INTERFACE
            # action = np.array([0.0,0.0,40.0])
            ract = actionRescale(action)
            propVel_pub.publish(int(ract[0]))
            boomAng_pub.publish(ract[1])
            rudderAng_pub.publish(ract[2])

            #--> GET OBSERVATIONS
            obs  = getObservations()
            distance, boat_orientation, linear_velocity, aparent_wind_speed, \
                aparent_wind_angle, boom_angle, rudder_angle, propeller_speed, roll_angle = obs

            if count > 150:
                break
            else:
                count += 1

    closeGazEnv()

def pathTest(model, wind_speed = 10, obs_space_size = 5, dist = 100):
    random_number = random.randint(10000, 15000)
    port        = f"{random_number}"
    port_gazebo = f"{random_number + 1}"
    os.environ["ROS_MASTER_URI"]    = f"http://localhost:{port}"
    os.environ["GAZEBO_MASTER_URI"] = f"http://localhost:{port_gazebo}"

    ros_path = os.path.dirname(subprocess.check_output(["which", "roscore"]))

    print(ros_path)

    EBOAT_HOME       = "/home/araujo/yara_ws/src/Yara_OVE/"
    launch_file_path = os.path.join(EBOAT_HOME, "eboat_gazebo/launch/ocean_RL_training.launch")

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
    set_state     = rospy.ServiceProxy('/gazebo/set_model_state' , SetModelState)
    spawn_model   = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    navpoints = [np.concatenate((rot(dist, -45)  ,np.zeros(1)), dtype=np.float32)]
    navpoints.append(navpoints[-1] + np.concatenate((rot(dist, 45), np.zeros(1)), dtype=np.float32))
    navpoints.append(navpoints[-1] + np.concatenate((rot(dist, 135), np.zeros(1)), dtype=np.float32))
    navpoints.append(navpoints[-1] + np.concatenate((rot(dist, 90), np.zeros(1)), dtype=np.float32))
    navpoints.append(navpoints[-1] + np.concatenate((rot(dist, -135), np.zeros(1)), dtype=np.float32))
    navpoints.append(navpoints[-1] + np.concatenate((rot(dist, 180), np.zeros(1)), dtype=np.float32))
    navpoints.append(navpoints[-1] + np.concatenate((rot(dist, -90), np.zeros(1)), dtype=np.float32))

    _ = getObservations()

    wind_pub.publish(Point(wind_speed, 0, 0))

    sensor_array = subprocess.Popen([sys.executable, "./sensor_array.py"])

    for i, waypoint in enumerate(navpoints):
        _ = spawnModel(rosserviseproxy=spawn_model, model_name="wayPointMarker", model_id=f"wayPointMarker{i}", waypoint=waypoint)

    _ = spawnModel(rosserviseproxy=spawn_model, model_name="destinationMarker", model_id="wayPointMarker", waypoint=navpoints[0])

    for i, waypoint in enumerate(navpoints):
        print(f"Sailing to waypoint {i}")

        # --> PAUSE SIMULATION
        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            pause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/pause_physics service call failed!"))

        setState("wayPointMarker", pose=waypoint, theta=0, set_state=set_state)

        # --> UNPAUSE SIMULATION
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            unpause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/unpause_physics service call failed!"))

        # --> GET OBSERVATIONS
        obs = getObservations()

        count = 0

        while obs[0] > 5:
            # --> ESAILOR
            robs = observationRescale(obs)[:obs_space_size]
            action, _state = model.predict(robs)

            # --> SEND ACTION TO THE BOAT CONTROL INTERFACE
            ract = actionRescale(action)
            propVel_pub.publish(int(ract[0]))
            boomAng_pub.publish(ract[1])
            rudderAng_pub.publish(ract[2])

            # --> GET OBSERVATIONS
            obs = getObservations()

            if count > 150:
                break
            else:
                count += 1                
        

    sensor_array.kill()
    closeGazEnv(port, roslaunch)

def main():
    # --> RL agent
    #model = PPO.load("/home/araujo/eboat_ws/src/eboat_gz_1/models/PPO/ensign_five/eboat_ocean_50") #--> model0_11042023_09_24_34
    # model = PPO.load("/home/araujo/eboat_ws/src/eboat_gz_1/models/PPO/ensign_nine/eboat_ocean_50") #--> model1_13042023_10_15_19
    # model = PPO.load("/home/araujo/eboat_ws/src/eboat_gz_1/models/PPO/model1_17042023_10_52_09/eboat_ocean_50")
    # model = PPO.load("/home/araujo/eboat_ws/src/eboat_gz_1/models/PPO/model1_19042023_22_10_20/eboat_ocean_50")
    # model = PPO.load("/home/araujo/eboat_ws/src/eboat_gz_1/models/PPO/model1_20042023_21_36_39/eboat_ocean_50")
    model = PPO.load("/home/araujo/yara_ws/src/Yara_OVE/esailor/models/PPO/model35_25082023_23_29_13/eboat_ocean_50.zip")

    # singleWayPoint(model, wind_speed = 10)
    # regattaCourse(model, wind_speed=10, wind_angle=0, obs_space_size = 9)
    pathTest(model, wind_speed=10, obs_space_size=5)

if __name__ == "__main__":
    main()