#!/home/eduardo/miniconda3/envs/esailor/bin/python

import numpy as np
import pandas as pd
import rospy
import sys
import os
import subprocess
import time

from matplotlib import pyplot as plt

from std_srvs.srv import Empty
from std_msgs.msg import Float32, Int16, Float32MultiArray
from geometry_msgs.msg import Point

from gazebo_msgs.srv import SetModelState, SpawnModel
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose

from tf.transformations import quaternion_from_euler

from stable_baselines3 import PPO

DMAX = 125

def getObservations():
    # --> GET OBSERVATIONS
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

def truncate(value):
    ival = int(value)
    if (abs(value - ival) < 0.5):
        return ival
    elif value < 0:
        return (ival - 1)
    else:
        return (ival + 1)

def rewardFunction(obs, DPREV, DMAX = 125):
    reward = (DPREV - obs[0]) / DMAX

    if reward > 0:
        reward *= (1.0 - 0.9 * truncate(obs[7]) / 5.0)
    else:
        reward -= 0.01 * truncate(obs[7])

    # --> obsData = [distance, trajectory angle, linear velocity, aparent wind speed, aparent wind angle, boom angle, rudder angle, eletric propultion speed, roll angle]
    #               [   0    ,        1        ,       2        ,         3         ,         4         ,     5     ,      6      ,            7            ,     8     ]
    return reward

def acceleration(propVel_pub, reset_proxy, dt = 20):
    data = []
    for s in range(1, 6):
        obs = getObservations()
        data.append(np.array([s, 0, obs[2], obs[9], obs[10]]))
        propVel_pub.publish(s)
        for t in range(dt):
            obs = getObservations()
            data.append(np.array([s, t + 1, obs[2], obs[9], obs[10]]))
        propVel_pub.publish(0)
        for t in range(dt):
            obs = getObservations()
            data.append(np.array([s, t + dt + 1, obs[2], obs[9], obs[10]]))

        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            reset_proxy()
        except (rospy.ServiceException) as e:
            print("/gazebo/reset_simulation service call failed!")

    df = pd.DataFrame(data, columns=["prop", "time", "vel", "X", "Y"])
    df.to_csv("./acceleration.csv", sep=";", index = False)

def movingTurn(propVel_pub, rudderAng_pub, reset_proxy, dt=20):
    data = []
    for C in [1, -1]:
        for ra in [15, 30, 45, 60]:
            for s in range(1, 6):
                obs = getObservations()
                data.append(np.array([C, s, ra, 0, obs[2], obs[9], obs[10]]))
                propVel_pub.publish(s)
                for t in range(dt):
                    obs = getObservations()
                    data.append(np.array([C, s, ra, t + 1, obs[2], obs[9], obs[10]]))
                rudderAng_pub.publish(ra*C)
                for t in range(15):
                    obs = getObservations()
                    data.append(np.array([C, s, ra, t + dt + 1, obs[2], obs[9], obs[10]]))

                propVel_pub.publish(0)
                rudderAng_pub.publish(0.0)
                _ = getObservations()

                rospy.wait_for_service('/gazebo/reset_simulation')
                try:
                    reset_proxy()
                except (rospy.ServiceException) as e:
                    print("/gazebo/reset_simulation service call failed!")

    df = pd.DataFrame(data, columns=["sign", "prop", "rangle", "time", "vel", "X", "Y"])
    df.to_csv("./movingTurn.csv", sep=";", index=False)

def turningFromRest(propVel_pub, rudderAng_pub, reset_proxy, dt=15):
    data = []
    for C in [1, -1]:
        rudderAng_pub.publish(C*60)
        for s in range(1, 6):
            obs = getObservations()
            data.append(np.array([C, s, 0, obs[2], obs[9], obs[10]]))
            propVel_pub.publish(s)
            for t in range(dt):
                obs = getObservations()
                data.append(np.array([C, s, t + 1, obs[2], obs[9], obs[10]]))
            propVel_pub.publish(0)

            rospy.wait_for_service('/gazebo/reset_simulation')
            try:
                reset_proxy()
            except (rospy.ServiceException) as e:
                print("/gazebo/reset_simulation service call failed!")

    df = pd.DataFrame(data, columns=["sign", "prop", "time", "vel", "X", "Y"])
    df.to_csv("./turningFromRest.csv", sep=";", index=False)

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

def rot(modulus, angle):
    theta = angle * np.pi / 180.0
    R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]], dtype=float)

    return np.dot(np.array([1, 0], dtype=float) * modulus, R)

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

def regatta(pause, unpause, boomAng_pub, rudderAng_pub, propVel_pub, wind_pub, sep_dist = 100, wind_speed = 10, wind_angle = 0):
    dmax = 1.25 * sep_dist
    navpoints = [np.concatenate((rot(sep_dist, 45), np.zeros(1)), dtype=np.float32)]
    navpoints.append(navpoints[0] + np.concatenate((rot(sep_dist, -45), np.zeros(1)), dtype=np.float32))
    navpoints.append(navpoints[1] + np.concatenate((rot(sep_dist, -135), np.zeros(1)), dtype=np.float32))
    navpoints.append(np.array([0, 0, 0]))

    #--> CREATE FIGURE
    fig = plt.figure()
    ax  = fig.add_subplot(111)
    ax.set_xlabel("X coordinate in meters", fontsize=14)
    ax.set_ylabel("Y coordinate in meters", fontsize=14)
    navcolors = [ "green", "red", "blue", "orange"]
    navids    = ["navpoint B", "navpoint C", "navpoint D", "navpoint A"]

    #--> CREATE WAYPOINTS FOR VISUALIZATION
    path2follow = [0, 1, 2, 3, 1, 3]
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    for i in range(len(navpoints)):
        waypoint = navpoints[i]
        ipose = Pose()
        ipose.position.x = waypoint[0]
        ipose.position.y = waypoint[1]
        ipose.position.z = waypoint[2]
        with open(
                "/home/eduardo/USVSim/eboat_ws/src/eboat_gz_1/eboat_description/models/wayPointMarker/model.sdf") as f:
            sdffile = f.read()
            try:
                result = spawn_model(f"wayPointMarker{i}",
                                     sdffile,
                                     f"wayPointMarker{i}",
                                     ipose, "world")
            except rospy.ServiceException:
                print("/gazebo/SpawnModel service call failed")
        print(waypoint)
        ax.plot(waypoint[0], waypoint[1], marker='o', markersize=12, color = navcolors[i])

    windSpeed = np.concatenate((rot(wind_speed, wind_angle), np.zeros(1)), dtype=np.float32)
    wind_pub.publish(Point(windSpeed[0], windSpeed[1], windSpeed[2]))

    # --> RL agent
    model = PPO.load("/home/eduardo/USVSim/eboat_ws/src/eboat_gz_1/models/PPO/ensign_five/eboat_ocean_50")
    # model = PPO.load("/home/eduardo/USVSim/eboat_ws/src/eboat_gz_1/models/PPO/model1_13042023_10_15_19/eboat_ocean_50")
    # model = PPO.load("/home/eduardo/USVSim/eboat_ws/src/eboat_gz_1/models/PPO/model1_17042023_10_52_09/eboat_ocean_50")

    #--> TO STORE OBSERVATIONSL DATA TO SAVE
    obslist = []

    #--> INITIALIZE AUXILIARY SUBPROCESS
    os.system("rm ./flag_file.txt")
    os.system("rm ./aux_data*.csv")
    # subprocess.Popen(["./getPositions.py"])

    for i in path2follow:
        waypoint = navpoints[i]
        print(f"Traveling to {navids[i]}")

        #--> PAUSE SIMULATION
        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            pause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/pause_physics service call failed!"))

        setState("wayPointMarker", pose=waypoint, theta=0)

        #--> UNPAUSE SIMULATION
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            unpause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/unpause_physics service call failed!"))

        #--> GET OBSERVATIONS
        obsData = None
        while obsData is None:
            try:
                obsData = rospy.wait_for_message('/eboat/mission_control/observations', Float32MultiArray,
                                                 timeout=20).data
            except:
                pass
            # --> obsData = [distance, trajectory angle, linear velocity, aparent wind speed, aparent wind angle, boom angle, rudder angle, eletric propultion speed, roll angle]
            #               [   0    ,        1        ,       2        ,         3         ,         4         ,     5     ,      6      ,            7            ,     8     ]

        obs = np.array(obsData, dtype=float)

        while obs[0] > 5:
            # print(f"Distance: {obs[0]}")
            # --> ESAILOR
            robs = observationRescale(obs)[:5]
            action, _state = model.predict(robs)
            obslist.append(obs)

            # --> SEND ACTION TO THE BOAT CONTROL INTERFACE
            # action = np.array([0.0,0.0,40.0])
            ract = actionRescale(action)
            propVel_pub.publish(int(ract[0]))
            boomAng_pub.publish(ract[1])
            rudderAng_pub.publish(ract[2])

            # --> GET OBSERVATIONS
            obsData = None
            while obsData is None:
                try:
                    obsData = rospy.wait_for_message('/eboat/mission_control/observations', Float32MultiArray,
                                                     timeout=20).data
                except:
                    pass
            obs = np.array(obsData, dtype=float)

    # -->PAUSE SIMULATION
    rospy.wait_for_service("/gazebo/pause_physics")
    try:
        pause()
    except(rospy.ServiceException) as e:
        print(("/gazebo/pause_physics service call failed!"))

    # count = 0
    # while (not(os.path.exists("./flag_file.txt"))) & (count < 5):
    #     time.sleep(5)
    #     count += 1
    # if count < 10:
    #     data = pd.read_csv("./aux_data_pos.csv", sep=";")
    #     ax.scatter(data.X, data.Y)
    #     plt.show()
    # else:
    #     print("ALGO DEU ERRADO!")
    #     os.system(("killall -9 getPositions.py"))

    df = pd.DataFrame(obslist, columns=["dist", "traj_ang", "surge_vel", "apwind_speed", "apwind_ang", "boom_angle", "rudder_angle", "prop_speed", "roll_angle","X","Y"])
    df.to_csv("./aux_data_obs.csv", sep=";", index = False)

def autoMission(pause, unpause, boomAng_pub, rudderAng_pub, propVel_pub, wind_pub, reset_proxy, navpoints, navids, path2follow, sep_dist = 100, wind_speed = 10, wind_angle = 0):
    dmax = 1.25 * sep_dist

    #--> WAYPOINT FOR GUIDANCE
    ipose            = Pose()
    ipose.position.x = 0
    ipose.position.y = 0
    ipose.position.z = 0
    spawn_model      = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    with open(
            "/home/eduardo/USVSim/eboat_ws/src/eboat_gz_1/eboat_description/models/wayPointMarker/model.sdf") as f:
        sdffile = f.read()
        try:
            result = spawn_model(f"wayPointMarker",
                                 sdffile,
                                 f"wayPointMarker",
                                 ipose, "world")
        except rospy.ServiceException:
            print("/gazebo/SpawnModel service call failed")

    # --> RL agent
    model = PPO.load("/home/eduardo/USVSim/eboat_ws/src/eboat_gz_1/models/PPO/ensign_five/eboat_ocean_50")

    #--> STORE OBSERVATIONAL DATA TO SAVE
    ctrlData = []

    #--> SET WIND SPEED AND DIRECTION
    windSpeed = np.concatenate((rot(wind_speed, wind_angle), np.zeros(1)), dtype=np.float32)
    wind_pub.publish(Point(windSpeed[0], windSpeed[1], windSpeed[2]))

    #--> WAITE FOR THE ENVIRONMENT TO BE STABLE
    for i in range(3):
        obs = getObservations()

    # --> PAUSE SIMULATION
    rospy.wait_for_service("/gazebo/pause_physics")
    try:
        pause()
    except(rospy.ServiceException) as e:
        print(("/gazebo/pause_physics service call failed!"))

    #--> RESET ENVIRONMENT
    rospy.wait_for_service('/gazebo/reset_simulation')
    try:
        reset_proxy()
    except (rospy.ServiceException) as e:
        print("/gazebo/reset_simulation service call failed!")

    for k in range(20):
        print("---------------------------------------------")
        print(f"Mission track: {k}")
        cumu = 0.0
        for i in path2follow:
            waypoint = navpoints[i]
            print(f"Traveling to {navids[i]}")
            setState("wayPointMarker", pose=waypoint, theta=0)

            time.sleep(4)

            #--> UNPAUSE SIMULATION
            rospy.wait_for_service('/gazebo/unpause_physics')
            try:
                unpause()
            except(rospy.ServiceException) as e:
                print(("/gazebo/unpause_physics service call failed!"))

            #--> GET OBSERVATIONS
            obs = getObservations()

            # --> PAUSE SIMULATION
            rospy.wait_for_service("/gazebo/pause_physics")
            try:
                pause()
            except(rospy.ServiceException) as e:
                print(("/gazebo/pause_physics service call failed!"))

            flag = True
            while obs[0] > 6:
                # --> ESAILOR
                robs = observationRescale(obs)[:5]
                action, _state = model.predict(robs)

                # --> SEND ACTION TO THE BOAT CONTROL INTERFACE
                ract = actionRescale(action)
                propVel_pub.publish(int(ract[0]))
                boomAng_pub.publish(ract[1])
                rudderAng_pub.publish(ract[2])

                # -->COMPUTE REWARD
                if flag:
                    flag = False
                    rw = 0.0
                elif obs[0] <= 6:
                    rw = 1
                else:
                    rw = rewardFunction(obs, ctrlData[-1][1], dmax)

                cumu += rw

                # --> STORE ACTION/STATE DATA
                ctrlData.append(np.concatenate((np.array([k]), obs, ract, np.array([rw, cumu]))))

                # --> UNPAUSE SIMULATION
                rospy.wait_for_service('/gazebo/unpause_physics')
                try:
                    unpause()
                except(rospy.ServiceException) as e:
                    print(("/gazebo/unpause_physics service call failed!"))

                # --> GET OBSERVATIONS
                obs = getObservations()

                # --> PAUSE SIMULATION
                rospy.wait_for_service("/gazebo/pause_physics")
                try:
                    pause()
                except(rospy.ServiceException) as e:
                    print(("/gazebo/pause_physics service call failed!"))

        propVel_pub.publish(0)
        boomAng_pub.publish(0.0)
        rudderAng_pub.publish(0.0)
        # -->RESET ENVIRONMENT
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            reset_proxy()
        except (rospy.ServiceException) as e:
            print("/gazebo/reset_simulation service call failed!")

    # -->PAUSE SIMULATION
    rospy.wait_for_service("/gazebo/pause_physics")
    try:
        pause()
    except(rospy.ServiceException) as e:
        print(("/gazebo/pause_physics service call failed!"))

    df = pd.DataFrame(ctrlData, columns=["track", "dist", "traj_ang", "surge_vel", "apwind_speed", "apwind_ang", "boom_angle", "rudder_angle", "prop_speed", "roll_angle","X","Y","actProp","actBoom","actRudder","reward","cumu"])
    df.to_csv("./auto_mission_2.csv", sep=";", index = False)
    print(df)

def manualMission(pause, unpause, boomAng_pub, rudderAng_pub, propVel_pub, wind_pub, reset_proxy, navpoints, navids, path2follow, sep_dist = 100, wind_speed = 10, wind_angle = 0):
    dmax = 1.25 * sep_dist

    #--> WAYPOINT FOR GUIDANCE
    ipose            = Pose()
    ipose.position.x = 0
    ipose.position.y = 0
    ipose.position.z = 0
    spawn_model      = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    with open(
            "/home/eduardo/USVSim/eboat_ws/src/eboat_gz_1/eboat_description/models/wayPointMarker/model.sdf") as f:
        sdffile = f.read()
        try:
            result = spawn_model(f"wayPointMarker",
                                 sdffile,
                                 f"wayPointMarker",
                                 ipose, "world")
        except rospy.ServiceException:
            print("/gazebo/SpawnModel service call failed")

    #--> STORE OBSERVATIONAL DATA TO SAVE
    ctrlData = []

    #--> SET WIND SPEED AND DIRECTION
    windSpeed = np.concatenate((rot(wind_speed, wind_angle), np.zeros(1)), dtype=np.float32)
    wind_pub.publish(Point(windSpeed[0], windSpeed[1], windSpeed[2]))

    sensor_array = subprocess.Popen([sys.executable, "./sensor_array.py"])

    #--> WAITE FOR THE ENVIRONMENT TO BE STABLE
    for i in range(3):
        obs = getObservations()

    # --> PAUSE SIMULATION
    rospy.wait_for_service("/gazebo/pause_physics")
    try:
        pause()
    except(rospy.ServiceException) as e:
        print(("/gazebo/pause_physics service call failed!"))

    #--> RESET ENVIRONMENT
    rospy.wait_for_service('/gazebo/reset_simulation')
    try:
        reset_proxy()
    except (rospy.ServiceException) as e:
        print("/gazebo/reset_simulation service call failed!")

    for k in range(1):
        cumu = 0.0
        for i in path2follow:
            waypoint = navpoints[i]
            print(f"Traveling to {navids[i]}")
            setState("wayPointMarker", pose=waypoint, theta=0)

            time.sleep(4)

            #--> UNPAUSE SIMULATION
            rospy.wait_for_service('/gazebo/unpause_physics')
            try:
                unpause()
            except(rospy.ServiceException) as e:
                print(("/gazebo/unpause_physics service call failed!"))

            #--> GET OBSERVATIONS
            obs = getObservations()

            # --> PAUSE SIMULATION
            rospy.wait_for_service("/gazebo/pause_physics")
            try:
                pause()
            except(rospy.ServiceException) as e:
                print(("/gazebo/pause_physics service call failed!"))

            flag = True
            while obs[0] > 6:
                # -->USER
                print("\n--------------------------------------------------")
                print("Distance        : {:6.3f}".format(obs[0]))
                print("Trajectory angle: {:6.3f}".format(obs[1]))
                print("Surge velocity  : {:6.3f}".format(obs[2]))
                print("Ap. wind speed  : {:6.3f}".format(obs[3]))
                input_action = input("Enter Action (p, b, r): ").split(" ")
                if len(input_action) > 1:
                    action = np.array(input_action, dtype=np.float32)

                # --> SEND ACTION TO THE BOAT CONTROL INTERFACE
                propVel_pub.publish(int(action[0]))
                boomAng_pub.publish(action[1])
                rudderAng_pub.publish(action[2])

                # -->COMPUTE REWARD
                if flag:
                    flag = False
                    rw = 0.0
                elif obs[0] <= 6:
                    rw = 1
                else:
                    rw = rewardFunction(obs, ctrlData[-1][1], dmax)

                cumu += rw

                # --> STORE ACTION/STATE DATA
                ctrlData.append(np.concatenate((np.array([k]), obs, action, np.array([rw, cumu]))))

                # --> UNPAUSE SIMULATION
                rospy.wait_for_service('/gazebo/unpause_physics')
                try:
                    unpause()
                except(rospy.ServiceException) as e:
                    print(("/gazebo/unpause_physics service call failed!"))

                # --> GET OBSERVATIONS
                obs = getObservations()

                # --> PAUSE SIMULATION
                rospy.wait_for_service("/gazebo/pause_physics")
                try:
                    pause()
                except(rospy.ServiceException) as e:
                    print(("/gazebo/pause_physics service call failed!"))

        propVel_pub.publish(0)
        boomAng_pub.publish(0.0)
        rudderAng_pub.publish(0.0)
        # -->RESET ENVIRONMENT
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            reset_proxy()
        except (rospy.ServiceException) as e:
            print("/gazebo/reset_simulation service call failed!")

    # -->PAUSE SIMULATION
    rospy.wait_for_service("/gazebo/pause_physics")
    try:
        pause()
    except(rospy.ServiceException) as e:
        print(("/gazebo/pause_physics service call failed!"))

    df = pd.DataFrame(ctrlData, columns=["track", "dist", "traj_ang", "surge_vel", "apwind_speed", "apwind_ang", "boom_angle", "rudder_angle", "prop_speed", "roll_angle","X","Y","actProp","actBoom","actRudder","reward","cumu"])
    df.to_csv("./manaual_mission.csv", sep=";", index = False)
    print(df)
    sensor_array.kill()


def regattaMulti(pause, unpause, boomAng_pub, rudderAng_pub, propVel_pub, wind_pub, reset_proxy, sep_dist=100,
                 wind_speed=10, wind_angle=0):
    dmax = 1.25 * sep_dist
    navpoints = [np.concatenate((rot(sep_dist, 45), np.zeros(1)), dtype=np.float32)]
    navpoints.append(navpoints[0] + np.concatenate((rot(sep_dist, -45), np.zeros(1)), dtype=np.float32))
    navpoints.append(navpoints[1] + np.concatenate((rot(sep_dist, -135), np.zeros(1)), dtype=np.float32))
    navpoints.append(np.array([0, 0, 0]))
    navids = ["navpoint B", "navpoint C", "navpoint D", "navpoint A"]

    # --> CREATE WAYPOINTS FOR VISUALIZATION
    path2follow = [0, 1, 2, 3, 1, 3]
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    for i in range(len(navpoints)):
        waypoint = navpoints[i]
        ipose = Pose()
        ipose.position.x = waypoint[0]
        ipose.position.y = waypoint[1]
        ipose.position.z = waypoint[2]
        with open(
                "/home/eduardo/USVSim/eboat_ws/src/eboat_gz_1/eboat_description/models/wayPointMarker/model.sdf") as f:
            sdffile = f.read()
            try:
                result = spawn_model(f"wayPointMarker{i}",
                                     sdffile,
                                     f"wayPointMarker{i}",
                                     ipose, "world")
            except rospy.ServiceException:
                print("/gazebo/SpawnModel service call failed")

    autoMission(pause, unpause, boomAng_pub, rudderAng_pub, propVel_pub, wind_pub, reset_proxy, navpoints, navids, path2follow, sep_dist, wind_speed, wind_angle)
    # manualMission(pause, unpause, boomAng_pub, rudderAng_pub, propVel_pub, wind_pub, reset_proxy, navpoints, navids, path2follow, sep_dist, wind_speed, wind_angle)

def manualTack(pause, unpause, boomAng_pub, rudderAng_pub, propVel_pub, wind_pub, wind_speed = 10, wind_angle = 0):
    # -->PAUSE SIMULATION
    rospy.wait_for_service("/gazebo/pause_physics")
    try:
        pause()
    except(rospy.ServiceException) as e:
        print(("/gazebo/pause_physics service call failed!"))

    windSpeed = np.concatenate((rot(wind_speed, wind_angle), np.zeros(1)), dtype=np.float32)
    wind_pub.publish(Point(windSpeed[0], windSpeed[1], windSpeed[2]))

    # --> UNPAUSE SIMULATION
    rospy.wait_for_service('/gazebo/unpause_physics')
    try:
        unpause()
    except(rospy.ServiceException) as e:
        print(("/gazebo/unpause_physics service call failed!"))

    # --> GET OBSERVATIONS
    obsData = None
    while obsData is None:
        try:
            obsData = rospy.wait_for_message('/eboat/mission_control/observations', Float32MultiArray,
                                             timeout=20).data
        except:
            pass
        # --> obsData = [distance, trajectory angle, linear velocity, aparent wind speed, aparent wind angle, boom angle, rudder angle, eletric propultion speed, roll angle]
        #               [   0    ,        1        ,       2        ,         3         ,         4         ,     5     ,      6      ,            7            ,     8     ]

    obs = np.array(obsData, dtype=float)

    sensor_array = subprocess.Popen([sys.executable, "./sensor_array.py"])

    for i in range(150):
        # -->PAUSE SIMULATION
        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            pause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/pause_physics service call failed!"))

        input_action = input("              PS BA RA\nEnter Action: ").split(" ")

        # --> UNPAUSE SIMULATION
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            unpause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/unpause_physics service call failed!"))

        if len(input_action) > 1:
            action = np.array(input_action, dtype=np.float32)
            if action[1] < 0:
                break
            else:
                propVel_pub.publish(int(action[0]))
                boomAng_pub.publish(action[1])
                rudderAng_pub.publish(action[2])

        # --> GET OBSERVATIONS
        obsData = None
        while obsData is None:
            try:
                obsData = rospy.wait_for_message('/eboat/mission_control/observations', Float32MultiArray,
                                                 timeout=20).data
            except:
                pass
            # --> obsData = [distance, trajectory angle, linear velocity, aparent wind speed, aparent wind angle, boom angle, rudder angle, eletric propultion speed, roll angle]
            #               [   0    ,        1        ,       2        ,         3         ,         4         ,     5     ,      6      ,            7            ,     8     ]

        obs = np.array(obsData, dtype=float)

        print(f"Surge velocity = {obs[2]}")

    sensor_array.kill()

def autoTack(pause, unpause, boomAng_pub, rudderAng_pub, propVel_pub, wind_pub, wind_speed = -6.17, wind_angle = 0):
    # -->PAUSE SIMULATION
    rospy.wait_for_service("/gazebo/pause_physics")
    try:
        pause()
    except(rospy.ServiceException) as e:
        print(("/gazebo/pause_physics service call failed!"))

    windSpeed = np.concatenate((rot(wind_speed, wind_angle), np.zeros(1)), dtype=np.float32)
    wind_pub.publish(Point(windSpeed[0], windSpeed[1], windSpeed[2]))

    setState("eboat", pose=[0, 0, 0], theta=-45)

    # --> UNPAUSE SIMULATION
    rospy.wait_for_service('/gazebo/unpause_physics')
    try:
        unpause()
    except(rospy.ServiceException) as e:
        print(("/gazebo/unpause_physics service call failed!"))

    sensor_array = subprocess.Popen([sys.executable, "./sensor_array.py"])

    followed_path = []

    for i in range(15):
        # --> GET OBSERVATIONS
        obsData = None
        while obsData is None:
            try:
                obsData = rospy.wait_for_message('/eboat/mission_control/observations', Float32MultiArray,
                                                 timeout=20).data
            except:
                pass
            # --> obsData = [distance, trajectory angle, linear velocity, aparent wind speed, aparent wind angle, boom angle, rudder angle, eletric propultion speed, roll angle]
            #               [   0    ,        1        ,       2        ,         3         ,         4         ,     5     ,      6      ,            7            ,     8     ]

        obs = np.array(obsData, dtype=float)

        followed_path.append(obs[9:])

        if i == 6:
            ra = -60
        else:
            ra = 0

        propVel_pub.publish(0)
        boomAng_pub.publish(15)
        rudderAng_pub.publish(ra)

        print(f"Surge velocity = {obs[2]}")

    # -->PAUSE SIMULATION
    rospy.wait_for_service("/gazebo/pause_physics")
    try:
        pause()
    except(rospy.ServiceException) as e:
        print(("/gazebo/pause_physics service call failed!"))
    _ = input("Simulacao pausada")

    df = pd.DataFrame(followed_path, columns=["X", "Y"])
    df.to_csv("./tack_position_data.csv", sep=";", index=False)

    sensor_array.kill()

def autoTackMultiWind(pause, unpause, boomAng_pub, rudderAng_pub, propVel_pub, wind_pub, reset_proxy, wind_angle = 0):
    # -->PAUSE SIMULATION
    rospy.wait_for_service("/gazebo/pause_physics")
    try:
        pause()
    except(rospy.ServiceException) as e:
        print(("/gazebo/pause_physics service call failed!"))

    sensor_array = subprocess.Popen([sys.executable, "./sensor_array.py"])

    followed_path = []
    wind_speed = np.array([5, 7, 9, 12])
    for ws in wind_speed:
        windSpeed = np.concatenate((rot((ws * -0.514444), wind_angle), np.zeros(1)), dtype=np.float32)
        wind_pub.publish(Point(windSpeed[0], windSpeed[1], windSpeed[2]))

        setState("eboat", pose=[0, 0, 0], theta=-45)

        # --> UNPAUSE SIMULATION
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            unpause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/unpause_physics service call failed!"))

        for i in range(21):
            # --> GET OBSERVATIONS
            obs = getObservations()

            followed_path.append([ws, obs[2], obs[9], obs[10]])

            if (i == 10):
                ra = -60
                print(f"wind/surge velocity = {obs[3]}/{obs[2]}")
            else:
                ra = 0

            propVel_pub.publish(0)
            boomAng_pub.publish(15)
            rudderAng_pub.publish(ra)

        propVel_pub.publish(0)
        boomAng_pub.publish(0)
        rudderAng_pub.publish(0)
        wind_pub.publish(Point(0, 0, 0))
        time.sleep(4)

        # -->PAUSE SIMULATION
        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            pause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/pause_physics service call failed!"))

        # -->RESET ENVIRONMENT
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            reset_proxy()
        except (rospy.ServiceException) as e:
            print("/gazebo/reset_simulation service call failed!")



    # _ = input("Simulacao pausada")

    df = pd.DataFrame(followed_path, columns=["wind","surge","X", "Y"])
    df.to_csv("./tack_position_multi_data.csv", sep=";", index=False)

    sensor_array.kill()

def sailinjgPoints(boomAng_pub, rudderAng_pub, wind_pub, reset_proxy):
    sensor_array = subprocess.Popen([sys.executable, "./sensor_array.py"])
    data = []
    for wind_speed in np.arange(13,dtype=int):
        print("-----------------------------------------------------")
        for C in [-1, 1]:
            for alpha in [30, 60, 90, 135, 160]:
                if wind_speed > 0:
                    alphal = ((alpha - 180) * C)
                    windVec = np.concatenate((rot(wind_speed, alphal), np.zeros(1)), dtype=np.float32)
                    wind_pub.publish(Point(windVec[0], windVec[1], windVec[2]))
                    for t in range(6):
                        print(f"ws  = {wind_speed}\nang = {alpha}\nt   = {t}")
                        obs = getObservations()
                        if obs[4] > alphal:
                            rudderAng_pub.publish(-0.2*abs(obs[4]-alphal))
                        elif obs[4] < alphal:
                            rudderAng_pub.publish(0.2*abs(obs[4]-alphal))
                        else:
                            rudderAng_pub.publish(0)
                        #
                        if (abs(obs[4]) > 139) & (abs(obs[4]) < 161):    # ~30
                            boomAng_pub.publish(10)
                        elif (abs(obs[4]) > 109) & (abs(obs[4]) < 131):  # ~60
                            boomAng_pub.publish(30)
                        elif (abs(obs[4]) > 79) & (abs(obs[4]) < 101):   # ~90
                            boomAng_pub.publish(60)
                        elif (abs(obs[4]) > 34) & (abs(obs[4]) < 56):   # ~135
                            boomAng_pub.publish(80)
                        else:
                            boomAng_pub.publish(85)
                        #
                        data.append(np.concatenate((np.array([wind_speed, alpha*C, t+4]), obs[2:-1]), axis=0))
                else:
                    for t in range(6):
                        print(f"ws  = {wind_speed}\nang = {alpha}\nt   = {t}")
                        obs = getObservations()
                        data.append(np.concatenate((np.array([wind_speed, alpha * C, t + 4]), obs[2:-1]), axis=0))


                rudderAng_pub.publish(0)
                boomAng_pub.publish(0)
                wind_pub.publish(Point(0, 0, 0))
                _ = getObservations()

                # -->RESET ENVIRONMENT
                rospy.wait_for_service('/gazebo/reset_simulation')
                try:
                    reset_proxy()
                except (rospy.ServiceException) as e:
                    print("/gazebo/reset_simulation service call failed!")

    df = pd.DataFrame(data, columns=["truewind", "sailingpoint", "time", "linear velocity", "apwindspeed", "apwindang", "boom angle", "rudder angle", "prop speed"])
    df.to_csv("sailing_points.csv", sep = ";", index=False)
    print(df)
    sensor_array.kill()


def sailinjgPoints(pause, unpause, boomAng_pub, rudderAng_pub, wind_pub, reset_proxy, il = 6):
    sensor_array = subprocess.Popen([sys.executable, "./sensor_array.py"])
    data = []

    # -->PAUSE SIMULATION
    rospy.wait_for_service("/gazebo/pause_physics")
    try:
        pause()
    except(rospy.ServiceException) as e:
        print(("/gazebo/pause_physics service call failed!"))

    for wind_speed in np.arange(13, dtype=int):
        print("-----------------------------------------------------")
        for C in [1]: #[-1, 1]:
            for alpha in [30, 60, 90, 135, 160]:
                if wind_speed > 0:
                    alphal = ((alpha - 180) * C)
                    windVec = np.concatenate((rot(wind_speed, -alphal), np.zeros(1)), dtype=np.float32)
                    wind_pub.publish(Point(windVec[0], windVec[1], windVec[2]))

                    # --> UNPAUSE SIMULATION
                    rospy.wait_for_service('/gazebo/unpause_physics')
                    try:
                        unpause()
                    except(rospy.ServiceException) as e:
                        print(("/gazebo/unpause_physics service call failed!"))

                    # -->GET OBSERVATIONS
                    obs = getObservations()

                    for t in range(il):
                        # -->PAUSE SIMULATION
                        rospy.wait_for_service("/gazebo/pause_physics")
                        try:
                            pause()
                        except(rospy.ServiceException) as e:
                            print(("/gazebo/pause_physics service call failed!"))

                        print(f"ws  = {wind_speed}\nang = {alpha}\nt   = {t}")
                        #
                        data.append(np.concatenate((np.array([wind_speed, alpha * C, t * 4]), obs[2:-1]), axis=0))
                        #
                        if obs[2] >= 1:
                            rudderAng_pub.publish(-obs[1] / obs[2])
                        else:
                            rudderAng_pub.publish(-obs[1])
                        #
                        if (abs(obs[4]) > 135) & (abs(obs[4]) < 166):  # ~30 15-45
                            boomAng_pub.publish(10)
                        elif (abs(obs[4]) > 105) & (abs(obs[4]) < 136):  # ~60 45-75
                            boomAng_pub.publish(30)
                        elif (abs(obs[4]) > 75) & (abs(obs[4]) < 106):  # ~90 75-105
                            boomAng_pub.publish(60)
                        # elif (abs(obs[4]) > 34) & (abs(obs[4]) < 56):  # ~135 120-180
                        #     boomAng_pub.publish(80)
                        else: # ~135 120-180
                            boomAng_pub.publish(85)
                        #
                        # --> UNPAUSE SIMULATION
                        rospy.wait_for_service('/gazebo/unpause_physics')
                        try:
                            unpause()
                        except(rospy.ServiceException) as e:
                            print(("/gazebo/unpause_physics service call failed!"))

                        # -->GET OBSERVATIONS
                        obs = getObservations()
                else:
                    # --> UNPAUSE SIMULATION
                    rospy.wait_for_service('/gazebo/unpause_physics')
                    try:
                        unpause()
                    except(rospy.ServiceException) as e:
                        print(("/gazebo/unpause_physics service call failed!"))

                    # -->GET OBSERVATIONS
                    obs = getObservations()

                    for t in range(il):
                        # -->PAUSE SIMULATION
                        rospy.wait_for_service("/gazebo/pause_physics")
                        try:
                            pause()
                        except(rospy.ServiceException) as e:
                            print(("/gazebo/pause_physics service call failed!"))

                        print(f"ws  = {wind_speed}\nang = {alpha}\nt   = {t}")

                        data.append(np.concatenate((np.array([wind_speed, alpha * C, t * 4]), obs[2:-1]), axis=0))

                        # --> UNPAUSE SIMULATION
                        rospy.wait_for_service('/gazebo/unpause_physics')
                        try:
                            unpause()
                        except(rospy.ServiceException) as e:
                            print(("/gazebo/unpause_physics service call failed!"))

                        # -->GET OBSERVATIONS
                        obs = getObservations()

                rudderAng_pub.publish(0)
                boomAng_pub.publish(0)
                wind_pub.publish(Point(0, 0, 0))
                _ = getObservations()

                # -->RESET ENVIRONMENT
                rospy.wait_for_service('/gazebo/reset_simulation')
                try:
                    reset_proxy()
                except (rospy.ServiceException) as e:
                    print("/gazebo/reset_simulation service call failed!")

    df = pd.DataFrame(data, columns=["truewind", "sailingpoint", "time", "linear velocity", "apwindspeed", "apwindang",
                                     "boom angle", "rudder angle", "prop speed"])
    df.to_csv("sailing_points.csv", sep=";", index=False)
    print(df)
    sensor_array.kill()

def close():
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

def main():
    port = "11311"
    port_gazebo = "11345"
    os.environ["ROS_MASTER_URI"] = "http://localhost:" + port
    os.environ["GAZEBO_MASTER_URI"] = "http://localhost:" + port_gazebo

    ros_path = os.path.dirname(subprocess.check_output(["which", "roscore"]))

    print(ros_path)

    EBOAT_HOME = "/home/eduardo/USVSim/eboat_ws/src/eboat_gz_1"
    launch_file_path = os.path.join(EBOAT_HOME, "eboat_gazebo/launch/ocean_fixed_cam.launch")
    # launch_file_path = os.path.join(EBOAT_HOME, "eboat_gazebo/launch/ocean.launch")


    roslaunch = subprocess.Popen([sys.executable, os.path.join(ros_path, b"roslaunch"), "-p", port, launch_file_path])
    print("Gazebo launched!")

    gzclient_pid = 0
    rospy.init_node('maneuvers', anonymous=True)

    boomAng_pub   = rospy.Publisher("/eboat/control_interface/sail", Float32, queue_size=5)
    rudderAng_pub = rospy.Publisher("/eboat/control_interface/rudder", Float32, queue_size=5)
    propVel_pub   = rospy.Publisher("/eboat/control_interface/propulsion", Int16, queue_size=5)
    wind_pub      = rospy.Publisher("/eboat/atmosferic_control/wind", Point, queue_size=5)
    unpause       = rospy.ServiceProxy('/gazebo/unpause_physics' , Empty)
    pause         = rospy.ServiceProxy('/gazebo/pause_physics'   , Empty)
    reset_proxy   = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
    reset_world   = rospy.ServiceProxy('/gazebo/reset_world', Empty)
    # set_state     = rospy.ServiceProxy('/gazebo/set_model_state' , SetModelState)

    obs = getObservations()

    # _ = man1(propVel_pub, reset_proxy)
    # _ = man2(propVel_pub, rudderAng_pub, reset_proxy, pause, unpause)
    # acceleration(propVel_pub, reset_proxy)
    # movingTurn(propVel_pub, rudderAng_pub, reset_proxy, dt=20)
    # turningFromRest(propVel_pub, rudderAng_pub, reset_proxy, dt=15)
    # autoTackMultiWind(pause, unpause, boomAng_pub, rudderAng_pub, propVel_pub, wind_pub, reset_proxy, wind_angle=0)
    regattaMulti(pause, unpause, boomAng_pub, rudderAng_pub, propVel_pub, wind_pub, reset_proxy, sep_dist=100, wind_speed=10, wind_angle=0)
    # sailinjgPoints(pause, unpause, boomAng_pub, rudderAng_pub, wind_pub, reset_proxy, il=6)

    # regatta(pause, unpause, boomAng_pub, rudderAng_pub, propVel_pub, wind_pub, sep_dist = 100, wind_speed = 10, wind_angle = 0)
    # manualTack(pause, unpause, boomAng_pub, rudderAng_pub, propVel_pub, wind_pub, wind_speed=7, wind_angle=135)
    # autoTack(pause, unpause, boomAng_pub, rudderAng_pub, propVel_pub, wind_pub, wind_speed=-6.17, wind_angle=0)

    #--> CLOSE
    close()

    return 0

if __name__ == '__main__':
    _ = main()