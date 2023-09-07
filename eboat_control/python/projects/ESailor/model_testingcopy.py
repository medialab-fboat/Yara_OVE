#!/home/araujo/miniconda3/envs/esailor/bin/python

import gymnasium as gym
import rospy
import numpy as np
import sys, os, signal, subprocess
import time
import glob

from scipy import interpolate

from datetime import datetime

from gymnasium import utils, spaces
from std_srvs.srv import Empty

from std_msgs.msg import Float32, Int16, Float32MultiArray
from geometry_msgs.msg import Point
from gymnasium.utils import seeding

from tf.transformations import quaternion_from_euler
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelState

import random
from rosgraph_msgs.msg import Clock

from std_srvs.srv import Empty

from std_msgs.msg import Float32, Int16, Float32MultiArray
from geometry_msgs.msg import Point, Pose

from tf.transformations import quaternion_from_euler
from gazebo_msgs.srv import SetModelState, SpawnModel, DeleteModel
from gazebo_msgs.msg import ModelState

def getObservations(self):
    obsData = None
    while obsData is None:
        try:
            obsData = rospy.wait_for_message('/eboat/mission_control/observations', Float32MultiArray,
                                                 timeout=20).data
        except:
            pass
            # --> obsData = [distance, trajectory angle, linear velocity, aparent wind speed, aparent wind angle, boom angle, rudder angle, eletric propultion speed, roll angle]
            #               [   0    ,        1        ,       2        ,         3         ,         4         ,     5     ,      6      ,            7            ,     8     ]
    print(obsData)

    return np.array(obsData, dtype=float)

def rescale(self, m, rmin, rmax, tmin, tmax):
        
        return (((m - rmin) / (rmax - rmin)) * (tmax - tmin) + tmin)

def observationRescale(self, observations):
    lobs = len(observations)
    robs = np.zeros(lobs, dtype=np.float32)
        # --> Distance from the waypoint (m) [0   , DMAX];
    robs[0] = 2 * (observations[0] / self.DMAX) - 1
        # --> Trajectory angle               [-180, 180]
    robs[1] = observations[1] / 180.0
        # --> Boat linear velocity (m/s)     [0   , 10 ]
    robs[2] = observations[2] / 5 - 1
        # --> Aparent wind speed (m/s)       [0   , 30]
    robs[3] = observations[3] / 15 - 1
        # --> Apparent wind angle            [-180, 180]
    robs[4] = observations[4] / 180.0
    if lobs > 5:
            # --> Boom angle                     [0   , 90]
        robs[5] = (observations[5] / 45.0) - 1
            # --> Rudder angle                   [-60 , 60 ]
        robs[6] = observations[6] / 60.0
            # --> Electric propulsion speed      [-5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5]
        robs[7] = observations[7] / 5.0
            # --> Roll angle                     [-180, 180]
        robs[8] = observations[8] / 180.0

    return robs

def main():
    random_number    = random.randint(10000, 15000)
    # port_ros    = str(random_number)
    # port_gazebo = str(random_number + 1)
    port_ros = "11311"
    port_gazebo = "11345"

    #-->EXPORT ADDRESS FOR THE ROS MASTER NODE AND THE GAZEBO SERVER
    os.environ["ROS_MASTER_URI"]    = "http://localhost:" + port_ros
    os.environ["GAZEBO_MASTER_URI"] = "http://localhost:" + port_gazebo

    #-->SEARCH FOR A LAUNCH FILE
    HOME = os.path.expanduser('~')
    path2launchfile = None
    if path2launchfile == None:
        print("As the user did not provide a viable launch file, I will search for one.\nThis may take a while, pelase wait!")
        files = glob.glob(os.path.join(HOME,"**/*eboat_gazebo/launch/ocean_RL_training.launch"), recursive=True)
        if len(files) > 0:
            path2launchfile = files[0]
            del(files)
        else:
            path2launchfile = input("\nI did not find a viable launch file!\nPlease provide an valid path:\n")
            if (((len(path2launchfile) > 0) & (not(os.path.exists(path2launchfile)))) | (len(path2launchfile) == 0)):
                raise IOError("File " + path2launchfile + " does not exist")

    #-->LAUNCH THE SIMULATION USING SUBPROCESS
    ros_path = os.path.dirname(subprocess.check_output(["which", "roscore"]))
    _roslaunch = subprocess.Popen([sys.executable, os.path.join(ros_path, b"roslaunch"), "-p", port_ros, path2launchfile])
    
    model = PPO.load("/home/araujo/yara_ws/src/Yara_OVE/esailor/models/PPO/model35_25082023_23_29_13/eboat_ocean_50.zip")
    
    #-->INITIALIZE A ROS NODE FOR THE TRAINING PROCESS
    try:
        rospy.init_node(f"gym", anonymous=True)
    except:
        print("ROSMASTER is not running!")
        print(time.time())
        exit(1)
            
    #-->DEFINE THE NECESSARY ROS TOPICS AND SERVICES
    boomAng_pub   = rospy.Publisher(f"/eboat/control_interface/sail", Float32, queue_size=5)
    rudderAng_pub = rospy.Publisher(f"/eboat/control_interface/rudder", Float32, queue_size=5)
    propVel_pub   = rospy.Publisher(f"/eboat/control_interface/propulsion", Int16, queue_size=5)
    wind_pub      = rospy.Publisher(f"/eboat/atmosferic_control/wind", Point, queue_size=5)
    unpause       = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    pause         = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
      
    obs = observationRescale(getObservations()[:5])
    count = 0
    while obs[0] < 5:
        act = model.predict(obs)
        print(act)
        propVel_pub.publish(int(act[0] * 5.0))
        boomAng_pub.publish((act[1] + 1) * 45.0)
        rudderAng_pub.publish(act[2] * 60.0)
        
        obs = observationRescale(getObservations()[:5])
        
        count += 1
        
        if count > 100:
            break
  

if __name__ == '__main__':
    main()

print("Testing completed.")