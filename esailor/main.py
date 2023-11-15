import numpy as np
import rospy
import time
import os, sys
import subprocess

import gymnasium as gym

import esailor_gym

from glob import glob

HOME = os.path.expanduser("~")

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
def testeDoAgente():

    path2launch = []
    for dir, _, _ in os.walk(os.path.join(HOME)):
        path2launch.extend(glob(os.path.join(dir, "*/src/Yara_OVE/eboat_gazebo/launch/ocean_fixed_cam.launch")))

    print("\n\n--------------------------------------------------------")
    if len(path2launch) > 0:
        path2launch = path2launch[0]
        print(path2launch)
    else:
        print("Something goes wrong! Path to launch file not found!")
        exit(1)

    ############################################################################################
    port = "11311"
    port_gazebo = "11345"
    os.environ["ROS_MASTER_URI"] = "http://localhost:" + port
    os.environ["GAZEBO_MASTER_URI"] = "http://localhost:" + port_gazebo

    ros_path = os.path.dirname(subprocess.check_output(["which", "roscore"]))

    print(ros_path)

    EBOAT_HOME = "/home/araujo/eboat_ws/src/eboat_gz_1"
    launch_file_path = os.path.join(EBOAT_HOME, "eboat_gazebo/launch/ocean_fixed_cam.launch")

    roslaunch = subprocess.Popen([sys.executable, os.path.join(ros_path, b"roslaunch"), "-p", port, launch_file_path])
    print("Gazebo launched!")

    ############################################################################################
    rospy.init_node('esailor', anonymous=True)
    boomAng_pub   = rospy.Publisher("/eboat/control_interface/sail", Float32, queue_size=5)
    rudderAng_pub = rospy.Publisher("/eboat/control_interface/rudder", Float32, queue_size=5)
    propVel_pub   = rospy.Publisher("/eboat/control_interface/propulsion", Int16, queue_size=5)
    wind_pub      = rospy.Publisher("/eboat/atmosferic_control/wind", Point, queue_size=5)
    unpause       = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    pause         = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
    reset_proxy   = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
    reset_world   = rospy.ServiceProxy('/gazebo/reset_world', Empty)
    # set_state     = rospy.ServiceProxy('/gazebo/set_model_state' , SetModelState)

    obs = getObservations()

    close()

def agentPPOTraining():
    env = gym.make(f'GazeboOceanEboatEnvCC-v0')

    time.sleep(20)

    print("\n\n\nPASSANDO ADIANTE\n\n")

    env.close()
    os.system('/home/araujo/eboat_ws/kill_gaz.sh')

if __name__ == '__main__':
    # testeDoAgente()
    agentPPOTraining()

    # env = gym.make(f'GazeboOceanEboatEnvCC-v1')
    #
    # time.sleep(20)
    #
    # print("\n\n\nPASSANDO ADIANTE\n\n")
    #
    # env.close()
    # os.system('/home/araujo/eboat_ws/kill_gaz.sh')
