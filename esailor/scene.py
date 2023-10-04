#!/home/araujo/miniconda3/envs/esailor/bin/python

#-->PYTHON UTIL
import time
import random
import numpy as np
import glob
from datetime import datetime
import sys, os, signal, subprocess

#-->ROS
import rospy
from std_msgs.msg import Float32, Int16, Float32MultiArray
from std_srvs.srv import Empty

#-->GAZEBO
from geometry_msgs.msg import Point, Pose
from gazebo_msgs.srv import SetModelState, SpawnModel, DeleteModel

class scene():
    def __init__(self, path2launchfile = None):
        random_number = random.randint(10000, 15000)
        self.port_ros = str(random_number)
        self.port_gazebo = str(random_number + 1)
        # self.port_ros = "11311"
        # self.port_gazebo = "11345"

        # -->EXPORT ADDRESS FOR THE ROS MASTER NODE AND THE GAZEBO SERVER
        os.environ["ROS_MASTER_URI"] = "http://localhost:" + self.port_ros
        os.environ["GAZEBO_MASTER_URI"] = "http://localhost:" + self.port_gazebo

        # -->SEARCH FOR A LAUNCH FILE
        HOME = os.path.expanduser('~')
        if path2launchfile == None:
            print(
                "As the user did not provide a viable launch file, I will search for one.\nThis may take a while, pelase wait!")
            # files = glob.glob(os.path.join(HOME, "**/*ocean_fixed_cam.launch"), recursive=True)
            files = glob.glob(os.path.join(HOME, "**/*ocean.launch"), recursive=True)
            if len(files) > 0:
                path2launchfile = files[0]
                del (files)
            else:
                path2launchfile = input("\nI did not find a viable launch file!\nPlease provide an valid path:\n")
                if (((len(path2launchfile) > 0) & (not (os.path.exists(path2launchfile)))) | (
                        len(path2launchfile) == 0)):
                    raise IOError("File " + path2launchfile + " does not exist")

        # -->LAUNCH THE SIMULATION USING SUBPROCESS
        ros_path = os.path.dirname(subprocess.check_output(["which", "roscore"]))
        self._roslaunch = subprocess.Popen(
            [sys.executable, os.path.join(ros_path, b"roslaunch"), "-p", self.port_ros, path2launchfile])

        # -->INITIALIZE A ROS NODE
        try:
            rospy.init_node(f"scene", anonymous=True)
        except:
            print("ROSMASTER is not running!")
            print(time.time())
            exit(1)

        # -->DEFINE THE NECESSARY ROS TOPICS AND SERVICES
        self.model_namespace = "eboat"
        self.boomAng_pub = rospy.Publisher(f"/{self.model_namespace}/control_interface/sail", Float32, queue_size=5)
        self.rudderAng_pub = rospy.Publisher(f"/{self.model_namespace}/control_interface/rudder", Float32, queue_size=5)
        self.propVel_pub = rospy.Publisher(f"/{self.model_namespace}/control_interface/propulsion", Int16, queue_size=5)
        self.wind_pub = rospy.Publisher(f"/eboat/atmosferic_control/wind", Point, queue_size=5)

        # -->SERACH FOR THE SDF FILE DESCRIBING THE ISLAND
        HOME = os.path.expanduser('~')
        files = glob.glob(os.path.join(HOME, f"**/*Yara_OVE/**/*sand_island_0/model.sdf"), recursive=True)
        if len(files) > 0:
            sdffilepath = files[0]
            del (files)
        else:
            raise IOError(f"File wayPointMarker/model.sdf does not exist")

        # -->SPAWN THE WAYPOINT MARKER IN THE GAZEBO SIMULATION
        self.spawn_sdf = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        ipose = Pose()
        ipose.position.x = 250.0
        ipose.position.y = 0.0
        ipose.position.z = 0.0
        self.waypoint_namespace = f"wayPointMarker"
        with open(sdffilepath, "r") as f:
            sdffile = f.read()
            try:
                result = self.spawn_sdf(model_name=self.waypoint_namespace,
                                        model_xml=sdffile,
                                        robot_namespace=self.waypoint_namespace,
                                        initial_pose=ipose,
                                        reference_frame="world")
            except rospy.ServiceException:
                result = "/gazebo/SpawnModel service call failed"
            print(f"\n\n===========================\n{result}\n===========================\n")

        obsData = None
        while obsData is None:
            try:
                obsData = rospy.wait_for_message(f'/{self.model_namespace}/mission_control/observations',
                                                 Float32MultiArray,
                                                 timeout=10).data
            except:
                pass

    def getObservations(self):
        obsData = None
        while obsData is None:
            try:
                obsData = rospy.wait_for_message(f'/{self.model_namespace}/mission_control/observations',
                                                 Float32MultiArray,
                                                 timeout=20).data
            except:
                pass
            # --> obsData: 0 distance from the goal,
            #              1 angle between the foward direction and the direction towards the goal
            #              2 surge velocity
            #              3 apparent wind speed,
            #              4 apparent wind angle,
            #              5 boom angle,
            #              6 rudder angle,
            #              7 eletric propultion power,
            #              8 roll angle
            #             10 boat's current X position
            #             11 boat's current Y position

        return np.array(obsData, dtype=float)

    def scene1(self):
        self.wind_pub.publish(Point(-7, 0, 0))

        self.propVel_pub.publish(1)
        self.rudderAng_pub.publish(20)
        obs = self.getObservations()
        # while abs(obs[4]) > 150:
        #     obs = self.getObservations()
        #     print(f"Ângulo do vento    : {obs[4]}")
        #     print(f"Velocidade de surge: {obs[2]}")
        #     print(obs[1])
        # self.rudderAng_pub.publish(0.0)
        # self.propVel_pub.publish(0)
        # self.boomAng_pub.publish(15)
        #
        # while obs[2] < 2.7:
        #     obs = self.getObservations()
        #     print(f"Ângulo do vento    : {obs[4]}")
        #     print(f"Velocidade de surge: {obs[2]}")
        #     print(obs[1])
        # self.rudderAng_pub.publish(-30)
        #
        while abs(obs[1] < 15):
            obs = self.getObservations()
            print(f"Ângulo do vento    : {obs[4]}")
            print(f"Velocidade de surge: {obs[2]}")
            print(obs[1])
        self.rudderAng_pub.publish(10.0)
        self.boomAng_pub.publish(10)
        while abs(obs[1] < 30):
            obs = self.getObservations()
            print(f"Ângulo do vento    : {obs[4]}")
            print(f"Velocidade de surge: {obs[2]}")
            print(obs[1])
        self.rudderAng_pub.publish(0.0)
        self.propVel_pub.publish(0)
        self.boomAng_pub.publish(15)

        while abs(obs[1] < 80):
            obs = self.getObservations()
            print(f"Ângulo do vento    : {obs[4]}")
            print(f"Velocidade de surge: {obs[2]}")
            print(obs[1])
        self.rudderAng_pub.publish(-25)
        self.boomAng_pub.publish(10)

        obs = self.getObservations()
        self.rudderAng_pub.publish(0.0)

        for i in range(10):
            obs = self.getObservations()
            print(f"Ângulo do vento    : {obs[4]}")
            print(f"Velocidade de surge: {obs[2]}")
            print(obs[1])

    def script1(self):
        pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        # -->PAUSE SIMULATION
        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            pause()
        except(rospy.ServiceException) as e:
            print(("/gazebo/pause_physics service call failed!"))

        _ = input("press any key to proceed")
        self.wind_pub.publish(Point(0, 6.5, 0))
        # obs = self.getObservations()
        self.boomAng_pub.publish(45.0)



    def close(self):
        ppid = self._roslaunch.pid
        print(f"\n\n===================\nProcess id: {ppid}\n===================\n")
        os.system(f'ps -au eduardo | grep {self._roslaunch.pid}')
        os.killpg(os.getpgid(self._roslaunch.pid), signal.SIGTERM)

        print("\n\n\nCLOSE FUNCTION\n\n")
def main():
    myscene = scene()
    # myscene.scene1()
    myscene.script1()
    _ = input("press any key to end simulation")
    myscene.close()

if __name__ == "__main__":
    main()