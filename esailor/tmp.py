#!/home/araujo/miniconda3/envs/esailor2/bin/python

import numpy as np
import matplotlib.pyplot as plt
import os
import glob
import sys
import subprocess
import rospy
import time

from scipy import interpolate

from geometry_msgs.msg import Pose

def mutliplier():
    x = np.arange(1, 61)
    y = 0.15 * (x / 60)
    plt.plot(x,y ,color="tab:blue")
    y = 0.15 * (x / 60)**2
    plt.plot(x, y, color="tab:red")
    y = 0.15 * (x / 60)**4
    plt.plot(x, y, color="tab:orange")
    plt.show()

def waypoints():
    plt.plot(0, 0, marker = 'o', color='tab:blue')
    y = 0
    x = 0
    color = ["blue", "red", "green", "orange", "violet", "cyan"]
    for i, theta in enumerate([90, 30, -30, -130, 180, -90]):
        rad = theta * (np.pi / 180.0)
        y += 100 * np.sin(rad)
        x += 100 * np.cos(rad)
        plt.plot(x, y, marker='x', color=color[i])

    plt.ylim(-300, 300)
    plt.xlim(-300, 300)
    plt.show()

def surgeAccordingWind():
    x = np.array([3, 5, 6, 7, 10])
    y = np.array([40, 21, 18, 16, 12])
    f = interpolate.interp1d(x, y, kind="cubic")

    xnew = np.arange(3, 10, 0.1)
    ynew = f(xnew)

    xnew = np.concatenate([xnew, np.array([10])])
    ynew = np.concatenate([ynew, np.array([12])])

    plt.plot(xnew, ynew)

    x = np.arange(3, 11, 1, dtype=int)
    y = f(x)
    print(x)
    print(x-3)
    for i, val in enumerate(y):
        if (val - np.floor(val)) < 0.5:
            y[i] = np.floor(val)
        else:
            y[i] = np.ceil(val)
    print(y)

    plt.show()

def sailingDirection():
    theta = np.arange(0, 91, 0.1) * np.pi / 180
    func  = np.cos(theta)**3
    plt.plot(theta * 180 / np.pi, func)
    plt.xticks(np.arange(0, 91, 30))
    func = np.cos(theta) ** 5
    plt.plot(theta * 180 / np.pi, func)
    plt.grid()
    plt.show()


def lateralReturnVal():
    y = np.arange(-50, 51)
    x = np.cos(y * np.pi / 100)**9
    plt.plot(y, x)
    plt.grid()
    plt.show()

def liftDragCoefficientsForRudderAndKeel():
    df = pandas.read_csv("/home/araujo/scripts/naca001234_400k.csv")
    A = df.atkang.values
    cl = df.cl.values
    cd = df.cd.values

    fcl = interpolate.interp1d(A, cl, kind="cubic")
    fcd = interpolate.interp1d(A, cd, kind="cubic")

    atkang = np.arange(91)
    newcl = fcl(atkang)
    newcd = fcd(atkang)

    print(newcl)

    # plt.plot(newcl)
    # plt.plot(newcd)
    # plt.show()

def testGetPhysicsProperties(path2launchfile = None):
    port_ros = "11411"
    port_gazebo = "11445"

    # -->EXPORT ADDRESS FOR THE ROS MASTER NODE AND THE GAZEBO SERVER
    os.environ["ROS_MASTER_URI"] = "http://localhost:" + port_ros
    os.environ["GAZEBO_MASTER_URI"] = "http://localhost:" + port_gazebo

    # -->SEARCH FOR A LAUNCH FILE
    HOME = os.path.expanduser('~')
    if path2launchfile == None:
        print(
            "As the user did not provide a viable launch file, I will search for one.\nThis may take a while, pelase wait!")
        files = glob.glob(os.path.join(HOME, "**/*empty_ocean.launch"), recursive=True)
        if len(files) > 0:
            path2launchfile = files[0]
            del (files)
        else:
            path2launchfile = input("\nI did not find a viable launch file!\nPlease provide an valid path:\n")
            if (((len(path2launchfile) > 0) & (not (os.path.exists(path2launchfile)))) | (len(path2launchfile) == 0)):
                raise IOError("File " + path2launchfile + " does not exist")

    # -->LAUNCH THE SIMULATION USING SUBPROCESS
    ros_path = os.path.dirname(subprocess.check_output(["which", "roscore"]))
    roslaunch = subprocess.Popen([sys.executable, os.path.join(ros_path, b"roslaunch"), "-p", port_ros, path2launchfile])

    # -->DEFINE A UNIQUE MODEL NAME FOR OUR BOAT
    modelname = f"eboat4"

    # -->SERACH FOR THE URDF FILE DESCRIBING THE BOAT
    files = glob.glob(os.path.join(HOME, f"**/*Yara_OVE/**/*{modelname}.urdf.xacro"), recursive=True)
    if len(files) > 0:
        urdffilepath = files[0]
        del (files)
    else:
        raise IOError(f"File {modelname}.urdf.xacro does not exist")

    # -->TRANSFORM THE XACRO FILE TO URDF
    os.system(f"xacro {urdffilepath} > {modelname}.urdf")

    # -->SPAWN THE MODEL IN THE GAZEBO SIMULATION
    spawn_urdf       = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
    model_namespace  = "eboat"
    ipose            = Pose()
    ipose.position.x = 0.0
    ipose.position.y = 0.0
    ipose.position.z = 0.0
    count = 0
    spawnflag = "Fail"
    while (spawnflag == "Fail") & (count < 18):
        with open(f"{urdffilepath}.urdf", "r") as f:
            urdffile = f.read()
            try:
                result = spawn_urdf(model_name=model_namespace,
                                    model_xml=urdffile,
                                    robot_namespace=model_namespace,
                                    initial_pose=ipose,
                                    reference_frame="world")
                spawnflag = "Sucess"
            except rospy.ServiceException:
                result = "/gazebo/SpawnModel service call failed"
                count += 1
                time.sleep(5)
    print(f"\n\n===========================\n{result}\n===========================\n")

if __name__ == '__main__':
    # waypoints()
    # surgeAccordingWind()
    # sailingDirection()
    # liftDragCoefficientsForRudderAndKeel()
    # testGetPhysicsProperties()
    lateralReturnVal()

    # import rospy
    # import time
    # from geometry_msgs.msg import Point
    # from std_msgs.msg import Float32, Float32MultiArray
    # try:
    #     status = rospy.init_node(f"teste", anonymous=True)
    #     print(status)
    # except:
    #     print("ROSMASTER is not running!")
    #     print(time.time())
    #     exit(1)
    # pub = rospy.Publisher("/eboat/atmosferic_control/wind", Point, queue_size=50)
    # pub2 = rospy.Publisher("/eboat/control_interface/rudder", Float32, queue_size=5)
    #
    # try:
    #     obsData = rospy.wait_for_message('/eboat/mission_control/observations', Float32MultiArray, timeout=20).data
    # except:
    #     print("Exception!")
    # print(obsData)
    # pub.publish(Point(5.0, 0.0, 0.0))
    # print(pub2.publish(np.float32(30.0)))
    # _ = input()