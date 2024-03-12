#!/home/eduardo/miniconda3/envs/esailor2/bin/python

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
    plt.plot(y, x, color="tab:blue")
    x = x - 1.0
    plt.plot(y, x, color="tab:red")
    x = np.sin(y * np.pi / 100) ** 9
    plt.plot(y, x, color="tab:orange")
    plt.grid()
    plt.show()

def liftDragCoefficientsForRudderAndKeel():
    df = pandas.read_csv("/home/eduardo/USVSim/scripts/naca001234_400k.csv")
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

def liftDragCoefficientsForSail():
    lift_coef = np.array([0.0000, 0.0283, 0.0498, 0.0671, 0.0829, 0.1000, 0.1211, 0.1489, 0.1862, 0.2357, 0.3000, 0.3804, 0.4714,
             0.5663, 0.6582, 0.7400, 0.8069, 0.8616, 0.9089, 0.9535, 1.0000, 1.0518, 1.1066, 1.1604, 1.2095, 1.2500,
             1.2791, 1.2972, 1.3058, 1.3063, 1.3000, 1.2885, 1.2738, 1.2577, 1.2425, 1.2300, 1.2217, 1.2164, 1.2122,
             1.2074, 1.2000, 1.1888, 1.1744, 1.1582, 1.1413, 1.1250, 1.1102, 1.0970, 1.0848, 1.0734, 1.0625, 1.0516,
             1.0404, 1.0284, 1.0151, 1.0000, 0.9829, 0.9640, 0.9436, 0.9222, 0.9000, 0.8776, 0.8556, 0.8348, 0.8160,
             0.8000, 0.7871, 0.7762, 0.7658, 0.7542, 0.7400, 0.7221, 0.7010, 0.6779, 0.6539, 0.6300, 0.6071, 0.5850,
             0.5633, 0.5418, 0.5200, 0.4976, 0.4745, 0.4505, 0.4257, 0.4000, 0.3733, 0.3458, 0.3176, 0.2889, 0.2600,
             0.2310, 0.2021, 0.1738, 0.1463, 0.1200, 0.0950, 0.0710, 0.0475, 0.0240, 0.0000, -0.0325, -0.0644, -0.0958,
             -0.1266, -0.1569, -0.1867, -0.2159, -0.2446, -0.2727, -0.3003, -0.3273, -0.3538, -0.3797, -0.4051, -0.4300,
             -0.4543, -0.4781, -0.5013, -0.5240, -0.5462, -0.5678, -0.5888, -0.6093, -0.6293, -0.6487, -0.6676, -0.6859,
             -0.7037, -0.7210, -0.7377, -0.7539, -0.7695, -0.7846, -0.7991, -0.8131, -0.8265, -0.8394, -0.8518, -0.8636,
             -0.8749, -0.8856, -0.8958, -0.9054, -0.9145, -0.9231, -0.9311, -0.9386, -0.9455, -0.9519, -0.9577, -0.9630,
             -0.9677, -0.9719, -0.9756, -0.9787, -0.9813, -0.9833, -0.9848, -0.9858, -0.9862, -0.9860, -0.9853, -0.9841,
             -0.9823, -0.9800, -0.9771, -0.9737, -0.9698, -0.9653, -0.9603, -0.9547, -0.9486, -0.9419, -0.9347, -0.9269,
             -0.9186, -0.9098, -0.9004, -0.8905, -0.8800])
    drag_coef = np.array([0.0500, 0.0488, 0.0484, 0.0489, 0.0502, 0.0523, 0.0553, 0.0591, 0.0638, 0.0693, 0.0756, 0.0828, 0.0909,
             0.0997, 0.1094, 0.1200, 0.1314, 0.1436, 0.1567, 0.1706, 0.1854, 0.2010, 0.2175, 0.2348, 0.2529, 0.2719,
             0.2917, 0.3123, 0.3338, 0.3562, 0.3794, 0.4028, 0.4260, 0.4488, 0.4714, 0.4937, 0.5156, 0.5373, 0.5587,
             0.5798, 0.6005, 0.6210, 0.6412, 0.6611, 0.6807, 0.7000, 0.7190, 0.7377, 0.7561, 0.7742, 0.7920, 0.8095,
             0.8268, 0.8437, 0.8603, 0.8766, 0.8927, 0.9084, 0.9238, 0.9390, 0.9538, 0.9684, 0.9826, 0.9966, 1.0102,
             1.0236, 1.0367, 1.0494, 1.0619, 1.0741, 1.0859, 1.0975, 1.1088, 1.1198, 1.1303, 1.1404, 1.1501, 1.1593,
             1.1681, 1.1764, 1.1843, 1.1918, 1.1988, 1.2054, 1.2115, 1.2172, 1.2225, 1.2273, 1.2317, 1.2357, 1.2392,
             1.2422, 1.2449, 1.2470, 1.2488, 1.2501, 1.2509, 1.2514, 1.2514, 1.2509, 1.2500, 1.2487, 1.2469, 1.2447,
             1.2420, 1.2389, 1.2354, 1.2314, 1.2270, 1.2221, 1.2168, 1.2111, 1.2049, 1.1983, 1.1912, 1.1837, 1.1758,
             1.1674, 1.1587, 1.1498, 1.1409, 1.1319, 1.1228, 1.1137, 1.1046, 1.0953, 1.0861, 1.0767, 1.0673, 1.0579,
             1.0484, 1.0388, 1.0292, 1.0195, 1.0098, 1.0000, 0.9902, 0.9802, 0.9703, 0.9603, 0.9502, 0.9400, 0.9299,
             0.9196, 0.9093, 0.8989, 0.8885, 0.8780, 0.8675, 0.8569, 0.8462, 0.8355, 0.8248, 0.8139, 0.8031, 0.7921,
             0.7811, 0.7701, 0.7590, 0.7478, 0.7366, 0.7253, 0.7139, 0.7025, 0.6911, 0.6796, 0.6680, 0.6564, 0.6447,
             0.6329, 0.6211, 0.6093, 0.5974, 0.5854, 0.5734, 0.5613, 0.5491, 0.5369, 0.5247, 0.5124, 0.5000])
    fig = plt.figure()
    ax  = fig.add_subplot(111)
    ax.plot(lift_coef, color="tab:blue")
    ax.plot(drag_coef, color="tab:red")
    ax.grid()
    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111)
    x = drag_coef[:101]
    y = lift_coef[:101]
    ax2.plot(x, y, color="tab:blue")
    for i in range(5, 100, 5):
        x = drag_coef[i]
        y = lift_coef[i]
        ax2.plot(x, y, marker = "o", color="tab:blue")
        ax2.text(x, y, i)
    ax2.grid()
    plt.show()

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
    # liftDragCoefficientsForSail()

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