#!/home/araujo/miniconda3/envs/esailor2/bin/python

import numpy as np
import rospy
import time
from std_msgs.msg import Float32, Int16, Float32MultiArray
from geometry_msgs.msg import Point, Pose
from gazebo_msgs.srv import SetModelState, SpawnModel, DeleteModel
import os, subprocess, glob

def main():
    # -->INITIALIZE A ROS NODE FOR THE TRAINING PROCESS
    # try:
    #     rospy.init_node(f"test_physics", anonymous=True)
    # except:
    #     print("ROSMASTER is not running!")
    #     print(time.time())
    #     exit(1)

    rospy.init_node('model_testing', anonymous=True)

    # -->SERACH FOR THE URDF FILE DESCRIBING THE BOAT
    modelname = "eboat4tr"
    HOME = os.path.expanduser('~')
    files = glob.glob(os.path.join(HOME, f"**/*Yara_OVE/**/*{modelname}.urdf.xacro"), recursive=True)
    if len(files) > 0:
        urdffilepath = files[0]
        del (files)
    else:
        raise IOError(f"File {modelname}.urdf.xacro does not exist")

    # -->TRANSFORM THE XACRO FILE TO URDF
    # subprocess.run(["xacro", urdffilepath, f"{modelname}.urdf"], capture_output=True)
    os.system(f"xacro {urdffilepath} > {modelname}.urdf")

    # -->SPAWN THE MODEL IN THE GAZEBO SIMULATION
    spawn_urdf = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
    ipose = Pose()
    ipose.position.x = 0.0
    ipose.position.y = 0.0
    ipose.position.z = 0.0
    model_namespace = "eboat"
    count = 0
    spawnflag = "Fail"
    while (spawnflag == "Fail") & (count < 18):
        with open(f"{modelname}.urdf", "r") as f:
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

    boomAng_pub = rospy.Publisher("/eboat/control_interface/sail", Float32, queue_size=1)
    rudderAng_pub = rospy.Publisher("/eboat/control_interface/rudder", Float32, queue_size=5)
    propVel_pub = rospy.Publisher("/eboat/control_interface/propulsion", Int16, queue_size=5)
    wind_pub = rospy.Publisher("/eboat/atmosferic_control/wind", Point, queue_size=5)

    obsData = None
    try:
        obsData = rospy.wait_for_message("/eboat/mission_control/observations", Float32MultiArray,
                                         timeout=20).data
    except:
        pass
    print(obsData, "\n")

    val = np.array([0, 10, -20, 30, -40], dtype=np.float32)
    rudderAng_pub.publish(np.float32(0))
    time.sleep(2)
    propVel_pub.publish(3)


if __name__ == "__main__":
    main()