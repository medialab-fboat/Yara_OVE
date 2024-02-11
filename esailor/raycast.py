#!/home/eduardo/miniconda3/envs/esailor/bin/python

import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan

class rays():
    def __init__(self):
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("/eboat/laser/scan", LaserScan, self._laser_scan_callback)

        self.laser_scan = np.zeros(5, dtype=int)
        rospy.logdebug("Waiting for /scan to be READY...")
        while ((self.laser_scan is None) and (not rospy.is_shutdown())):
            try:
                self.laser_scan = rospy.wait_for_message("/eboat/laser/scan", LaserScan, timeout=1.0)
                rospy.logdebug("Current /eboat/laser/scan READY=>")
            except:
                rospy.logerr("Current /eboat/laser/scan not ready yet, retrying for getting laser_scan")

    def _laser_scan_callback(self, data):
        laser_ranges = np.asarray(data.ranges)
        laser_ranges[laser_ranges == np.inf] = data.range_max

        self.laser_scan[4] = np.min(laser_ranges[0:23])
        self.laser_scan[3] = np.min(laser_ranges[24:47])
        self.laser_scan[2] = np.min(laser_ranges[48:72])
        self.laser_scan[1] = np.min(laser_ranges[73:96])
        self.laser_scan[0] = np.min(laser_ranges[97:120])


    def step(self):
        obsData = None
        while obsData is None:
            try:
                obsData = rospy.wait_for_message('/eboat/mission_control/observations',
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

            print(np.array(obsData, dtype=float))
            print("-------")
            print(self.laser_scan)
            print("============")

        return np.array(obsData, dtype=float)

if __name__ == '__main__':
    test = rays()
    for i in range(2):
        test.step()