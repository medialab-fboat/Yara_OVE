#!/home/eduardo/miniconda3/envs/esailor/bin/python

__author__  = 'Eduardo Charles Vasconcellos <evasconcellos at id.uff.br>'
__version__ = '0.0'
__licence__ = 'Apache2'

import cv2
import sys
import time
import numpy as np

from scipy.ndimage import  filters

#-->ROS libraries
import roslib
import rospy

#-->ROS messages
from sensor_msgs.msg import Image, CompressedImage

VERBOSE = False

from std_msgs.msg import String


def callback(ros_data):
    np_arr = np.fromstring(ros_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:

    cv2.imshow('bow_camera', image_np)
    cv2.waitKey(2)


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('boat_camera', anonymous=True)

    rospy.Subscriber("/mission_control/bow_camera/image_raw/compressed", CompressedImage, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()