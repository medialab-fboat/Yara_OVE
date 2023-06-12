#!/home/eduardo/miniconda3/envs/gazgym/bin/python

from time import sleep
import rospy
from std_msgs.msg import Int16

def sailor():
    pub = rospy.Publisher('/eboat/control_interface/propulsion', Int16, queue_size=10)
    rospy.init_node('Sailor', anonymous=True)
    rate = rospy.Rate(1) # 1 Hz

    while not rospy.is_shutdown():
        val = 1
        rospy.loginfo(val)
        pub.publish(val)
        rate.sleep()

if __name__ == '__main__':
    try:
        sailor()
    except rospy.ROSInterruptException:
        pass