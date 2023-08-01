#!/home/araujo/miniconda3/envs/esailor/bin/python

import sys
import subprocess

import pygame.joystick
import rospy
import time
import numpy as np

import matplotlib.pyplot as plt

from sensor_msgs.msg import Joy

from std_msgs.msg import Int16, Float32

if __name__ == '__main__':
    pygame.joystick.init()
    angVel = 4.0 * 20e-1
    d2r     = np.pi / 180.0

    ################################
    rospy.init_node('Control_Interface', anonymous=True)
    bang_pub = rospy.Publisher('/boto/control_interface/sail', Float32, queue_size=20)
    rang_pub = rospy.Publisher('/boto/control_interface/rudder', Float32, queue_size=20)
    ################################

    plt.ion()

    fig = plt.figure(figsize=[6, 8])
    img = plt.imread(
        "/home/araujo/eboat_ws/src/eboat_gz_1/eboat_control/python/projects/ESailor/boat2_top_view.png")
    plt.imshow(img)
    plt.axis('off')
    fig.subplots_adjust(left=0.225, bottom=0.125, right=1.0 - 0.225, top=0.83, wspace=0.2, hspace=0.2)
    # --> AXIS 0
    ax0 = fig.add_subplot(3, 1, 1, projection='polar')
    ax0.set_theta_zero_location(loc='N', offset=0.0)
    ax0.set_xticks(np.pi / 180. * np.arange(180, -180, -30), labels=[])
    ax0.set_thetalim(-np.pi, np.pi)
    ax0.set_rgrids(radii=[], labels=[])
    ax0.scatter([5 * np.pi / 180.0, 185.0 * np.pi / 180.0], [1, 1], c='white')
    ax0.patch.set_alpha(0.0)
    ax0.set_position(pos=[0.399, 0.7, 0.2, 0.2], which='original')
    jib, = ax0.plot([0, np.pi], [0, 1], color='tab:orange', linestyle='-', linewidth=3)

    # -->AXIS 1
    ax1 = fig.add_subplot(3, 1, 2, projection='polar')
    ax1.set_theta_zero_location(loc='N', offset=0.0)
    ax1.set_xticks(np.pi / 180. * np.arange(180, -180, -30))
    ax1.set_thetalim(-np.pi, np.pi)
    ax1.set_rgrids(radii=[], labels=[])
    ax1.scatter([5 * np.pi / 180.0, 185.0 * np.pi / 180.0], [1, 1], c='white')
    ax1.patch.set_alpha(0.0)
    ax1.set_position(pos=[0.248, 0.253, 0.5, 0.5], which='original')
    sail, = ax1.plot([0, np.pi], [0, 1], color='tab:orange', linestyle='-', linewidth=3)

    # -->AXIS2
    ax2 = fig.add_subplot(3, 1, 3, projection='polar')
    ax2.set_theta_zero_location(loc='N', offset=0.0)
    ax2.set_xticks(np.pi / 180. * np.arange(180, -180, -30))
    ax2.set_thetalim(-np.pi, np.pi)
    ax2.set_rgrids(radii=[], labels=[])
    ax2.patch.set_alpha(0.0)
    ax2.set_position(pos=[0.375, 0.014, 0.25, 0.25], which='original')
    rudder1, = ax2.plot([0, 0], [0, 1], color="tab:green", linestyle="-", linewidth=2)
    rudder2, = ax2.plot([0, np.pi], [0, 1], color="tab:green", linestyle="-.", linewidth=2)

    surge_vel = fig.text(x=0.73, y=0.3, s="Surge vel: {:4.2f}".format(0.0), fontsize=12)

    fig.canvas.draw()
    fig.canvas.flush_events()

    t2 = np.pi
    r2 = 0.0
    count = 0
    while True:
        joy2sail = pygame.joystick.Joystick(0).get_axis(1)
        joy2rudder = pygame.joystick.Joystick(0).get_axis(2)
        # print(f"joy axis = {joy2sail}")
        t2 += joy2sail * angVel * d2r
        if t2 < 100 * d2r:
            t2 = 100 * d2r
            print("entrou aqui")
        elif t2 > np.pi:
            t2 = np.pi
        else:
            pass
        # if (t2 >= 100 * d2r) and (t2 <= np.pi):
        # print(f"t2 = {t2 * 180.0 / np.pi} / {180.0 - t2 * 180.0 / np.pi}")
        if not rospy.is_shutdown():
            bang_pub.publish(180.0 * (1 - t2 / np.pi))
        
        # -->RUDDER ANGLE
        r2 += joy2rudder * angVel * d2r
        if r2 < -30.0 * d2r:
            r2 = -30.0 * d2r
        elif r2 > 30.0 * d2r:
            r2 = 30.0 * d2r

        if not rospy.is_shutdown():
            rang_pub.publish(r2 * 180.0 / np.pi)
        
        sail.remove()
        jib.remove()
        sail, = ax1.plot([0, t2], [0, 1], color='tab:orange', linestyle='-', linewidth=3)
        jib,  = ax0.plot([0, t2], [0, 1], color='tab:orange', linestyle='-', linewidth=3)

        rudder1.remove()
        rudder2.remove()
        rudder1, = ax2.plot([0, r2], [0, 1], color="tab:green", linestyle="-", linewidth=2)
        rudder2, = ax2.plot([0, r2 + np.pi], [0, 1], color="tab:green", linestyle="-.", linewidth=2)

        fig.canvas.draw()
        fig.canvas.flush_events()

        time.sleep(0.1)