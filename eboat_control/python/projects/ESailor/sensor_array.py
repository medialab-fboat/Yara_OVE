#!/home/araujo/miniconda3/envs/esailor/bin/python

import numpy as np
import rospy, rosnode, rosgraph
import sys, os, signal
import subprocess
import time
import random

import matplotlib.pyplot as plt

from std_msgs.msg import Float32MultiArray

class Visualizer():
    def __init__(self):
        # to run GUI event loop
        plt.ion()

        # self.fig = plt.figure(figsize=[5,6])
        self.fig = plt.figure(figsize=[5, 7])
        img = plt.imread("/home/araujo/eboat_ws/src/eboat_gz_1/eboat_control/python/projects/ESailor/boat_top_view.png")
        plt.imshow(img)
        plt.axis('off')
        # -->AXIS 1
        self.ax = self.fig.add_subplot(2, 1, 1, projection='polar')
        self.ax.set_theta_zero_location(loc = 'N', offset = 0.0)
        # self.ax.set_xticks(np.pi / 180. * np.arange(180, -180, -20))
        self.ax.set_xticks(np.pi / 180. * np.arange(180, -180, -30))
        self.ax.set_thetalim(-np.pi, np.pi)
        self.ax.set_rgrids(radii=[], labels=[])
        self.ax.scatter([5*np.pi/180.0, 185.0*np.pi/180.0], [1, 1], c='white')
        self.ax.patch.set_alpha(0.0)
        # self.fig.subplots_adjust(left=0.125, bottom=0.03, right=0.9, top=0.83, wspace=0.2, hspace=0.2)
        self.fig.subplots_adjust(left=0.225, bottom=0.125, right=1.0-0.225, top=0.83, wspace=0.2, hspace=0.2)
        # self.ax.set_position(pos=[0.113, 0.194, 0.8, 0.8], which='original')
        self.ax.set_position(pos=[0.126, 0.247, 0.75, 0.75], which='original')

        self.sail, = self.ax.plot([0,np.pi], [0, 1], color='tab:orange', linestyle=':', linewidth=3)

        theta  = np.pi
        self.r = 0.4
        self.u = 1
        self.v = 1
        # self.apwind = self.ax.quiver(theta, self.r, self.u, self.v, color='blue', angles=90, pivot='tip', width=0.02, scale=10)
        self.apwind, = self.ax.plot(0, 0.8, color='blue', marker='o', markersize=16)

        self.angs  = np.arange(170, 191, 2) * np.pi / 180.0
        self.radis = np.full(self.angs.shape[0], fill_value=1)
        self.prop = [self.ax.plot(self.angs, self.radis, color = 'gray',linestyle='--', linewidth=2)[0]]

        # -->TRAJECTORY ANGLE
        self.traj, = self.ax.plot(0, 1, marker="*", markersize=14, color="red")

        # -->AXIS2
        self.ax2 = self.fig.add_subplot(2, 1, 2, projection='polar')
        self.ax2.set_theta_zero_location(loc = 'N', offset = 0.0)
        self.ax2.set_xticks(np.pi / 180. * np.arange(180, -180, -30))
        self.ax2.set_thetalim(-np.pi, np.pi)
        self.ax2.set_rgrids(radii=[], labels=[])
        self.ax2.patch.set_alpha(0.0)
        self.ax2.set_position(pos=[0.351, 0.014, 0.3, 0.3], which='original')
        self.rudder1, = self.ax2.plot([0, 0], [0, 1], color="tab:green", linestyle="-", linewidth=2)
        self.rudder2, = self.ax2.plot([0, np.pi], [0, 1], color="tab:green", linestyle="-.", linewidth=2)

        self.surge_vel = self.fig.text(x=0.73, y=0.3, s="Surge vel: {:4.2f}".format(0.0),fontsize=12)


    def update(self, obs):
        if obs[3] < 0.5:
            self.apwind.remove()
            self.apwind, = self.ax.plot(0, 0.8, color='blue', marker='o', markersize=16)

            self.sail.remove()
            t2 = np.pi + obs[5] * np.pi / 180.0
            self.sail, = self.ax.plot([0, t2], [0, 1], color='tab:orange', linestyle=':', linewidth=3)
        else:
            self.apwind.remove()
            theta = (obs[4] + 180) * np.pi / 180
            self.apwind = self.ax.quiver(theta, self.r, self.u, self.v, color='blue', angles=(obs[4] + 90), pivot='tip', width=0.02, scale=5)

            self.sail.remove()
            if (theta > np.pi):
                t2 = np.pi - obs[5]*np.pi/180.0
            else:
                t2 = np.pi + obs[5]*np.pi/180.0
            self.sail, = self.ax.plot([0, t2], [0, 1], color='tab:orange', linestyle='--', linewidth=3)

        propvel = int(obs[7])
        for i in range(len(self.prop)):
            self.prop[i].remove()
        if propvel == 0:
            self.prop = [self.ax.plot(self.angs, self.radis, color = 'gray',linestyle='--', linewidth=2)[0]]
        else:
            if propvel > 0:
                colortheme = "tab:green"
            else:
                colortheme = "tab:red"

            self.prop = [self.ax.plot(self.angs, self.radis, color=colortheme, linestyle='-', linewidth=2)[0]]
            for i in range(1, abs(propvel)):
                self.prop.append(self.ax.plot(self.angs, self.radis-0.05*i, color=colortheme, linestyle='-', linewidth=2)[0])

        # -->TRAJECTORY ANGLE
        self.traj.remove()
        self.traj, = self.ax.plot(obs[1]*np.pi/180.0, 1, marker="*", markersize=14, color="red")

        # -->RUDDER ANGLE
        self.rudder1.remove()
        self.rudder2.remove()
        self.rudder1, = self.ax2.plot([0, obs[6] * np.pi / 180.0], [0, 1], color="tab:green", linestyle="-", linewidth=2)
        self.rudder2, = self.ax2.plot([0, (obs[6]+180.0) * np.pi / 180.0], [0, 1], color="tab:green", linestyle="-.", linewidth=2)

        self.surge_vel.remove()
        self.surge_vel = self.fig.text(x=0.73, y=0.3, s="Surge vel: {:4.2f}".format(obs[2]), fontsize=12)



    def callback(self, msg):
        # rospy.loginfo(rospy.get_caller_id() + f"I heard {msg.data[4]}")
        self.update(np.array(msg.data))

if __name__ == '__main__':
    vis = Visualizer()

    rospy.init_node("sensor_array", anonymous=True)
    rospy.Subscriber("/eboat/mission_control/observations", Float32MultiArray, vis.callback)

    plt.show(block=True)
