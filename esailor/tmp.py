#!/home/eduardo/miniconda3/envs/esailor2/bin/python

import numpy as np
import matplotlib.pyplot as plt

from scipy import interpolate

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
    theta = np.arange(180, -180, -0.1) * np.pi / 180
    func  = np.cos(theta)**3
    plt.plot(theta * 180 / np.pi, func)
    plt.xticks(np.arange(-180, 181, 45))
    plt.grid()
    plt.show()

if __name__ == '__main__':
    # waypoints()
    # surgeAccordingWind()
    sailingDirection()
