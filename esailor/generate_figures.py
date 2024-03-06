#!/home/eduardo/miniconda3/envs/esailor/bin/python

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def mission():
    ppo = pd.read_csv("PPO_mission_2.csv", sep=";")
    sac = pd.read_csv("SAC_mission_2.csv", sep=";")
    pid = pd.read_csv("pid_mission_2.csv", sep=";")

    fig1 = plt.figure(figsize=[8, 6])
    y = ppo.X.values
    x = ppo.Y.values * (-1)
    plt.plot(x, y, color="tab:blue", label="PPO")
    y = sac.X.values
    x = sac.Y.values * (-1)
    plt.plot(x, y, color="tab:orange", label="SAC")
    y = pid.X.values
    x = pid.Y.values * (-1)
    plt.plot(x, y, color="tab:red", label="PID")

    # -->PATH PLANNING FOR THE TEST MISSION
    baseDist = 100
    baseVec = np.array([1.0, 0.0])
    path2follow = [baseVec * baseDist]
    thetaVec = [-45, -90, -135, -180, 45, 135]
    D = [baseDist, baseDist, baseDist, baseDist, baseDist, baseDist]
    for i, theta in enumerate(thetaVec):
        rad = theta * np.pi / 180.0
        rot = np.array([[np.cos(rad), -np.sin(rad)], [np.sin(rad), np.cos(rad)]])
        path2follow.append(path2follow[-1] + np.dot((baseVec * D[i]), rot))
    for i, waypoint in enumerate(path2follow):
        if i > (len(path2follow) - 2):
            plt.plot(-waypoint[1], waypoint[0], 'X', color="black", markersize="11")
            plt.text(-waypoint[1] - 5, waypoint[0] - 10, "END")
        else:
            plt.plot(-waypoint[1], waypoint[0], 'o', color="black", markersize="10")
            plt.text(-waypoint[1], waypoint[0] + 5, f"{i}")

    plt.plot(0, 0, '^', color="black", markersize="11")
    plt.text(-10, -10, "START")

    # plt.xlim(-410, 100)
    # plt.ylim(-100, 410)
    plt.legend()
    plt.xlabel("Distance in meters", fontsize=13)
    plt.xticks(ticks = np.arange(-250, 1, 50),
               labels = [f"{i}" for i in np.arange(250, -1, -50)], fontsize=11)
    plt.ylabel("Distance in meters", fontsize=13)
    plt.yticks(fontsize=11)
    plt.grid()
    plt.arrow(x=-36, y=-10, dx=0, dy=25, width=1.0, head_width=3, ec="black", fc="black")
    plt.arrow(x=-40, y=-10, dx=0, dy=25, width=1.0, head_width=3, ec="black", fc="black")
    plt.arrow(x=-44, y=-10, dx=0, dy=25, width=1.0, head_width=3, ec="black", fc="black")
    plt.text(-40, 24, "WIND", ha="center")
    plt.tight_layout()
    #--------------------------------------------------------------------------------------------------------------
    fig2 = plt.figure(figsize=[8,6])
    x = sac.index.values * 2
    y = sac.surge.values
    plt.plot(x, y, color="tab:orange", linestyle=":", label="SAC")
    plt.plot(x[-1], y[-1], marker="o", color="tab:orange")
    plt.text(x[-1], y[-1] - 0.1, f"{x[-1]}s", color="tab:orange", fontsize=10)
    #---
    x = pid.index.values * 2
    y = pid.surge.values
    plt.plot(x, y, color="tab:red", linestyle="-", label="PID")
    plt.plot(x[-1], y[-1], marker="o", color="tab:red")
    plt.text(x[-1]+4, y[-1]+0.05, f"{x[-1]}s", color="tab:red", fontsize=10)
    #---
    x = ppo.index.values * 2
    y = ppo.surge.values
    plt.plot(x, y, color="tab:blue", linestyle="-", label="PPO")
    plt.plot(x[-1], y[-1], marker="o", color="tab:blue")
    plt.text(x[-1]+5, y[-1] - 0.1, f"{x[-1]}s", color="tab:blue", fontsize=10)
    plt.legend()
    plt.xlabel("Time in seconds", fontsize=13)
    plt.xticks(fontsize=11)
    plt.ylabel("E-Boat's surge velocity in m/s", fontsize=13)
    plt.yticks(fontsize=11)
    plt.grid()
    plt.text(x=390, y=2.95, s="true wind = 6.17 m/s (12 knots)", fontsize=10)
    plt.tight_layout()
    print(x[154]*2)
    #--------------------------------------------------------------------------------------------------------------
    observations = ["rudderAng", "boomAng", "propPwr"]
    figs = []
    for obs in observations:
        figs.append(plt.figure(figsize=[8, 6]))
        x = sac.index.values * 2
        y = sac[obs].values
        plt.plot(x, y, color="tab:orange", linestyle=":", label="SAC")
        plt.plot(x[-1], y[-1], marker="o", color="tab:orange")
        plt.text(x[-1], y[-1] - 0.1, f"{x[-1]}s", color="tab:orange", fontsize=10)
        #---
        x = pid.index.values * 2
        y = pid[obs].values
        plt.plot(x, y, color="tab:red", linestyle="-", label="PID")
        plt.plot(x[-1], y[-1], marker="o", color="tab:red")
        plt.text(x[-1]+4, y[-1]+0.05, f"{x[-1]}s", color="tab:red", fontsize=10)
        #---
        x = ppo.index.values * 2
        y = ppo[obs].values
        plt.plot(x, y, color="tab:blue", linestyle="-", label="PPO")
        plt.plot(x[-1], y[-1], marker="o", color="tab:blue")
        plt.text(x[-1]+5, y[-1] - 0.1, f"{x[-1]}s", color="tab:blue", fontsize=10)
        #---
        plt.legend()
        plt.xlabel("Time in seconds", fontsize=13)
        plt.xticks(fontsize=11)
        if obs == "rudderAng":
            ylabel = "Rudder angle"
        elif obs == "boomAng":
            ylabel = "Boom angle (Sail)"
        else:
            ylabel = "Electric propeller"
        plt.ylabel(ylabel, fontsize=13)
        plt.yticks(fontsize=11)
        plt.grid()
        plt.tight_layout()
    print(ppo.columns)
    # --------------------------------------------------------------------------------------------------------------
    fig1.savefig(f"/home/eduardo/FBoat/Fig27a.png", transparent=True, format="png")
    fig2.savefig(f"/home/eduardo/FBoat/Fig27b.png", transparent=True, format="png")
    letra = ["c", "d", "e"]
    for i, fig in enumerate(figs):
        fig.savefig(f"/home/eduardo/FBoat/Fig27{letra[i]}.png", transparent=True, format="png")

    plt.show()

def returnPerEpisode():
    fig = plt.figure(figsize=[8,8])
    x = np.arange(0, 245*2048 + 1, 100)
    y = np.full(x.shape[0], fill_value=1.8)
    plt.plot(x, y, color="tab:red", linestyle="-.", label="expected")
    ppo = pd.read_csv("./logs/PPO_esailor_93_A3232_C3232_03032024_return.csv")
    sac = pd.read_csv("./logs/SAC_esailor_93_A3232_C3232_03032024_return.csv")
    x = ppo.Step.values
    y = ppo.Value.values
    print(x)
    plt.plot(x[9:], y[9:], color="tab:blue", linestyle="-", label="PPO")
    x = sac.Step.values
    y = sac.Value.values
    plt.plot(x[9:], y[9:], color="tab:orange", linestyle="-", label="SAC")
    xticks  = np.arange(0, 245*2048, 100000)
    xlabels = ["0"] + [f"{i/1000}k" for i in xticks[1:]]
    plt.xlabel("Steps", fontsize=13)
    plt.ylabel("Average return per epiode", fontsize=13)
    plt.xticks(xticks, xlabels, fontsize=11)
    plt.yticks(np.arange(0.6, 1.9, 0.2), fontsize=11)
    plt.legend(fontsize=11)
    plt.grid()
    plt.tight_layout()
    plt.savefig(f"/home/eduardo/FBoat/Fig26a.png", transparent=True, format="png")
    plt.show()

def meanEpisodeLength():
    fig = plt.figure(figsize=[8, 8])
    # x = np.arange(0, 245 * 2048 + 1, 100)
    # y = np.full(x.shape[0], fill_value=55)
    # plt.plot(x, y, color="tab:red", linestyle="-.", label="expected")
    ppo = pd.read_csv("./logs/PPO_esailor_93_A3232_C3232_03032024_episode.csv")
    sac = pd.read_csv("./logs/SAC_esailor_93_A3232_C3232_03032024_episode.csv")
    x = ppo.Step.values
    y = ppo.Value.values / 50.0
    print(x)
    plt.plot(x[9:], y[9:], color="tab:blue", linestyle="-", label="PPO")
    x = sac.Step.values
    y = sac.Value.values / 50.0
    plt.plot(x[9:], y[9:], color="tab:orange", linestyle="-", label="SAC")
    xticks = np.arange(0, 245 * 2048, 100000)
    xlabels = ["0"] + [f"{i / 1000}k" for i in xticks[1:]]
    plt.xlabel("Steps", fontsize=13)
    plt.ylabel("A.S.E / M.E.S.E", fontsize=13)
    plt.xticks(xticks, xlabels, fontsize=11)
    plt.yticks(fontsize=11)
    plt.legend(fontsize=11)
    plt.grid()
    plt.tight_layout()
    plt.savefig(f"/home/eduardo/FBoat/Fig26b.png", transparent=True, format="png")
    plt.show()

if __name__ == "__main__":
    # returnPerEpisode()
    # meanEpisodeLength()
    mission()