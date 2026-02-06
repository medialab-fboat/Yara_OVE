# Yara_OVE
Author: Eduardo Charles Vasconcellos

# YARA Ocean Virtual Environment

#### Gazebo/ROS based virtual environment to reinforcement learning research on Sailing Robots and USVs.
Yara is an Open Source Ocean Virtual Environment for Sailboats and other USVs

## Getting Started
* The instructions assume a basic familiarity with the ROS 1 environment and Gazebo 11.  If these tools are new to you, you can learn the ROS basics by following this [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials)ROS Tutorials
* This environment has been developed and tested on Linux systems.

## Instalation

First of all you should have ROS Noetic, Gazebo 11 and miniconda installed in your system. For installation instructions see:
* http://wiki.ros.org/noetic
* https://classic.gazebosim.org/tutorials?cat=install.
* https://docs.conda.io/en/latest/miniconda.html

Once you have ROS and Gazebo installed, to make the environment run you will need to do:

1 - Create a workspace for the project

> $ ```mkdir -p ~/yara_ws/src```

2 - Change to the src directory

> $ ```cd ~/yara_ws/src```

3 - Clone de repository

> $```git clone https://github.com/medialab-fboat/Yara_OVE.git```

4 - Change to the workspace directory

> $ ```cd ~/yara_ws```

5 - Compile the source

> $ ```catkin_make```

6 - Source de ROS package

> $ ```source ~/yara_ws/devel/setup.bash```

6.1 - Alternatively, you can insert the command above in your .bashrc file

> $ ```echo "source ${HOME}/yara_ws/devel/setup.bash" > ${HOME}/.bashrc```

7 - Test it by launching YARA OVE
> $ ```roslaunch eboat_gazebo ocean.launch```

* If the Gazebo GUI is displayed and you can see the boat floating, everything works, and you are good to go! If not, don't hesitate to contact us and let us know if we can help you.

8 - Create an anaconda environment using the YML file in ~/yara_ws/src/eboat_gz_1/

> $ ```conda env create --file ${HOME}/yara_ws/src/eboat_gz_1/esailor.yml```

9 - Run the setup.py script

> $ ```python3 setup.py```

---

### Launching YARA OVE

**OBS 1:** The YARA OVE has to sailboat models: the EBoat and the Fortune612. The EBoat is the digital twin of the UFF/UFRN sailing robot F-Boat; it is a sailing robot 2.5 meters long and has a single sail and an electric propeller. The Fortune 612 is based on an RC sailing boat; it is 0.99 meters long and has a main sail and a jib sail controlled by a single cable.

\\

1 - Launching YARA OVE with EBoat
> $ ```roslaunch eboat_gazebo ocean.launch```

2 - Launching YARA OVE with Fortune 612
> $ ```roslaunch forutne612_gazebo ocean.launch```

---

\\

**OBS 2:** This project contains some gym environments predifined. You can use them or create your own environments for your RL tasks. The environments are stored in ~/yara_ws/src/eboat_gz_1/eboat_gym_gaz/envs/

\\
**OBS 3:** The original YML file will install tensorflow-gpu and pythorch-cuda. In case you do not have a compatible GPU available, replace them by the CPU version of each one.

How to cite this Yara: \\
Vasconcellos, Eduardo Charles, et al. "Yara: An Ocean Virtual Environment for Research and Development of Autonomous Sailing Robots and Other Unmanned Surface Vessels." Journal of Intelligent & Robotic Systems 111.3 (2025): 1-20.
