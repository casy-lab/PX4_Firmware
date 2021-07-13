# PX4 Firmware Extension for "UAV-Based Search and Rescue in Avalanches using ARVA: An Extremum Seeking Approach"
In the context of the european project [AirBorne](https://www.airborne-project.eu/), this work deals with the problem of localizing a victim buried by an avalanche by means of a drone equipped with an ARVA (Appareil de Recherche de Victimes d'Avalanche) sensor. The proposed control solution is based on three main units: the ARVA measurement conditioning unit, the Extremum Seeking (ES)-based reference position generator unit, and the Internal Model-Based (IMB) low-level position control unit. This control strategy is shown to succeed in steering the drone in a neighborhood of the victim position. The effectiveness and robustness of the proposed algorithm has been tested by using the code provided in this repo. In particular, a modified version of the PX4 autopilot implementing the aforementioned ES unit and IMB unit has been developed. Moreover, a new Gazebo plugin has been implemented to simulate the behavior of the ARVA unit.

The core files with the implementation of the three units can be found in the following.
* ES reference position generator unit: https://github.com/casy-lab/PX4_Firmware/tree/branch/airborne/src/modules/extremum_seeking
* IMB low-level control unit: https://github.com/casy-lab/PX4_Firmware/tree/branch/airborne/src/modules/mc_att_control/IMBControl
* ARVA measurement conditioning unit: https://github.com/casy-lab/sitl_gazebo/blob/7d7b4af2a106626ff2d1b6bf38c0151bb2c50f04/src/gazebo_arva_plugin.cpp

## Authors
  * Ilario Antonio Azzollini - PhD Student
    * Email: ilario.azzollini@unibo.it
  * Nicola Mimmo - Junior Assistant Professor
    * Email: nicola.mimmmo2@unibo.it
  * Lorenzo Gentilini - PhD Student
    * Email: lorenzo.gentilini6@unibo.it
  * Lorenzo Marconi - Full Professor
    * Email: lorenzo.marconi@unibo.it

## References
For the details of the work, please refer to the papers:
* [1] I.A. Azzollini, N. Mimmo, and L. Marconi. **An Extremum Seeking Approach to Search and Rescue Operations in Avalanches using ARVA**. IFAC-PapersOnLine, 53(2):1627-1632, 2020. 21th IFAC World Congress. ([Paper](https://www.sciencedirect.com/science/article/abs/pii/S2405896320328706), [BibTex](https://github.com/casy-lab/PX4_Firmware/blob/branch/airborne/support_files/bib_ifac.txt)).
* [2] I.A. Azzollini, N. Mimmo, L. Gentilini, and L. Marconi. **UAV-Based Search and Rescue in Avalanches using ARVA: An Extremum Seeking Approach**. arXiv preprint arXiv:2106.14514, 2021. ([Paper](https://arxiv.org/abs/2106.14514), [BibTex](https://github.com/casy-lab/PX4_Firmware/blob/branch/airborne/support_files/bib_arxiv.txt)).

In particular, this repository contains the code related to the complete solution presented in [2], which is an extension of [1].

## Results (see reference [2])
*Results of the software-in-the-loop simulation.*
<img src="https://github.com/casy-lab/PX4_Firmware/blob/branch/airborne/support_files/sitl.png" width = 100% height = 50% />

*Results of the hardware-in-the-loop simulation.*
<img src="https://github.com/casy-lab/PX4_Firmware/blob/branch/airborne/support_files/hitl.png" width = 100% height = 50% />

## 1. System Prerequisites
### 1.1 Ubuntu and ROS
In order to build and run this autopilot, with the simulated ARVA plugin for Gazebo, one of the following options is required:
* Ubuntu 64-bit 16.04 with ROS Kinetic.
* Ubuntu 64-bit 18.04 with ROS Melodic.

Please follow the official [ROS Tutorial](http://wiki.ros.org/ROS/Installation) for installation.

### 1.2 MAVROS and GeographicLib
Install ROS repositories for mavros:
```
sudo apt-get install ros-*ros_distro*-mavros ros-*ros_distro*-mavros-extras
```
Install [GeographicLib](https://geographiclib.sourceforge.io/) dataset by running:
```
cd mavros/mavros/scripts
sudo ./install_geographiclib_datasets.sh
```

### 1.3 Gazebo Simulator
Install Gazebo simulator:
```
sudo apt-get install ros-*ros_distro*-gazebo-ros-pkgs ros-*ros_distro*-gazebo-ros-control
```

## 2. Build The Autopilot
Clone the code from GitHub repository:
```
git clone https://github.com/casy-lab/PX4_Firmware --recursive
```
Then build the code in Software-In-The-Loop (SITL) simulation mode by running:
```
DONT_RUN=1 make px4_sitl_default gazebo
```
If you want build the code for a specific platform, run:
```
DONT_RUN=1 make px4_fmu-v*Your_Specific_Platform*_default
```
In particular, at the moment, the IMB regulator and ES modules are available in Hardware-In-The-Loop (HITL) simulation for the Pixhawk Cube board, so we can build the code by:
```
DONT_RUN=1 make px4_fmu-v3_default
```
To run the autopilot in SITL simulation mode, without ROS interface, type:
```
make px4_sitl gazebo_iris
```
Those commands will launch a Gazebo simulation by spawning an Iris drone model at coordinates: 
```
X = -49.0 m
Y = -49.0 m
Z =  0.0 m
```
The spawned Iris drone is already endowed with the ARVA receiver.
The Gazebo world is shaped to mimic an avalanche scenario, and the ARVA transmitter (victim) is located at coordinates:
```
X = -15.0 m
Y = -25.0 m
Z =  17.0 m
```
and with orientation wrt the inertial frame given by (see [2] for the notation):
```
Y = 10.0°
R = 155.0°
P = 0.0°
```

## 3. Extremum Seeking Example
In order to execute the search mission, in SITL mode, once the simulation has been launched, run (on PX4 console):
```
extremum_seeking start
commander mode search
```
The drone then starts to search the ARVA transmitter by following setpoints provided by the ES unit.

In order to change the transmitter-victim position or orientation, one can modify the header file *Tools/sitl_gazebo/include/gazebo_arva_plugin.h*.
While, to change the drone initial position, please check out the file *Tools/sitl_gazebo/worlds/iris.world*.

## 4. HITL Simulation
Currently, the IMB Regulator and the ES module are implemented only for the Pixhawk Cube board.
In order to test the algorithm in an HITL simulation, first set to *true* the flags *serial_enabled* and *hil_mode* in the file *Tools/sitl_gazebo/models/rotors_description/urdf/iris_base.xacro*. Then, connect the Pixhawk Cube board via USB and execute the following commands:
```
cd ab_es_proj_firmware
make px4_fmu-v3_default upload
```
Once done, launch the Gazebo simulation using:
```
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
gazebo Tools/sitl_gazebo/worlds/iris.world 
```
Finally, launch the QGroundControl application and use the Mavlink interface to start the search mission:
```
extremum_seeking start
commander mode search
```
