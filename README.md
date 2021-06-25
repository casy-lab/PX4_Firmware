# PX4 Firmware - IMB & ARVA Sensor Extension
This GitHub repository provides a modified version of the PX4 autopilot which implements a new Internal Model Based (IMB) position controller able to provide null regime tracking errors under periodic excitations. Moreover a new Gazebo plugin has been implemented to simulate the behaviour of an ARVA transceiver systems, and an Extremum Seeking module is used to produce harmonic position trajectories useful to maximize the ARVA function, thus to find out the trasmitter.

## Internal Model Based Regulator
*Results of the software in the loop simulation.*
<img src="https://github.com/casy-lab/PX4_Firmware/blob/branch/airborne/support_files/sitl.png" width = 100% height = 50% />

*Results of the hardware in the loop simulation.*
<img src="https://github.com/casy-lab/PX4_Firmware/blob/branch/airborne/support_files/hitl.png" width = 100% height = 50% />

## Authors
  * Ilario Antonio Azzollini - Phd Student
    * Email: ilario.azzollini@unibo.it
  * Nicola Mimmo - Junior Assistant Professor
    * Email: nicola.mimmmo2@unibo.it
  * Lorenzo Gentilini - Phd Student
    * Email: lorenzo.gentilini6@unibo.it
  * Lorenzo Marconi - Full Professor
    * Email: lorenzo.marconi@unibo.it

## References
*If you use the ARVA plugin or this IMB regulator for your academic research, please cite our related papers.*
* I.A. Azzollini, N. Mimmo, and L. Marconi. **An Extremum Seeking Approach to Search and Rescue Operations in Avalanches using ARVA**. IFAC-PapersOnLine, 53(2):1627-1632, 2020. 21th IFAC World Congress. ([PDF](https://www.sciencedirect.com/science/article/abs/pii/S2405896320328706), [BibTex](https://github.com/casy-lab/PX4_Firmware/blob/branch/airborne/support_files/bib_ifac.txt)).
* **Paper Title**, Authors, Conference/Journal/Date ([PDF](link to pdf), [BibTex](link to bib)).

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
Then build the code in simulation mode by running:
```
cd ab_es_proj_firmware
DONT_RUN=1 make px4_sitl_default gazebo
```
If you want build the code for a specific platform, run:
```
cd ab_es_proj_firmware
DONT_RUN=1 make px4_fmu-v*Your_Specific_Platform*_default
```
Currently IBM regulator and Extremum Seeking module are available in simulation and for Pixhawk Cube:
```
cd ab_es_proj_firmware
DONT_RUN=1 make px4_fmu-v3_default
```
To run the autopilot in simulation mode, without ROS interface, type:
```
cd ab_es_proj_firmware
make px4_sitl gazebo_iris
```
These command will launch a Gazebo simulation by spawning an Iris drone model at coordinates: 
```
X = -49.0 m
Y = -49.0 m
Z =  0.0 m
```
The spawned Iris drone is already endowed with the ARVA receiver.
The Gazebo world is shaped to mimic an avalanche scenario, and an ARVA transmitter is located at coordinates:
```
X = -15.0 m
Y = -25.0 m
Z =  17.0 m
```
and with rotations:
```
Y = 10.0°
R = 155.0°
P = 0.0°
```

## 3. Extremum Seeking Example
In order to execute the search mission, in SITL mode, once the simulation has been launched, run (on px4 console):
```
extremum_seeking start
commander mode search
```
The drone then starts to search the ARVA transmitter by following setpoints provided by the Bounded Update Rate algorithm.

In order to change the transmitter position or orientation, one can modify the header file *Tools/sitl_gazebo/include/gazebo_arva_plugin.h*.
While to change the drone initial position, please check out the file *Tools/sitl_gazebo/worlds/iris.world*.

## 4. Hardware in the Loop Simulation
Currently the IBM Regulator and the Extremum Seeking module are implemented just for Pixhawk Cube board.
In order to test the algorithm with HITL simulations, first set to *true* the flags *serial_enabled* and *hil_mode* in the file *Tools/sitl_gazebo/models/rotors_description/urdf/iris_base.xacro*. Then, connect the Pixhawk Cube board via USB plug and execute the following commands:
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