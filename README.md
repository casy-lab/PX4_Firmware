# PX4 Autopilot - IMB Regulator
This GitHub repository provides a modified version of PX4 autopilot which implements an Internal Model Based (IMB) position controller able to provide null regime tracking error under periodic excitations. Moreover a new Gazebo plugin has been implemented to simulate the behaviour of an ARVA transceiver systems, and an Extremum Seeking module is used to produce harmonic position trajectories useful to maximize the ARVA function, thus to find out the trasmitter.

## Internal Model Based Regulator
<img src="https://github.com/lorenzogentilini/ab_es_proj_firmware/blob/devel/lorenzo/support_files/positions.png" width = 60% height = 40% />

## ARVA Simulator
<img src="https://github.com/lorenzogentilini/ab_es_proj_firmware/blob/devel/lorenzo/support_files/arva.png" width = 60% height = 40% />

## Bounded Update Rate Extremum Seeking

## Authors
  * Ilario Antonio Azzollini - Phd Student
    * Email: ilario.azzollini@unibo.it
  * Lorenzo Gentilini - Phd Student
    * Email: lorenzo.gentilini6@unibo.it
  * Nicola Mimmo - Junior Assistant Professor
    * Email: nicola.mimmmo2@unibo.it

## References
*If you use the ARVA plugin or this IMB regulator for your academic research, please cite our related papers.*
* **Paper Title**, Authors, Conference/Journal/Date ([PDF](link to pdf), [BibTex](link to bib)).
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

## 2. Build Autopilot
Clone the code from GitHub repository:
```
git clone https://github.com/lorenzogentilini/ab_es_proj_firmware.git --recursive
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
make px4_sitl_default gazebo
```
These command will launch a Gazebo simulation by spawning an Iris drone model already endowed of an ARVA receiver and producing an empty world with an ARVA transmitter at coordinates:
```
X =  50.0 m
Y = -50.0 m
Z =  0.0 m
```
and with rotations:
```
Y = 10.0°
R = 155.0°
P = 0.0°
```

## 3. Extremum Seeking Example
In order to execute the Extremum Seeking module, once the simulation has been launched, run (on px4 console):
```
extremum_seeking start
```
The drone then starts to find the ARVA transmitter poses at *[50.0, -50.0, 0.0]*, by following setpoints provided by the Bounded Update Rate algorithm.

In order to change such position one can modify the header file *Tools/sitl_gazebo/include/gazebo_arva_plugin.h*.
