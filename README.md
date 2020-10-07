# PX4 Autopilot - IMB Regulator

## Internal Model Based Regulator
<img src="https://github.com/lorenzogentilini/ab_es_proj_firmware/blob/devel/lorenzo/support_files/positions.png" width = 70% height = 40% />

## ARVA Simulator
<img src="https://github.com/lorenzogentilini/ab_es_proj_firmware/blob/devel/lorenzo/support_files/arva.png" width = 50% height = 40% />

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
* **Paper Title**, Authors, Conference/Journal/Date ([pdf](link to pdf), [bib](link to bib)).
* **Paper Title**, Authors, Conference/Journal/Date ([pdf](link to pdf), [bib](link to bib)).

## 1. System Prerequisites
### 1.1 Ubuntu and ROS
In order to build and run this autopilot, with the simulated ARVA plugin for Gazebo, one of the following option is required:
* Ubuntu 64-bit 16.04 with ROS Kinetic.
* Ubuntu 64-bit 18.04 with ROS Melodic.

Please follow the official [ROS Tutorial](http://wiki.ros.org/ROS/Installation) for installation.

### 1.2 MAVROS and GeographicLib
Install ROS repositories for mavros:
'''
  sudo apt-get install ros-*ros distro*-mavros ros-*ros distro*-mavros-extras
'''
## 2. Build Autopilot

## 3. Extremum Seeking Example
