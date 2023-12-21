# FIXME currently not up to date!

# The Tactile Odometry Drone

The Tactile Odometry Drone is a project in which we estimate aerial robots' odometry using information from physical contact. This repository agglomerates all information related to the tactile drone project.

![DronePortrait](./images/drone-portrait.png)

# How To Cite This

To Be Determined

## Explainer Video 

<div align="center">
  <a href="https://youtu.be/Z3fTCOoG-8s"><img src="./images/mqkQ5mHpCkM.png" alt="ExplainerVideo"></a>
</div>

# Background

A large part of research in the drone community is focused on finding and avoiding obstacles in an unknown environment to explore the environment, e.g. search and rescue applications. Usually, these applications use some form of vision sensor, such as cameras and/or lidars. 
These sensors as well as GNSS signals that are often used in combination to provide state estimation are susceptible to disturbances in indoor/underground environments. 
A camera can fail in bad lighting conditions, a lidar provides noisy measurements when smoke or dust fills the environment, and GNSS signals are blocked when indoors.

In this work, we investigate how instead of avoiding obstacles, we can use contact to navigate safely in an environment where the above-mentioned challenges are present. For this, we designed and build an inherently compliant interaction tool inspired by the human finger. 
It enables an Aerial System to safely establish contact with the environment and sense the magnitude and direction of the contact force. 
Based on that, the local normal of the environment can be inferred and new reference poses can be generated that move the system along the environment.

Generally speaking, the system aligns the interaction tool with the contact force and generates new reference positions a defined distance away, normal to the contact force vector and the world z-axis. 
For details, please refer to the publication above (or the schematic below).

<p align="center">
    <img src="./images/AMinContact.png" alt= “PlannerScheme” width="400">
</p>


# Results

The system was tested on various wall configurations.
See below for the resulting performance.

Straight Wall | Angled Away Wall | Angled Towards Wall | Bigger Angled Towards Wall
:---:|:---:|:---:|:---:
[<img src="./images/straight_wall_thumbnail.png" alt= “VideoStraightWall” width="400">](https://drive.google.com/file/d/1wb-oomzRinPbccY4n6zmMQCECdUvhsUz/view?usp=sharing) | [<img src="./images/angled_away_thumbnail.png" alt= “VideoAngledAwayWall” width="400">](https://drive.google.com/file/d/1mu5qWBLV5GjljpLQdXPlA3Y-cYLmAlfE/view?usp=share_link) | [<img src="./images/angled_towards_thumbnail.png" alt= “VideoAngledTowardsWall” width="400">](https://drive.google.com/file/d/1ICDdNCFxODMqksMb9ZQBc0hroavPmKRQ/view?usp=share_link)| [<img src="./images/bigger_angled_towards_thumbnail.png" alt= “VideoBiggerAngledTowardsWall” width="400">](https://drive.google.com/file/d/1VwmuxHRx2dI9Sj_H0ANfp8pA5DEYi9PN/view?usp=share_link)


Superimposed Altitude Commands | Combined Wall
:---:|:---:
[<img src="./images/superimposed_thumbnail.png" alt= “VideoSuperimposedAltitudeCommands” width="400">](https://drive.google.com/file/d/1IpwDYCMSlrDi99IpKvpPwwR4NIW9pJZF/view?usp=share_link) | [<img src="./images/wavy_wall_thumbnail.png" alt= “WawyWall” width="400">](https://drive.google.com/file/d/1-QrKH-Mo7ntR5Cm1EPXF7howRcNcXn4H/view?usp=share_link) 

# Repository Structure

This repository contains all the sub-repositories that are needed to reproduce the experiments. 
All recorded data is hosted off-site at this [link](FIXME).
The rest of this repository is structured as follows:
- ``tactile-drone-offboard``
    
    This repository contains all the software that is intended to be run off-board.
    This is mainly visualization modules of the system (rviz and plotting scripts) but also contains the Optitrack to ROS2 interface.
    It also includes the calibration and system ID files which operate of data that is recorded onboard using ``ros2 bag``. 

- ``tactile-drone-onboard``

    This repository contains all the software that is intended to be run onboard on the companion computer on the drone. 
    This includes the ``micro-ros-agent`` that interfaces with PX4 running on the Pixhawk but also the drivers for the encoders, the contact force estimator, as well as the contact-based planner that generates new references.

- ``tactile-drone-design``

    This repository contains all the mechanical design files. 

