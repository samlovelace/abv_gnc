<div align="center">

<h1> abv_gnc</h1>

<p>Guidance, Navigation and Control for the Air-Bearing Vehicles owned and operated by The Autonomy Lab</p>

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue?logo=ros)
![C++](https://img.shields.io/badge/C++-17-blue?logo=cplusplus)
![Eigen](https://img.shields.io/badge/Eigen-3.4-orange)

</div>
<div align="left">
<h2> Documentation </h2>
The complete documentation can be found <a href="https://abv-gnc.readthedocs.io">here</a>.

<h2>Overview</h2>
The Air-Bearing Vehicle's are a pair of 3DOF spacecraft simulators used for experiments involving contact dynamics, formation flying, and other rendezvous and proximity operations. It utilizes a set of air-bearings with on-board Nitrogen tanks to move across a glass table with very little friction. A second on-board tank pressures 8 different thrusters used for position and attitude control.

<h2> Architecture </h2>
The software suite is comprised of multiple ROS 2 packages, each implementing a primary

<h2>Quick Start </h2>
After cloning the repo, run the setup script to install any missing dependencies and build the packages.
```bash
./setup.sh
``` 
After the script finishes, launch the core GNC nodes.

<h4> Jetson Orin </h4>
```bash 
ros2 launch abv_bringup jetson.launch.py
```

<h4>General</h4>
```bash
ros2 launch abv_bringup gnc.launch.py
```
