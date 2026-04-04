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
The software suite is comprised of multiple ROS 2 packages, each implementing a primary functionality. The table below shows the packages and summarizes their responsibility.

<table style="margin: 16px 0;">
<table>
  <thead>
    <tr>
      <th>Package</th>
      <th>Description</th>
    </tr>
  </thead>
  <tbody>
    <tr>
        <td><code>abv_bridge</code></td>
        <td>bridge node between abv_msgs and external</td>
    </tr>
    <tr>
        <td><code>abv_bringup</code></td>
        <td>configuration and launch files </td>
    </tr>
    <tr>
        <td><code>abv_commander</code></td>
        <td>command-line tool for sending commands</td>
    </tr>
    <tr>
        <td><code>abv_common</code></td>
        <td>dummy pkg for common code</td>
    </tr>
    <tr>
        <td><code>abv_controller</code></td>
        <td>feedback control via PID, thruster actuation</td>
    </tr>
    <tr>
      <td><code>abv_guidance</code></td>
      <td>waypoint sequencing and path following</td>
    </tr>
    <tr>
      <td><code>abv_msgs</code></td>
      <td>custom ROS2 message definitions</td>
    </tr>
    <tr>
      <td><code>abv_navigation</code></td>
      <td>state estimation via EKF, Optitrack integration</td>
    </tr>
    <tr>
        <td><code>abv_simulator</code></td>
        <td>numerical simulator for ABV dynamics</td>
    </tr>
    <tr>
        <td><code>abv_teleop</code></td>
        <td>keyboard or game controller teleop of the ABV </td>
    </tr>
  </tbody>
</table>

<h2>Quick Start </h2>
After cloning the repo, run the setup script to install any missing dependencies and build the packages.
<pre><code>./scripts/setup.sh
</code></pre>

After the script finishes, launch the core GNC nodes.

<h4> Jetson Orin </h4>
<pre><code>ros2 launch abv_bringup jetson.launch.py
</code></pre>

<h4>General</h4>
<pre><code>ros2 launch abv_bringup gnc.launch.py
</code></pre>
