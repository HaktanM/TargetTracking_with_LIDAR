## TargetTracking_with_LIDAR
This is an implementation of ['A Cartesian B-Spline Vehicle Model for Extended Object Tracking'](https://ieeexplore.ieee.org/document/8455717), which explores the utilization of B-spline-based Extended Kalman Filter (EKF) for state estimation. I also provide my additional notes [here](https://www.overleaf.com/read/cbgfpjhtdcnf#236155). If you want to understand the implementation in detail, you can refer to my notes.

To start with, you can just run the main.m file.

# Simulation with LIDAR
I provide a LIDAR simulator with a target moving on the scene. However, I noticed that when measurements arrive from a single direction, system tends to diverge. 
![LIDAR](https://github.com/HaktanM/TargetTracking_with_LIDAR/blob/main/giffs/case2.gif)

# Simulation with Arbitrary Measurements
When the measurements arrive from arbitrary direction, the system is more stable.
![LIDAR](https://github.com/HaktanM/TargetTracking_with_LIDAR/blob/main/giffs/case1.gif)

# Arbitrary Shapes for Targets
It is possible to desing an arbitrary shape for the target very easily.
![LIDAR](https://github.com/HaktanM/TargetTracking_with_LIDAR/blob/main/giffs/case3.gif)
