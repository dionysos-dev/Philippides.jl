# Model Predictive Control Approach to Enhance Stable Walking for Planar Bipedal Robots 

This project explores two Model Predictive Control (MPC) approaches. Both rely on the
Zero Moment Point (ZMP) criterion, pre-planned footholds, and state feedback to
generate stable walking patterns for fully actuated planar robots. The first approach
employs the Linear Inverted Pendulum Model (LIPM) as a simplified representation
to meet the typical time constraints imposed by embedded hardware. The second
approach incorporates the robot’s full nonlinear dynamics to optimise actuator
commands, thereby enhancing walking efficiency while increasing computational
complexity.


This research was conducted as a part of another project called [Dionysos](https://github.com/dionysos-dev/Dionysos.jl) in order to benchmark the biped robot. This project is associated to the following [thesis](https://thesis.dial.uclouvain.be/entities/masterthesis/06c8c04f-2ca4-4743-b592-893a6d6bbef7).

The project contains the following folder: 

* `deps/` : containing the robots URDF and a global user defined parameter file
* `postprocessing/` : Contains postprocessing code to sampling the simulated file and pass .TAR file in .mp4 file. 
* `simulations/` : contains both control approach
* `src/` : contains the necessary file dedicated for this strategy.

## First Control Approach Structure 
![Structure of the first approach](https://github.com/dionysos-dev/Philippides.jl/blob/MPC/MPCBipedRobot/assets/first_control_scheme.PNG)

The figure shows the intended structure of the first control approach. The controller separated into 3 mains blocks : 
* Pre-processing stage : 
    * Path Planner, evaluates the general robot path 
    * Foot Planner, evaluates the landing position of the right and left foot.
    * Swing Foot Trajectory, determines the 3D position of the swing foot.
    * ZMP Trajectory Generator, defines the reference ZMP trajectory to remain within the support polygon.
* Online planning : 
    * MPC, iteratively computes the CoM trajectory of the robots based on the reference ZMP.
    * Forward Kinematics, translates the measure joint space coordinates into workspace coordinates.  
    * Inverse Kinematics, converts the local foot and CoM position in workspace coordinates into joint space coordinates.
* Simulation Environment : 
    * Low-Level Controller, computes actuators commands based on joint coordinates reference. Here, either a classical PID controller with dynamic compensation or a LQR controler.
    * Robot, a virtual robot within a virtual environment. 

![First control simulation](https://github.com/dionysos-dev/Philippides.jl/blob/MPC/MPCBipedRobot/assets/first_control_approach_result.mp4)

## Second Control Approach Structure
![Structure of the second approach](https://github.com/dionysos-dev/Philippides.jl/blob/MPC/MPCBipedRobot/assets/second_control_scheme.PNG)

The figure shows the intended structure of the second control approach. The controller separated into 3 mains blocks : 
* Pre-processing stage. 
* Online planning : 
    * MPC, iteratively computes joint coordinates commands based on the reference ZMP and foot trajectory.
* Simulation Environment. 
    
![Second control simulation](https://github.com/dionysos-dev/Philippides.jl/blob/MPC/MPCBipedRobot/assets/second_control_approach_result.mp4)

## How to run this project 

See [simulations](simulations/) for further information.

## Actual Version 

This version of the project does not support a closed-loop system when used with LQR in first control strategies.
