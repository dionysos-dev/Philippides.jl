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

* deps/: containing the robots URDF and a global user defined parameter file
* postprocessing/: Contains postprocessing code to sampling the simulated file and pass .TAR file in .mp4 file. 
* simulations/: contains both control approach
* src/: contains the necessary file dedicated for this strategy.

## Controller Structure 
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

    ![Structure of the second approach](https://github.com/dionysos-dev/Philippides.jl/blob/MPC/MPCBipedRobot/assets/second_control_scheme.PNG

The figure shows the intended structure of the second control approach. The controller separated into 3 mains blocks : 
* Pre-processing stage. 
* Online planning : 
    * MPC, iteratively computes joint coordinates commands based on the reference ZMP and foot trajectory.
* Simulation Environment. 
    
![Result of the ZMP controller](https://github.com/7380Xing/Dionysos.jl/assets/99494151/1112c75a-d8aa-47c2-9f44-c9a1254466fb)

## Main References 
| Block | Reference(s) |
|-------|--------------|
| Foot Planner | R. Khusainov, A. Sagitov, A. Klimchik, and E. Magid. “Arbitrary Trajectory Foot Planner for Bipedal Walking:” in: Proceedings of the 14th International Conference on Informatics in Control, Automation and Robotics. 14th International Conference on Informatics in Control, Automation and Robotics. Madrid, Spain: SCITEPRESS - Science and Technology Publications, 2017, pp. 417–424. isbn: 978-989-758-263-9 978-989-758-264-6. doi: 10.5220/ 006442504170424.|
|Swing Foot | R. Khusainov, A. Sagitov, A. Klimchik, and E. Magid. “Arbitrary Trajectory Foot Planner for Bipedal Walking:” in: Proceedings of the 14th International Conference on Informatics in Control, Automation and Robotics. 14th International Conference on Informatics in Control, Automation and Robotics. Madrid, Spain: SCITEPRESS - Science and Technology Publications, 2017, pp. 417–424. isbn: 978-989-758-263-9 978-989-758-264-6. doi: 10.5220/ 006442504170424.|
| CoM Trajectory Generator | S. Kajita, F. Kanehiro, K. Kaneko, K. Fujiwara, K. Harada, K. Yokoi, and H. Hirukawa. “Biped walking pattern generation by using preview control of zero-moment point”. In: 2003 IEEE International Conference on Robotics and Automation (Cat. No.03CH37422). IEEE International Conference on Robotics and Automation. IEEE ICRA 2003. Taipei, Taiwan: IEEE, 2003, pp. 1620–1626. isbn: 978-0-7803-7736-3. doi: 10.1109/ROBOT.2003.1241826.|
| Preview Control  | T. Katayama, T. Ohki, T. Inoue, and T. Kato. “Design of an optimal controller for a discrete-time system subject to previewable demand”. In: International Journal of Control 41.3 (Mar. 1985), pp. 677–699. issn: 0020-7179, 1366-5820. doi: 10 . 1080 / 0020718508961156.|
## How to run this project 

This project has many examples, see [Examples](examples/) for further information.

## Actual Version 

This version of the project does not support a closed-loop system. In a short term, a closed-loop form will be developed to handle disturbed environment. 
