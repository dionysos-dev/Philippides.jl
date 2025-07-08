# 🤖 Model Predictive Control Approach to Enhance Stable Walking for Planar Bipedal Robots

This project explores two Model Predictive Control (MPC) approaches for generating stable walking gaits in fully actuated planar bipedal robots. Both methods rely on the Zero Moment Point (ZMP) criterion, pre-defined footstep planning, and state feedback to ensure dynamic stability during locomotion.

1. The **first approach** uses the Linear Inverted Pendulum Model (LIPM) as a simplified representation to meet the real-time constraints of embedded hardware.
2. The **second approach** incorporates the robot’s full nonlinear dynamics to optimize actuator commands for improved efficiency, at the cost of increased computational complexity.

This project was developed as part of the [Dionysos](https://github.com/dionysos-dev/Dionysos.jl) research initiative and is associated with this [Master’s thesis](https://thesis.dial.uclouvain.be/entities/masterthesis/06c8c04f-2ca4-4743-b592-893a6d6bbef7).

---

## 📁 Project Structure

- `deps/` – Contains the robot's URDF and a global parameter configuration file.
- `postprocessing/` – Scripts for sampling simulation outputs and converting `.tar` archives into `.mp4` video files.
- `simulations/` – Entry points for running the first and second control approaches.
- `src/` – Source files that define the control logic and related utilities.

---

## 🧠 First Control Approach – Structure

![First Control Scheme](https://github.com/dionysos-dev/Philippides.jl/blob/MPC/MPCBipedRobot/assets/first_control_scheme.PNG)

The controller is divided into three primary modules:

### 1. Preprocessing
- **Path Planner** – Defines the overall robot trajectory.
- **Foot Planner** – Computes left and right foot landing positions.
- **Swing Foot Trajectory** – Generates the desired swing foot motion in 3D space.
- **ZMP Trajectory Generator** – Produces the reference ZMP trajectory to ensure balance.

### 2. Online Planning
- **MPC Module** – Solves for the optimal Center of Mass (CoM) trajectory based on the ZMP reference.
- **Forward Kinematics** – Converts joint angles to Cartesian space coordinates.
- **Inverse Kinematics** – Translates desired CoM and foot positions into joint angles.

### 3. Simulation Environment
- **Low-Level Controller** – Computes motor commands based on joint references. Two options are supported:
  - Classical PID controller with dynamics compensation
  - LQR controller (⚠️ *Open-loop only*)
- **Robot Simulator** – A virtual robot inside a physics-based environment.

#### 🎥 Simulation Result

![First Control Simulation](https://github.com/dionysos-dev/Philippides.jl/blob/MPC/MPCBipedRobot/assets/first_control_approach_result.gif)

---

## 🧠 Second Control Approach – Structure

![Second Control Scheme](https://github.com/dionysos-dev/Philippides.jl/blob/MPC/MPCBipedRobot/assets/second_control_scheme.PNG)

The second control approach also follows a modular architecture but is more streamlined:

### 1. Preprocessing
- Handles footstep planning and initialization (same than in first control approach).

### 2. Online Planning
- **Full-Body MPC** – Directly computes joint control commands from the reference ZMP and foot trajectory using the full nonlinear dynamics of the robot.

### 3. Simulation Environment
- Executes joint commands in the robot simulator.

#### 🎥 Simulation Result

![Second Control Simulation](https://github.com/dionysos-dev/Philippides.jl/blob/MPC/MPCBipedRobot/assets/second_control_approach_result.gif)

---

## 🚀 How to Run This Project

Please refer to the [simulations directory](simulations/) for instructions on how to execute each control approach in Julia. A complete guide is provided in the [README](simulations/README.md) of the `simulations/` repository.

---

## ⚠️ Known Limitations

- The **first control approach with LQR** currently only supports **open-loop** execution. Real-time closed-loop control is not yet supported.

---

## 📚 References

- Thesis: [Model Predictive Control Approach to Enhance Stable Walking for Planar Bipedal Robots](https://thesis.dial.uclouvain.be/entities/masterthesis/06c8c04f-2ca4-4743-b592-893a6d6bbef7)
- Related Project: [Dionysos.jl](https://github.com/dionysos-dev/Dionysos.jl)

---

**© UCLouvain – 2025**  
*Developed by [Brieuc de Poucques](https://github.com/brieucdp), as part of a Master’s thesis project in Electromechanical Engineering.*
