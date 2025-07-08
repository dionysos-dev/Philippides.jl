# 🤖 MPCBipedRobot: Model Predictive Control for Planar Bipedal Robot Locomotion

This project provides a simulation framework for controlling a planar biped robot using Model Predictive Control (MPC) strategies.

---

## 📦 Installation Guide

### 1. Prerequisites

Ensure the following tools are installed on your system:

- [**Julia**](https://julialang.org/downloads/) ≥ 1.11
- [**Git**](https://git-scm.com/downloads) (Use Git Bash on Windows)
- [**Visual Studio Code**](https://code.visualstudio.com/) with the Julia extension *(optional but recommended)*

---

### 2. Clone the Repository

Open a terminal or Git Bash and run:

```bash
git clone git@github.com:dionysos-dev/Philippides.jl.git
cd Philippides.jl/
```

---

### 3. Set Up the Julia Environment

Launch the Julia REPL from the `MPCBipedRobot` directory:

```bash
cd MPCBipedRobot/
julia
```

Activate the project environment and install dependencies:

```julia
julia> ]
pkg> activate .
pkg> instantiate
```

This will automatically install all required packages listed in `Project.toml`.

---

## ▶️ Running Simulations

Once the environment is set up, you can run different control strategies.

---

### 🔹 First Control Approach (Simplified PID & LQR Control)

**1. PID Controller Simulation**

```julia
include("simulations/First_Control_Approach/simulation_PC.jl")
```

Runs the simulation using a position-based PID controller.

**2. LQR Controller Simulation**

```julia
include("simulations/First_Control_Approach/simulation_LQR.jl")
```

Runs the simulation using a linear-quadratic regulator.  
⚠️ *Note: Currently only available in open-loop mode.*

---

### 🔸 Second Control Approach (Full-Body MPC)

```julia
include("simulations/Second_Control_Approach/simulation.jl")
```

Runs simulations using a nonlinear MPC formulation based on the full robot dynamics.

---

## ⚙️ Configuration

Customize parameters to your needs by editing:

- `deps/param.jl` – Core robot and control parameters
- Simulation files – Modify scenario-specific values (e.g. step length, duration)

---

## 📁 Project Structure

```
Philippides.jl/
├── MPCBipedRobot/
│   ├── simulations/
│   │   ├── First_Control_Approach/
│   │   └── Second_Control_Approach/
│   ├── deps/
│   │   └── param.jl
│   ├── src/
│   │   └── ...
│   ├── Project.toml
│   └── ...
```

---

## 📌 Notes

- The repository includes both simplified and full-dynamics control methods for walking pattern generation.
- Visualization and result logging are handled automatically after simulation runs.
- Simulations are intended for academic research and benchmarking.

---

## 📫 Contact

For questions or collaboration requests, feel free to open an issue or reach out to the maintainers via GitHub.

---

**Happy Simulating!** 🚶‍♂️⚙️

**© UCLouvain – 2025**  
*Developed by [Brieuc de Poucques](https://github.com/brieucdp), as part of a Master’s thesis project in Electromechanical Engineering.*

