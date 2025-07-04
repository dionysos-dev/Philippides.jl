# ======================= Default values for the ZMP-based controller =======================

# ----------------------- Global parameters -----------------------
zmax    = 0.4225;                   # Maximal standing height of the robot [m]
zc_init = 0.21916919708029195;      # Height of the Center of Mass (CoM) [m] when upright (from URDF + RigidBodyDynamics)
ΔCoMz   = 0.026715057241694262;     # Variation of the height
zc = zc_init - ΔCoMz;               # Lowered height of the Center of Mass (CoM) [m]   
Ts = 0.02;                          # Sampling period [s]
g = 9.81;                           # Gravitational constant [m/s²]

#-----------------------Foot pattern parameters-----------------------
Lmax = 0.1;                         # Half of maximal step length for foot placement [m]
θ_max = 12 * pi / 180;              # Maximum step rotation angle [rad]
θ_0 = 0.0;                          # Initial orientation of the robot [rad]
isLeftSupport = true;               # Indicates that the robot starts with the left foot as support

# ----------------------- MPC parameters -----------------------
StartIndex = 1                      # Initial time index for MPC prediction
predictionTime = 1.0                # MPC prediction horizon duration [s]

# ----------------------- Path to follow definition -----------------------
## Straight reference path for 2D Robot Model 
t = vec(0:100)                      # Discrete time steps
yPath = 1.18 .+ 0.0 .* t            # Constant y-position (no lateral movement)
xPath = 0.01 * t                    # Linearly increasing x-position (forward motion)
θ_0 = 0                             # Initial heading angle
initial_position = [xPath[1], yPath[1], θ_0] # Initial reference position (center between feet)

# ----------------------- ZMP generator block -----------------------
Tstep = 1.0                      # Step duration [s]
δ = 0.2                          # Duration ratio of the double support phase (DSP) w.r.t the step period
Tdelay = 5                       # Initial delay for the robot to reach its final CoM height [s]
Twait = 5                        # Waiting time before starting walking [s]

#-----------------------Swing Foot Trajectory Generator-----------------------
Tver = 0.0 * Tstep;             # Duration of vertical motion phase [s] (Maximal when set to 0)
hstep = 0.01;                   # Maximum swing foot height w.r.t. the world frame [m]

#-----------------------Robot properties-----------------------
d                       = 0.052     # Distance between both feet (hip width) [m]
offset_hip_to_motor     = 0.04025   # Vertical offset between hip joint and motor [m]
offset_ankle_to_foot    = 0.009     # Vertical offset between ankle joint and foot sole [m]
footLength_x            = 7.0e-2    # Foot length along the x-axis [m]