## Import necessary packages
using LinearAlgebra                     # Basic linear algebra
using Plots                             # Plotting utilities
using RigidBodyDynamics                 # Robot dynamics engine
using MeshCat, MeshCatMechanisms, Blink # 3D visualization tools
using LaTeXStrings                      # Display LaTeX in plots
using ControlSystems

## Load custom module for ZMP-based control
include(joinpath(@__DIR__, "..", "..", "src", "ZMPBipedRobot.jl"))
import .ZMPBipedRobot

## Conditional initialization to avoid re-import in REPL
STABLE_CODE = false
if (!isdefined(Main, :ZMProbot) && !STABLE_CODE)
    ZMProbot = ZMPBipedRobot
elseif(isdefined(Main, :ZMProbot))
    Base.eval(Main, :(ZMProbot = nothing))
    ZMProbot = ZMPBipedRobot
else
    const ZMProbot = ZMPBipedRobot
end

###########################################################
#                    General Parameters                   #
###########################################################

# Project directory structure
project_dir = joinpath(@__DIR__, "../../") 

# Plot parameters 
INTERACTIVE_PLOT = false;
PLOT_OFFLINE     = false;
PLOT_SIMU        = true; 
PLOT_ADDITIONAL  = false;
SAVE_FIG         = false;
ZOOM             = false;
lw, dpi, msw, ms = 2, 600, 0.1, 2
if INTERACTIVE_PLOT; plotlyjs(); else; gr(); end
save_dir = project_dir * "./Plot/First_Control_Approach/Simu_LQR/"  # Directory (relative to project root) for saving simulation plots

# Visualiser parameter
ANIMATE_RESULT = true;

# Simulation parameter
GRAVITY         = true;
CONTACTS        = false;
GROUND          = true;
OPEN_LOOP       = true; 
ENHANCED_MODEL  = false;
ctrl            = true;                   

###########################################################
#                     Virtual Robot                       #
###########################################################
println("\n○ Virtual environment definition")

# Load URDF and end-effector names depending on model type
if ENHANCED_MODEL
    urdf_path   = project_dir * "deps/Enhanced_URDF/Composite_piece/planar_locked_biped_robot/robot.urdf" 
    endEffector = ("feet" , "feet_2")
else 
    urdf_path   = project_dir * "deps/ZMP_2DBipedRobot.urdf"
    endEffector = ("l_foot_link", "r_foot_link")
end 
hip_link = "boom_link"

# Define 5 contact points per end-effectors
cp0 = [0.0, 0.0, -0.009]
cp1 = [0.035, 0.02, -0.009]
cp2 = [0.035, -0.02, -0.009]
cp3 = [-0.035, 0.02, -0.009]
cp4 = [-0.035, -0.02, -0.009]

# Store contact points
ContactPoints = Array[]
push!(ContactPoints, cp0, cp1, cp2, cp3, cp4)

# Create virtual environment and robot with URDF, contact points, and dynamics settings
rs = ZMProbot.RobotSimulator(;
    filePath = urdf_path,
    symbolic = false,
    add_contact_points = CONTACTS,
    add_gravity = GRAVITY,
    add_flat_ground = GROUND,
    contactmodel = ZMProbot.contact_model(kn=10e3, α=2.0, μ=0.8),
    contactPoints=ContactPoints, 
    endEffector=endEffector
);
println("✅ Virtual robot.")

# Retrieve state space size
nq = length(configuration(rs.state)) #state space size

# Define actuation matrix: Bu maps control inputs to accelerations
Bu = zeros(nq, 4)
for i in 1:4
    Bu[2+i, i] = 1.0 
end 

# Generate the 3D visualiser 
vis = ZMProbot.set_visulalizer(;
            mechanism = rs.mechanism, 
            filePath = urdf_path
)

# Uncomment the line below to visualize the defined contact points in the visualiser 
# ZMProbot.show_contact_point(rs.state,    
                            # vis,
                            # rs.mechanism;
                            # endEffector=endEffector,
                            # contactPoints=ContactPoints
# ) 

# Uncomment the line below to display body reference frames in the visualizer
# ZMProbot.show_frame!(rs, vis)  
println("✅ Visualiser.")


# Define Joint position Bounds 
q_bounds    = Dict{Int, Tuple{Float64, Float64}}()
q_bounds[2] = (-1.0, 1.0)
q_bounds[3] = (-3pi/4, pi/4)
q_bounds[4] = (-3pi/4, pi/4)
q_bounds[5] = (0.0, pi)
q_bounds[6] = (0.0, pi)
q_bounds[7] = (-3pi/4, 3pi/4)
q_bounds[8] = (-3pi/4, 3pi/4)

# Define Joint Velocity Bounds 
q̇_bounds    = Dict{Int, Tuple{Float64, Float64}}()
q̇_bounds[3] = (-3.1415, 3.1415)
q̇_bounds[4] = (-3.1415, 3.1415)
q̇_bounds[5] = (-5.23, 5.23)
q̇_bounds[6] = (-5.23, 5.23)
q̇_bounds[7] = (-5.23, 5.23)
q̇_bounds[8] = (-5.23, 5.23)

# Define Joint Acceleration Bounds 
q̈_bounds    = Dict{Int, Tuple{Float64, Float64}}()
q̈_bounds[3] = (-3.1415/0.2, 3.1415/0.2)
q̈_bounds[4] = (-3.1415/0.2, 3.1415/0.2)
q̈_bounds[5] = (-5.23/0.2, 5.23/0.2) 
q̈_bounds[6] = (-5.23/0.2, 5.23/0.2) 
q̈_bounds[7] = (-5.23/0.2, 5.23/0.2) 
q̈_bounds[8] = (-5.23/0.2, 5.23/0.2) 

# Define robot intial configuration
nominal_config = zeros(nq)
ZMProbot.set_nominal!(rs, vis, nominal_config)

# Load or reload kinematics & dynamics callable functions if URDF path changed
if(!isdefined(Main, :SYMBODYNAMICS))
    include("../../src/compute_dynamics.jl")
    SYMBODYNAMICS = urdf_path
else 
    if (SYMBODYNAMICS != urdf_path)
        include("../../src/compute_dynamics.jl")
        SYMBODYNAMICS = urdf_path
    end 
end 

# Define kinematics and dynamics structure
fk = ZMProbot.define_ForwardKinematics(f_CoM, f_J_CoM, f_J̇_CoM, f_F, f_J_F, f_J̇_F)
fd = ZMProbot.define_ForwardDynamics(f_Mass_Matrix, f_Bias)

###########################################################
#                   Control Parameters                    #
###########################################################

# Simulation step parameter
Δt = 1e-3

# LQR weight 
Rd = ZMProbot.eye(nq, T=Float64)    * 1.0e-12    # Penalizing control effort
Qd = ZMProbot.eye(3*nq; T=Float64)  * 1e-5       # Penalizing tracking error 
Qf = ZMProbot.eye(3*nq; T=Float64)  * 1.0e3      # Penalizing final state error 

###########################################################
#                  Pre-processing Planning                #
###########################################################
println("\n○ Pre-process planning")

# Construct the biped robot which store the geomtrical propreties and the path planned 
br = ZMProbot.BipedRobot(;
    paramFileName = "param.jl",
) 

# Run the Footstep Planner Algorithm and get the foot position 
fp = ZMProbot.FootPlanner(; br = br, check = PLOT_OFFLINE, save=SAVE_FIG, savePath=save_dir)
println("✅ Footstep planner.")

# Generate the ZMP reference trajectory 
zt, _ = ZMProbot.ZMPTrajectory(; br = br, fp = fp, check = PLOT_OFFLINE, save=SAVE_FIG, savePath=save_dir)
Time = reduce(vcat, zt.timeVec)
tend = Time[end]
println("✅ ZMP trajectory generator.")

# Generate the Swing Foot trajectory 
sf = ZMProbot.SwingFootTrajectory(; br = br, fp = fp, zt = zt, check = PLOT_OFFLINE, save=SAVE_FIG, savePath=save_dir)

# Retrieve both feet position trajectories
p_FL = Array(reduce(hcat, sf.stepL)')
p_FR = Array(reduce(hcat, sf.stepR)') 

# Compute both feet velocity trajectories
v_FL = [ZMProbot.differentiate(p_FL[:, 1], br.Ts; init_nul=true) ZMProbot.differentiate(p_FL[:, 2], br.Ts; init_nul=true)  ZMProbot.differentiate(p_FL[:, 3], br.Ts; init_nul=true)]
v_FR = [ZMProbot.differentiate(p_FR[:, 1], br.Ts; init_nul=true) ZMProbot.differentiate(p_FR[:, 2], br.Ts; init_nul=true)  ZMProbot.differentiate(p_FR[:, 3], br.Ts; init_nul=true)]

# Compute both feet acceleration trajectories
a_FL = [ZMProbot.differentiate(v_FL[:, 1], br.Ts; init_nul=true) ZMProbot.differentiate(v_FL[:, 2], br.Ts; init_nul=true)  ZMProbot.differentiate(v_FL[:, 3], br.Ts; init_nul=true)]
a_FR = [ZMProbot.differentiate(v_FR[:, 1], br.Ts; init_nul=true) ZMProbot.differentiate(v_FR[:, 2], br.Ts; init_nul=true)  ZMProbot.differentiate(v_FR[:, 3], br.Ts; init_nul=true)]
println("✅ Foot trajectories.")

###########################################################
#                      Online Planning                    #
###########################################################
println("\n○ Online planning definition")

# MPC reference to track and cost matrices
MPC_ref = Matrix(reduce(hcat, zt.ZMP)[1, :]')
N       = trunc(Int, br.predictionTime / br.Ts)
Q       = Matrix{Float64}(1.0I, 1, 1) * 1.0     # Poids du suivi de la référence
R       = Matrix{Float64}(1.0I, 1, 1) * 1.0e-4   # Poids de l’effort de commande, attention, quand on diminue ça, l'IK ne va peut-etre pas trouvé une solution correct pour vit et acc

# Discretised cart-table model
A, B, C    = ZMProbot.cartTableModel(br.zc, br.g)
Ad, Bd, Cd = ZMProbot.continuous2discrete(A, B, C, br.Ts)

# Define dynamics evolution, observation, and objective functions
F_xu   = ZMProbot.stateTransitionFunction(Ad, Bd)
G_x    = ZMProbot.observationFunction(Cd)
O_xuyr = ZMProbot.objectiveFunction(Q, R)

# Define MPC structure with problem size, functions, time, option
mpcParam = ZMProbot.defineMpcParameter(
                    size(Ad, 2), size(Bd, 2),  
                    F_xu, G_x,
                    Time, N, 
                    MPC_ref, O_xuyr, :Min; 
                    optn=(("verbose", false), ("warm_start", true)), #("eps_abs", 1e-3), ("eps_rel", 1e-3)
                    Type=0
)
println("✅ MPC.")
 
# Initial CoM state
p_init_com = center_of_mass(rs.state).v

# CoM lowering phase: construct the CoM trajectory along z-axis
z_coeff  = ZMProbot.getSplineCoeff(zt.timeVec[1][1], zt.timeVec[1][end], p_init_com[3], br.zc)                               
zc       = vcat(ZMProbot.spline(zt.timeVec[1], z_coeff), br.zc * ones(length(Time) - length(zt.timeVec[1]) + mpcParam.N))
żc       = ZMProbot.differentiate(zc, br.Ts; init_nul=true)
z̈c       = ZMProbot.differentiate(żc, br.Ts; init_nul=true)
println("✅ CoM lowering phase.")

# Define inverse Kinematics structure
ik = ZMProbot.define_IK(q_bounds, q̇_bounds, q̈_bounds)
println("✅ Inverse kinematics.")

###########################################################
#                  Simulation Environement                #
###########################################################

# Retrieve hip and foot bodies 
offset_boom_motor = [0.0; 0.0; -br.offset_hip_to_motor]
hip_body = findbody(rs.mechanism, hip_link)
endEffector_body = Vector{RigidBody{Float64}}[]
for link in endEffector
    push!(endEffector_body, [findbody(rs.mechanism, link)])
end 
endEffector_body = reduce(vcat, endEffector_body)

# Get positions of both feet and hip
p_hipMotor = ZMProbot.local2world(rs.state, hip_body, offset_boom_motor)
p_endEffector = Vector{eltype(p_hipMotor)}[]
offset_feet_ankle = [0.0; 0.; -br.offset_ankle_to_foot]
for body in endEffector_body
    push!(p_endEffector, ZMProbot.local2world(rs.state, body,  offset_feet_ankle))
end 

###########################################################
#                  Code Initialisation & Memory           #
###########################################################

## Initialisation
# Joint commands and velocity 
τ_prev = zeros(nq)
q̇_prev = collect(velocity(rs.state))

# MPC 
x_init = [p_init_com[1]; 0.0; 0.0]
u0 = [0.0]

# IK 
q_init = copy(nominal_config)
q̇_init = zeros(nq, 1)
q̈_init = zeros(nq, 1)

## Memory 
# MPC 
x_ref = copy(x_init[1])
ẋ_ref = copy(x_init[2])
ẍ_ref = copy(x_init[3])
u_ref = [0.0]

# IK 
q_ref = copy(q_init)
q̇_ref = copy(q̇_init)
q̈_ref = copy(q̈_init)

# Simulation  
qsim = Array{}[]
vsim = Array{}[]
tsim = Array{}[]

# Mean CoM states and torques 
mean_comx = [0.0]
mean_comz = [p_init_com[3]]
mean_cȯmx = [0.0]
mean_cȯmz = [0.0]
mean_cömx = [0.0]
mean_cömz = [0.0]
τmean = zeros(8)

# ZMP estimations
zmp_LIPM_estimated  = [0.0] # Contains the simulated ZMP estimated with LIPM
zmp_VHIP_estimated  = [0.0] # Contains the simulated ZMP estimated with VHIPM
zmp_MPC_estimated   = [0.0] # Contains the MPC computed reference ZMP

# Online planning timer 
computation_time = []

###########################################################
#                    Main Simulation Loop                 #
###########################################################
println("\n○ Simulation loop \n")
println("------------------------")
for t in 0.0:br.Ts:tend-br.Ts 

    # Display current simulation time & start timer 
    println("• Time: ", string(t), "/", string(tend-br.Ts))
    start_time = time()

    ##############################
    # 1. Model Predictive Control
    ##############################

    # Update the MPC reference and index for current time
    ZMProbot.updateMpcParameter(mpcParam, t, MPC_ref)

    # Compute optimal CoM trajectory along the x-axis and control input
    x, ux = ZMProbot.computeMPControl(mpcParam, x_init, u0)

    #########################
    # 2. Inverse Kinematics
    #########################

    # Warm-start 
    ik_warm_start = [q_init q̇_init q̈_init]

    # Build CoM input for inverse kinematics
    y = [p_init_com[2]; 0.0; 0.0]
    z = [zc[mpcParam.index+1]; żc[mpcParam.index+1]; z̈c[mpcParam.index+1]]
    centerOfMass = hcat(x[:, 1], y, z)
    
    # Build foot inputs for inverse kinematics
    foot1        = [p_FL[mpcParam.index+1, :]'; v_FL[mpcParam.index+1, :]'; a_FL[mpcParam.index+1, :]']
    foot2        = [p_FR[mpcParam.index+1, :]'; v_FR[mpcParam.index+1, :]'; a_FR[mpcParam.index+1, :]']
    
    # Solve inverse kinematics to get joint references
    q, q̇, q̈ = ZMProbot.inverse_kinematic(
        ik, fk, ik_warm_start, 
        centerOfMass, foot1, foot2;
        optn=(("print_level", 0),),
        check=PLOT_ADDITIONAL
    )

    # Stop timer & measure elapsed time
    end_time = time()
    push!(computation_time, end_time - start_time)


    ############################
    # 3. LQR Control Definition
    ############################

    global Al, Bl, Dl = ZMProbot.LinearizedAugmentedDynamics(rs.mechanism, q, q̇, q̈, Bu, Δt, endEffector) #ok comme transfo de repère pour hip comme q1 bloqué
    global Ad, Bd, Dd = ZMProbot.LQR_discretisation(Al, Bl, Dl, Δt)
    
    AD = Ad
    BD = Bd + (Dd ./ Δt)

    println("• Contrôlabilité : ", rank(ctrb(AD, BD)), " / ", size(AD,1))
    println("• Observabilité  : ", rank(obsv(AD, sqrt(Qd))), " / ", size(AD,1))

    K = ZMProbot.Riccati_DTFH(AD, BD, br.Ts, Δt, Qd, Qf, Rd)
    xref = vcat(q, vcat(q̇, q̈))

    ############################
    # 4. Simulation and Control
    ############################

    # Simulate robot dynamics using computed joint references and LQR control
    controller! = ZMProbot.LQR_controller!(rs, xref, K, BD, Dd, τ_prev, Bu, q̇_prev, Δt, br.Ts, t)
    ts, qs, vs = RigidBodyDynamics.simulate(rs.state, br.Ts, controller!; Δt = Δt);

    # Store simulation results with time alignment
    ts = round.(ts; digits = 6)
    last_valid_idx = findlast(t -> t ≤ br.Ts, ts)
    if t == 0.0
        push!(qsim, qs[1:last_valid_idx])
        push!(vsim, vs[1:last_valid_idx])
        push!(tsim, ts[1:last_valid_idx] .+ t)
    else
        push!(qsim, qs[2:last_valid_idx])
        push!(vsim, vs[2:last_valid_idx])
        push!(tsim, ts[2:last_valid_idx] .+ t)
    end

    # Update robot state for next iteration (ensure consistency)
    set_configuration!(rs.state, qs[last_valid_idx])
    set_velocity!(rs.state, vs[last_valid_idx])

    #####################
    # 5. Sensor Feedback 
    #####################

    # Update joint state for next iteration
    global q_init = qs[end]
    global q̇_init = q̇
    global q̈_init = q̈

    # Store the first control input to warm-start the next iteration
    global u0 .= ux[:, 1]  
    
    # Update CoM state (x) for MPC depending on open/closed loop control 
    if OPEN_LOOP
        global x_init = x[:, 1]
    else
        global x_init = [rs.CoM[end][1]; rs.CȯM[end][1]; rs.CöM[end][1]] 
    end 

    # Store new foot positions (for ZMP correction if needed)
    for body in endEffector_body
        push!(p_endEffector, ZMProbot.local2world(rs.state, body, [0.0; 0.0; -br.offset_ankle_to_foot]))
    end 

    ## Adapte ZMP x ref based on actual foot position (uncomment lines below)
    # offsetL = p_FL[mpcParam.index, 1] - p_endEffector[end-1][1]
    # offsetR = p_FR[mpcParam.index, 1] - p_endEffector[end][1]
    # if zt.isDSP[mpcParam.index]
    #     offset = (offsetL +  offsetR) / 2
    # elseif sf.isLeftFlag[mpcParam.index]
    #     offset = offsetR
    # else 
    #     offset = offsetL 
    # end 
    # global Yref[1, mpcParam.index+1] = Yref[1, mpcParam.index+1] - offset

    # Get updated position of hip joint in world frame
    global p_hipMotor = ZMProbot.local2world(rs.state, hip_body, offset_boom_motor)
    println("• Hip global position: ", round.(p_hipMotor, digits=3))
 
    #####################
    # 6. ZMP estimations
    #####################

    # Simulated ZMP, measure using LIPM 
    push!(zmp_LIPM_estimated, (Cd * [rs.CoM[end][1]; rs.CȯM[end][1]; rs.CöM[end][1]])[1]) 
    # Simulated ZMP, measure using VHIPM
    push!(zmp_VHIP_estimated, rs.CoM[end][1] - (rs.CoM[end][3] / (br.g + rs.CöM[end][3]) *  rs.CöM[end][1]))
    # ZMP corresponding to MPC computed states 
    push!(zmp_MPC_estimated, (Cd * x[:, 1])[1])

    #############################
    # 7. Mean values computation
    #############################

    # Get number of sample during the last simulation time interval and compute means over it
    D = length(rs.CȯM[rs.index:end])
    println("• Mean Torques over $(br.Ts)s: ", round.(sum(rs.torques[rs.index:end]./D), digits=5))
    global τmean = hcat(τmean, sum(rs.torques[rs.index:end])./D)
    global mean_comx = vcat(mean_comx, sum(rs.CoM[rs.index:end])[1] ./ D)
    global mean_comz = vcat(mean_comz, sum(rs.CoM[rs.index:end])[3] ./ D)
    global mean_cȯmx = vcat(mean_cȯmx, sum(rs.CȯM[rs.index:end])[1] ./ D)
    global mean_cȯmz = vcat(mean_cȯmz, sum(rs.CȯM[rs.index:end])[3] ./ D)
    global mean_cömx = vcat(mean_cömx, sum(rs.CöM[rs.index:end])[1] ./ D)
    global mean_cömz = vcat(mean_cömz, sum(rs.CöM[rs.index:end])[3] ./ D)

    ################################
    # 8. Log Reference Trajectories
    ################################

    global x_ref = hcat(x_ref, x[1, 1])
    global ẋ_ref = hcat(ẋ_ref, x[2, 1])
    global ẍ_ref = hcat(ẍ_ref, x[3, 1])
    global q_ref = hcat(q_ref, q)
    global q̇_ref = hcat(q̇_ref, q̇)
    global q̈_ref = hcat(q̈_ref, q̈)
    push!(u_ref, u0[1]) 

    println("------------------------")
end


###########################################################
#                  Visualise Result                       #
###########################################################

qsim = map(q -> collect(q), qsim)
tsim = map(q -> collect(q), tsim)

# Open the visualiser and run the animation 
if ANIMATE_RESULT
    open(vis)
    qani = reduce(vcat, qsim)
    tani = reduce(vcat, tsim)
    animation = MeshCat.Animation(vis, tani, qani)
    setanimation!(vis, animation)
end

###########################################################
#                      Plot results                       #
###########################################################
println("\n○ Plots computation\n")

# Time vector
tplot = Time

# ZMP reference 
longZMP = reduce(hcat, zt.ZMP)
ZMP_x = reduce(vcat, longZMP[1, :])
update_ZMP_x = copy(MPC_ref[1, :])

# CoM reference 
com = vcat(x_ref, zc[1:end-mpcParam.N]')
cȯm = vcat(ẋ_ref, żc[1:end-mpcParam.N]')
cöm = vcat(ẍ_ref, z̈c[1:end-mpcParam.N]')

# Simulation results 
vsim = reduce(hcat, (reduce(vcat, vsim)))'
qsim = reduce(hcat, (reduce(vcat, qsim)))'
tsim = reduce(vcat, tsim)
torque_sim = reduce(hcat, rs.torques)
CoMsim = reduce(hcat, rs.CoM)
CȯMsim = reduce(hcat, rs.CȯM)
CöMsim = reduce(hcat, rs.CöM)
p_foot1 = reduce(hcat, p_endEffector[1:length(endEffector_body):end])
p_foot2 = reduce(hcat, p_endEffector[2:length(endEffector_body):end])


# Actuated Joint Plots 
offset = 2
if (length(torque_sim[1, :]) == length(tsim) + 1)
    len_sim = length(tsim)
    len_t = len_sim
elseif (length(torque_sim[1, :]) == length(tsim) - 1)
    len_t = length(torque_sim[1, :])
    len_sim = len_t
else
    len_t = length(tsim)
    len_sim = len_t
end

plt_θ = plot(;
    xlims = (0, tend),
    xlabel = L"$t$ [s]",
    legend = true,
    legendcolumns = 2,
    dpi = dpi,
    layout = (2, 2),
)
plt_ω = plot(;
    xlims = (0, tend),
    xlabel = L"$t$ [s]",
    ylabel = L"$\omega$ [rad/s]",
    layout = (2, 2),
    dpi = dpi,
)
plt_τ = plot(;
    xlims = (0, tend),
    dpi = dpi,
    xlabel = L"$t$ [s]",
    ylabel = L"$\tau$ [Nm]",
    layout = (2, 2),
)

for (joint, name) in enumerate(["Leg", "Knee"])
    for (side_idx, side) in enumerate(["Left", "Right"])
        str = latexstring("q_{$(joint + 2)$(side_idx)}") * " [rad]"
        ylabel!(plt_θ[joint, side_idx], str)
        
        str = latexstring("q̇_{$(joint + 2)$(side_idx)}") * " [rad/s]"
        ylabel!(plt_ω[joint, side_idx], str)

        str = latexstring("τ_{$(joint + 2)$(side_idx)}") * " [Nm]"
        ylabel!(plt_τ[joint, side_idx], str)
        plot!(
            plt_θ[joint, side_idx],
            tsim,
            qsim[:, (offset - 1) + (2 * joint  - 1 ) + side_idx]; # de 3 à 6
            lw = lw,
            label ="Simulated",
            # title = "$(side) $(name)",
        )
        plot!(
            plt_θ[joint, side_idx],
            tplot,
            q_ref[(offset - 1) + (2 * joint  - 1 ) + side_idx, :]; # de 4 à 7
            lw = lw,
            label = "Reference",
            # title = "$(side) $(name)",
        )
        plot!(
            plt_τ[joint, side_idx],
            tsim[1:len_t],
            torque_sim[(offset - 1) + (2 * joint  - 1 ) + side_idx, 1:len_sim];
            lw = lw,
            label ="Simulated",
            # title = "$(side) $(name)",
        )
        plot!(
            plt_τ[joint, side_idx],
            tplot,
            τmean[(offset - 1) + (2 * joint  - 1 ) + side_idx, :]; # de 3 à 6
            lw = lw,
            label = "Mean sim. over "*L"T_s",
            # color=3,
            # title = "$(side) $(name)",
        )
        plot!(
            plt_ω[joint, side_idx],
            tsim,
            vsim[:, (offset - 1) + (2 * joint  - 1 ) + side_idx];
            lw = lw,
            label ="Simulated",
            # title = "$(side) $(name)",
        )
        plot!(
            plt_ω[joint, side_idx],
            tplot,
            q̇_ref[(offset - 1) + (2 * joint  - 1 ) + side_idx, :];   # de 4 à 7
            lw = lw,
            label = "Reference",
            # title = "$(side) $(name)",
        )
        if (joint == 1 && side_idx==1)
            plot!(plt_θ[joint, side_idx]; legend = true)
            plot!(plt_ω[joint, side_idx]; legend = true)
            plot!(plt_τ[joint, side_idx]; legend = true)
        else 
            plot!(plt_θ[joint, side_idx]; legend = false)
            plot!(plt_ω[joint, side_idx]; legend = false)
            plot!(plt_τ[joint, side_idx]; legend = false)
        end
    end
end

# ------------------
#     CoM Plot 
# ------------------
lab = "Mean sim. over "*L"T_s"
Above_legend=false
Down_legend = :topright
Legendfontsize=6
Legendcolumns=1

plt_com = plot(; xlims = (0, tend), layout = (2, 1), dpi = dpi, legendfontsize=Legendfontsize, legendcolumns =Legendcolumns)
plot!(
    plt_com[1],
    tsim[1:len_t],
    CoMsim[1, 1:len_sim];
    lw = lw,
    label ="Simulated",
    ylabel =  L"x_c" *" [m]",
    title = "CoM position",
    legend=Above_legend
)
plot!(plt_com[1], tplot, com[1, :]; label = "Reference", lw = lw,legend=Above_legend)
plot!(
    plt_com[2],
    tsim[1:len_t],
    CoMsim[3, 1:len_sim];
    lw = lw,
    label ="Simulated",
    ylabel =  L"z_c" *" [m]",
    xlabel = L"$t$ [s]",
    legend=Down_legend
)
plot!(plt_com[2], tplot, com[2, :]; label = "Reference", lw = lw, legend=Down_legend)

plt_cȯm = plot(; xlims = (0, tend), layout = (2, 1), dpi = dpi,legendfontsize=Legendfontsize, legendcolumns =Legendcolumns)
plot!(
    plt_cȯm[1],
    tsim[1:len_t],
    CȯMsim[1, 1:len_sim];
    lw = lw,
    label ="Simulated",
    ylabel =  L"\dot{x}_c" *" [m/s]",
    title = "CoM velocity",
    legend=Above_legend
)
plot!(plt_cȯm[1], tplot, cȯm[1, :]; label = "Reference", lw = lw, legend=Above_legend)
plot!(plt_cȯm[1], tplot, mean_cȯmx; label = lab, lw = lw, legend=Above_legend)
plot!(
    plt_cȯm[2],
    tsim[1:len_t],
    CȯMsim[3, 1:len_sim];
    lw = lw,
    ylabel =  L"\dot{z}_c" *" [m/s]",
    xlabel = L"$t$ [s]",
    legend = Down_legend, 
    label="Simulated"
)
plot!(plt_cȯm[2], tplot, cȯm[2, :]; lw = lw, label ="Reference",legend = Down_legend)
plot!(plt_cȯm[2], tplot, mean_cȯmz; lw = lw, label=lab,legend = Down_legend)


plt_cöm = plot(; xlims = (0, tend), layout = (2, 1), dpi = dpi,legendfontsize=Legendfontsize, legendcolumns =Legendcolumns)
plot!(
    plt_cöm[1],
    tsim[1:len_t],
    CöMsim[1, 1:len_sim];
    lw = lw,
    label ="Simulated",
    ylabel =  L"\ddot{x}_c" * " "*L"[m/$s^2$]",
    title = "CoM acceleration",
    legend = Above_legend
)
plot!(plt_cöm[1], tplot, cöm[1, :]; label ="Reference", lw = lw, legend=Above_legend)
plot!(plt_cöm[1], tplot, mean_cömx; label = lab, lw = lw, legend=Above_legend)
plot!(
    plt_cöm[2],
    tsim[1:len_t],
    CöMsim[3, :];
    lw = lw,
    ylabel =  L"\ddot{z}_c" * " "*L"[m/$s^2$]",
    xlabel = L"$t$ [s]",
    legend = Down_legend,
    label="Simulated"
)
plot!(plt_cöm[2], tplot, cöm[2, :]; lw = lw, label ="Reference", legend = Down_legend)
plot!(plt_cöm[2], tplot, mean_cömz; lw = lw, label=lab, legend = Down_legend)

# ------------------
#     ZMP Plot 
# ------------------

# ZMP evolution Plot
plt_zmp = plot(; xlabel = L"$t$ [s]", ylabel = L"$X$ [m]", xlims = (0, tend), layout = (1, 1), dpi = dpi)
# plot!(plt_zmp[1], tplot, zmp_VHIP_estimated; label =latexstring("p_{sim, x}")*" Meas. with VHIPM", lw = lw)
plot!(plt_zmp[1], tplot, ZMP_x; color=:2, label =L"p_{ref, x}", lw = lw)
plot!(plt_zmp[1], tplot, zt.Limx[1, :]; linestyle=:dash, color=:red, label=nothing)
plot!(plt_zmp[1], tplot, zt.Limx[2, :]; linestyle=:dash, color=:red, label=nothing)
lab = latexstring("p_{x}")
plot!(plt_zmp[1], tplot, zmp_MPC_estimated; color=:green, label =lab, lw = lw)


plt_zmp2 = plot(; ylabel=L"$X$ [m]", xlabel = L"$t$ [s]", layout = (1, 1), dpi = dpi)
plot!(plt_zmp2, tplot, zmp_VHIP_estimated; label =latexstring("p_{sim, x}")*" Meas. with VHIPM", lw = lw)
lab = latexstring("p_{x}")
plot!(plt_zmp2, tplot, zmp_MPC_estimated; color=:green, label =lab, lw = lw)
plot!(plt_zmp2, tplot, zt.Limx[1, :]; linestyle=:dash, color=:red, label="Sim. stability area")
plot!(plt_zmp2, tplot, zt.Limx[2, :]; linestyle=:dash, color=:red, label=nothing)

# --------------------------------
#     MPC reference tracking Plot
# --------------------------------
plt_mpc = plot(
    xlabel = L"t\ [\mathrm{s}]",
    xlims  = (0, tend),
    dpi    = dpi,
)

#— now create the twin axis and plot on it —                         
plot!(plt_mpc, tplot, u_ref;
    label  = L"u_{\mathrm{ref}}",
    ylabel = L"\mathrm{Jerk}\ [\mathrm{m/s^3}]",
    color  = :skyblue,
    legend = :topleft, 
    lw     = lw,
)

ax2 = twinx(plt_mpc)   
plot!(ax2, tplot, ZMP_x; ylabel = L"X\ [\mathrm{m}]", color=2, label=L"p_{\mathrm{ref, x}}", lw=lw)
plot!(ax2, tplot, zt.Limx[1, :]; linestyle=:dash, color=:red, label="")
plot!(ax2, tplot, zt.Limx[2, :]; linestyle=:dash, color=:red, label="")
plot!(ax2, tplot, zmp_MPC_estimated; legend=:bottomright, color=:green, label=L"p_{\mathrm{x}}", lw=lw)


if PLOT_SIMU
    display(plt_θ)
    display(plt_τ)
    display(plt_ω)
    display(plt_com)
    display(plt_cȯm)
    display(plt_cöm)
    display(plt_zmp) 
    display(plt_mpc) 
    display(plt_zmp2)
    if SAVE_FIG
        savefig(plt_mpc, save_dir*"plt_mpc.png")
        savefig(plt_zmp, save_dir*"plt_zmp.png")
        savefig(plt_zmp2, save_dir*"plt_zmp2.png")
        savefig(plt_com, save_dir*"plt_com.png")
        savefig(plt_cȯm, save_dir*"plt_cȯm.png")
        savefig(plt_cöm, save_dir*"plt_cöm.png")
        savefig(plt_θ, save_dir*"plt_θ.png")
        savefig(plt_τ, save_dir*"plt_τ.png")
        savefig(plt_ω, save_dir*"plt_ω.png")
    end 
end

if ZOOM 
    xlims!(plt_zmp, 11, 13)
    ylims!(plt_zmp, -0.05, 0.3)
    xlims!(plt_zmp2, 11, 13)
    ylims!(plt_zmp2, -0.05, 0.3)
    xlims!(plt_com, 11, 13)
    ylims!(plt_com[1], 0, 0.26)
    ylims!(plt_com[2], 0.188, 0.195)
    xlims!(plt_cȯm, 11, 13)
    ylims!(plt_cȯm[1], -0.05, 0.27)
    ylims!(plt_cȯm[2], -0.02,0.02)
    xlims!(plt_cöm, 11, 13)
    ylims!(plt_cöm[1], -13, 13)
    ylims!(plt_cöm[2], -5, 5)
    xlims!(plt_θ, 0, 0.1)
    ylims!(plt_θ[1, 1], -0.06, 0.0)
    ylims!(plt_θ[1, 2], -0.06, 0.0)
    ylims!(plt_θ[2, 1], 0, 0.06)
    ylims!(plt_θ[2, 2], 0, 0.06)
    xlims!(plt_τ, 11, 13)
    xlims!(plt_ω, 11, 13)
    display(plt_θ)
    display(plt_τ)
    display(plt_ω)
    display(plt_zmp)
    display(plt_zmp2)
    display(plt_com)
    display(plt_cȯm)
    display(plt_cöm)
    if SAVE_FIG
        savefig(plt_θ, save_dir*"plt_zoom_θ.png")
        savefig(plt_τ, save_dir*"plt_zoom_τ.png")
        savefig(plt_mpc, save_dir*"plt_zoom_mpc.png")
        savefig(plt_zmp, save_dir*"plt_zoom_zmp.png")
        savefig(plt_zmp2, save_dir*"plt_zoom_zmp2.png")
        savefig(plt_com, save_dir*"plt_zoom_com.png")
        savefig(plt_cȯm, save_dir*"plt_zoom_cȯm.png")
        savefig(plt_cöm, save_dir*"plt_zoom_cöm.png")
        savefig(plt_ω, save_dir*"plt_zoom_ω.png")
    end
end  

# ------------------------
#     Additional Plots
# ------------------------

plt_x_f = plot(; xlabel = L"$t$ [s]", layout = (2, 1), dpi = dpi)
plot!(
    plt_x_f[1],
    tplot,
    p_foot1[1, :];
    lw = lw,
    label ="Simulated",
    ylabel = L"$x_{f1}$ [m]",
    #title = "Foot 1",
)
plot!(plt_x_f[1], tplot, p_FL[:, 1]; label = "Reference", lw = lw)
plot!(
    plt_x_f[2],
    tplot,
    p_foot2[1, :];
    lw = lw,
    #label ="Simulated",
    ylabel = L"$x_{f2}$ [m]",
    #title = "Foot 1",
    legend=false
)
plot!(plt_x_f[2], tplot, p_FR[:, 1]; #label = "Reference",
 lw = lw, legend=false)

plt_z_f = plot(; xlabel = L"$t$ [s]", layout = (2, 1), dpi = dpi)

plot!(
    plt_z_f[1],
    tplot,
    p_foot1[3, :];
    lw = lw,
    label ="Simulated",
    ylabel = L"$z_{f1}$ [m]",
    #title = "Foot 1",
)
plot!(plt_z_f[1], tplot, p_FL[:, 3]; label = "Reference", lw = lw)
plot!(
    plt_z_f[2],
    tplot,
    p_foot2[3, :];
    lw = lw,
    #label ="Simulated",
    ylabel = L"$z_{f2}$ [m]",
    #title = "Foot 1",
    legend=false
)
plot!(plt_z_f[2], tplot, p_FR[:, 3]; #label = "Reference",
 lw = lw, legend=false)

plt_feet1 = plot(; xlabel = L"$x$ [m]", layout = (1, 1), dpi = dpi)

plot!(
    plt_feet1,
    p_foot1[1, :],
    p_foot1[3, :];
    lw = lw,
    label ="Simulated",
    ylabel = L"$z$ [m]",
    title = "Foot 1",
)
plot!(plt_feet1, p_FL[:, 1], p_FL[:, 3]; label = "Reference", lw = lw)

plt_feet2 = plot(; xlabel = L"$x$ [m]", layout = (1, 1), dpi = dpi)

plot!(
    plt_feet2,
    p_foot2[1, :],
    p_foot2[3, :];
    lw = lw,
    label ="Simulated",
    ylabel = L"$z$ [m]",
    title = "Foot 2",
)
plot!(plt_feet2, p_FR[:, 1], p_FR[:, 3]; label = "Reference", lw = lw)

plt_pfeet = plot(; xlabel = L"$x$ [m]", layout = (2, 1), dpi = dpi)

plot!(
    plt_pfeet[1],
    tplot,
    p_FL[:, 1];
    lw = lw,
    label = L"Foot 1: $p_x [m]$",
)
plot!(
    plt_pfeet[1],
    tplot,
    p_FR[:, 1];
    lw = lw,
    label = L"Foot 2: $p_x [m]$",
)

plot!(
    plt_pfeet[2],
    tplot,
    p_FL[:, 3];
    lw = lw,
    label = L"Foot 1: $p_z [m]$",
)
plot!(
    plt_pfeet[2],
    tplot,
    p_FR[:, 3];
    lw = lw,
    label = L"Foot 2: $p_z [m]$",
)

plt_vfeet1 = plot(; xlabel = L"$x$ [m]", layout = (2, 1), dpi = dpi)

plot!(
    plt_vfeet1[1],
    tplot,
    v_FL[:, 1];
    lw = lw,
    label = L"$v_x [m/s]$",
    title = "Foot 1",
)
plot!(
    plt_vfeet1[1],
    tplot,
    a_FL[:, 1];
    lw = lw,
    label = L"$a_x [m/s^2]$",
)

plot!(
    plt_vfeet1[2],
    tplot,
    v_FL[:, 3];
    lw = lw,
    label = L"$v_z [m/s]$",
)
plot!(
    plt_vfeet1[2],
    tplot,
    a_FL[:, 3];
    lw = lw,
    label = L"$a_z [m/s^2]$",
)
plt_vfeet2 = plot(; xlabel = L"$x$ [m]", layout = (2, 1), dpi = dpi)

plot!(
    plt_vfeet2[1],
    tplot,
    v_FR[:, 1];
    lw = lw,
    label = L"$v_x [m/s]$",
    title = "Foot 2",
)
plot!(
    plt_vfeet2[1],
    tplot,
    a_FR[:, 1];
    lw = lw,
    label = L"$a_x [m/s^2]$",
)

plot!(
    plt_vfeet2[2],
    tplot,
    v_FR[:, 3];
    lw = lw,
    label = L"$v_z [m/s]$",
)
plot!(
    plt_vfeet2[2],
    tplot,
    a_FR[:, 3];
    lw = lw,
    label = L"$a_z [m/s^2]$",
)

plt_ankle = plot(; xlabel = L"$t$ [s]", layout = (2, 1), dpi = dpi)

plot!(
    plt_ankle[1],
    tsim,
    qsim[:, end-1]; 
    lw = lw,
    label ="Simulated",
)

plot!(
    plt_ankle[1],
    tplot,
    q_ref[7, :];
    lw = lw,
    label = "Reference",
    title = "Ankle left",
)

plot!(
    plt_ankle[2],
    tsim,
    qsim[:, end]; 
    lw = lw,
    label ="Simulated",
)

plot!(
    plt_ankle[2],
    tplot,
    q_ref[8, :];
    lw = lw,
    label = "Reference",
    title = "ankle right ",
)


plt_timing = plot(
    xlabel = L"$t$ [s]",
    xlims = (0, tend),
    layout = (1, 1),
    dpi = dpi
)

# Scatter plot of computation time
scatter!(
    plt_timing,
    tplot[2:end-1],
    computation_time[2:end];
    ylabel = latexstring("Computation") * " "*  latexstring("time")* " [s]",
    title = "Computation Time (Horizon: $(br.predictionTime)s)",
    label = "Online planning step",
    ms = 2
)

# Mean computation time
mean_timing = fill(sum(computation_time) / length(computation_time), length(computation_time))
plot!(
    plt_timing,
    tplot[1:end-1],
    mean_timing;
    label = "Mean = $(round(mean_timing[1]; digits=3)) s",
    lw = lw,
    c = :red
)

if PLOT_ADDITIONAL
    display(plt_x_f)
    display(plt_z_f)
    display(plt_feet1)
    display(plt_feet2)
    display(plt_ankle)
    display(plt_pfeet)
    display(plt_vfeet1)
    display(plt_vfeet2)
    ZMProbot.plot_IK(ik, tplot, dpi, lw; save=SAVE_FIG, savePath=save_dir)
    display(plt_timing)
    if SAVE_FIG
        savefig(plt_x_f, save_dir*"plt_x_f.png")
        savefig(plt_z_f, save_dir*"plt_z_f.png")
        savefig(plt_feet1, save_dir*"plt_feet1.png")
        savefig(plt_feet2, save_dir*"plt_zoom_zmp.png")
        savefig(plt_ankle, save_dir*"plt_ankle.png")
        savefig(plt_pfeet, save_dir*"plt_pfeet.png")
        savefig(plt_vfeet1, save_dir*"plt_vfeet1.png")
        savefig(plt_vfeet2, save_dir*"plt_vfeet2.png")
        savefig(plt_timing, save_dir*"plt_timing.png")
    end
end 
println("\n✅ Simulation successfully exectuted.\n")
