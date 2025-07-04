## Import necessary packages
using LinearAlgebra                     # Basic linear algebra
using Plots                             # Plotting utilities
using RigidBodyDynamics                 # Robot dynamics engine
using MeshCat, MeshCatMechanisms, Blink # 3D visualization tools
using LaTeXStrings                      # Display LaTeX in plots

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
SAVE_FIG         = false;
ZOOM             = false;
lw, dpi, msw, ms = 2, 600, 0.1, 2
if INTERACTIVE_PLOT; plotlyjs(); else; gr(); end
save_dir = project_dir * "./Plot/Second_Control_Approach/Simu/" # Directory (relative to project root) for saving simulation plots

# Visualiser parameter
ANIMATE_RESULT = true;

# Simulation parameter
GRAVITY         = true;
CONTACTS        = false;    # Due to incompatibility
GROUND          = true;
ENHANCED_MODEL  = true;  

# Save result 
SAVE_CSV     = true;
ref_fileName = save_dir * "walkingPattern_ref.csv"     # file name to save

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

# Define actuation matrix: B maps control inputs to accelerations
B = ZMProbot.eye(nq, T=Float64)

# Define foot 1 spatial Jacobian 
J_f1 = q-> [0 0 0 0 0 0 0 0; 
            0 0 1 0 1 0 1 0; 
            0 0 0 0 0 0 0 0; 
            f_J_F[1](q)
]

# Define foot 2 spatial Jacobian 
J_f2 = q-> [0 0 0 0 0 0 0 0; 
            0 0 0 1 0 1 0 1; 
            0 0 0 0 0 0 0 0; 
            f_J_F[2](q)
]

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
zt, cf = ZMProbot.ZMPTrajectory(; br = br, fp = fp, check = PLOT_OFFLINE, save=SAVE_FIG, savePath=save_dir)

# Retrieve ZMP and Time vector 
ZMP  = reduce(hcat, zt.ZMP)
Time = reduce(vcat, zt.timeVec) 
tend = Time[end]
println("✅ ZMP trajectory generator.")

# Generate the Swing Foot trajectory 
sf = ZMProbot.SwingFootTrajectory(; br = br, fp = fp, zt = zt, check = PLOT_OFFLINE, save=SAVE_FIG, savePath=save_dir)

# Retrieve both feet position trajectories
p_FL = reduce(hcat, sf.stepL)
p_FR = reduce(hcat, sf.stepR)

# Compute both feet velocity trajectories
v_FL = [ZMProbot.differentiate(p_FL[1, :], br.Ts; init_nul=true)'; ZMProbot.differentiate(p_FL[2, :], br.Ts; init_nul=true)';  ZMProbot.differentiate(p_FL[3, :], br.Ts; init_nul=true)']
v_FR = [ZMProbot.differentiate(p_FR[1, :], br.Ts; init_nul=true)'; ZMProbot.differentiate(p_FR[2, :], br.Ts; init_nul=true)';  ZMProbot.differentiate(p_FR[3, :], br.Ts; init_nul=true)']

# Compute both feet acceleration trajectories
a_FL = [ZMProbot.differentiate(v_FL[1, :], br.Ts; init_nul=true)'; ZMProbot.differentiate(v_FL[2, :], br.Ts; init_nul=true)';  ZMProbot.differentiate(v_FL[3, :], br.Ts; init_nul=true)']
a_FR = [ZMProbot.differentiate(v_FR[1, :], br.Ts; init_nul=true)'; ZMProbot.differentiate(v_FR[2, :], br.Ts; init_nul=true)';  ZMProbot.differentiate(v_FR[3, :], br.Ts; init_nul=true)']
println("✅ Foot trajectories.")


###########################################################
#                      Online Planning                    #
###########################################################
println("\n○ Online planning definition")

# Define MPC parameters 
nx     = 2*nq
N      = trunc(Int, br.predictionTime / br.Ts)
Q      = ZMProbot.eye(2, T=Float64)             # Weight on reference tracking 
R      = ZMProbot.eye(nq, T=Float64) #* 1.0e-3   # Weight on control inputs

# Initial CoM state
p_init_com = center_of_mass(rs.state).v

# CoM lowering phase: construct the CoM trajectory along z-axis
z_coeff = ZMProbot.getSplineCoeff(zt.timeVec[1][1], zt.timeVec[1][end], p_init_com[3], br.zc)                               
zc       = vcat(ZMProbot.spline(zt.timeVec[1], z_coeff), br.zc * ones(length(Time) - length(zt.timeVec[1])))
println("✅ CoM lowering phase.")

# Define dynamics evolution, observation, and objective functions
F_xu = ZMProbot.NL_stateTransitionFunction(nq, N, fk, fd)
G_x  = ZMProbot.NL_observationFunction(N, fk.F, fk.CoM, fk.J_CoM, fk.J̇_CoM, fk.J_F, fk.J̇_F, nq, p_FL, p_FR, v_FL, v_FR, a_FL, a_FR)
O_xuyr = ZMProbot.objectiveFunction(Q, R)

# Define additional optimised value extraction
q̈sim = []
valueFunction = [(ZMProbot.getAddValue, (nq, q̈sim))]

# Define reference 
MPC_ref = vcat(ZMP[1, :]', zc')

# Create MPC structure
mpcParam = ZMProbot.defineMpcParameter(
                                        nx, 
                                        nq,  
                                        F_xu, 
                                        G_x,
                                        Time, 
                                        N, 
                                        MPC_ref, 
                                        O_xuyr,
                                        :Min; 
                                        optn=(("print_level", 0), ("tol", 1e-6)),# ("tol", 1e-8), ("acceptable_tol", 1e-10)), # ("tol", 1e-6)#(("verbose", false), ("eps_abs", 1e-3), ("eps_rel", 1e-3), ("warm_start", true)),
                                        Type=1
)
println("✅ MPC.")

###########################################################
#                  Code Initialisation & Memory           #
###########################################################

## MPC variable initialisation
q0 = configuration(rs.state)
v0 = velocity(rs.state)
x0 = vcat(q0, v0) 
Γ0 = zeros(nq)

## Memory
# Simulation  
qsim = Array{}[]
vsim = Array{}[]
tsim = Array{}[]
usim = Array{}[]

# Simulation deviation from expected results   
esim        = Array{}[]
Contact_F1  = Array{}[]
Contact_F2  = Array{}[]

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

    # Compute optimal generalised control vector and corresponding state evolution
    x, Γ = ZMProbot.computeMPControl(mpcParam, x0, Γ0; valueFunctions=valueFunction)

    # Extract first element 
    global Γ0 .= Γ[:, 1]

    ##############################
    # 2. Ground Reaction Wrenches
    ##############################

    # Extract horizontal and vertical components of the ground force
    F = [Γ0[1]; Γ0[2]]

    # Initialise actuator commands vector 
    τ = Float64.(Γ0)

    # Evaluate the spatial Jacobians for each foot
    J1 = J_f1(x0[1:nq])
    J2 = J_f2(x0[1:nq])

    # Extract usefull part of Jacobian (used for force and torque projections)
    (αx, αz) = J1[4:2:6, 7]
    (βx, βz) = J2[4:2:6, 8]
    J1_T     = J1[2:2:6, :]'
    J2_T     = J2[2:2:6, :]'
    
    # Retrieve vertical position of each foot
    f1_z = f_F[1](x0[1:nq])[3]
    f2_z = f_F[2](x0[1:nq])[3]

    ##################################
    # 3. Actuator commands computation
    ##################################

    if f1_z <= 1e-3 && f2_z <=1e-3                  # Double support phase: both feet are in contact with the ground
        
        # Uniformly distribute ground reaction forces
        F = F ./ 2
        push!(Contact_F1, F)
        push!(Contact_F2, F)

        # Compute residual reaction torque at ankle contact point for each foot
        C_y1 = Γ0[7] - dot([αx αz], F)
        C_y2 = Γ0[8] - dot([βx βz], F)

        # Assemble full contact wrench vectors
        F_f1 = vcat(C_y1, F)
        F_f2 = vcat(C_y2, F)

        # Estimate actuators commands  
        τ -= (J1_T * F_f1 + J2_T * F_f2)

    elseif f1_z <= 1e-3                         # Single support phase on foot 1
        
        # Compute residual reaction torque at ankle contact point for foot 1
        C_y1 = Γ0[7] - dot([αx αz], F)

        # Assemble full contact wrench vector
        F_f1 = vcat(C_y1, F)

        # Estimate actuators commands
        τ -= J1_T * F_f1

        # Store contact contact forces 
        push!(Contact_F1, F)
        push!(Contact_F2, zero(F))

    elseif f2_z <= 1e-3                         # Single support phase on foot 2

        # Compute residual reaction torque at ankle contact point for foot 1
        C_y2 = Γ0[8] - dot([βx βz], F)

        # Assemble full contact wrench vector
        F_f2 = vcat(C_y2, F)

        # Estimate actuators commands
        τ -= J2_T * F_f2

        # Store contact contact forces 
        push!(Contact_F1, zero(F))
        push!(Contact_F2, F)
    else 
        # No foot in contact — invalid support phase
        return error("⚠️ Invalid contact phase: both feet are above height threshold")
    end 

    # if f_F[1](x0[1:nq])[3] <= 1e-3 && f_F[2](x0[1:nq])[3] <=1e-3
    #     F = F ./ 2
    #     torques -= (J1_bis*F + J2_bis*F)
    #     # torques -= (J1*F + J2*F)
    # elseif f_F[1](x0[1:nq])[3] <= 1e-3
    #     torques -= J1_bis*F
    # elseif f_F[2](x0[1:nq])[3] <= 1e-3
    #     torques -= J2_bis*F
    # else 
    #     return error("both feet above tolerance")
    # end 
    println("• Command Torques: ", τ)

    # Stop timer & measure elapsed time
    end_time = time()
    push!(computation_time, end_time - start_time)

    ############################
    # 4. Simulation and Control
    ############################

    # Simulate robot dynamics using generalised control vector
    controller! = ZMProbot.torques_controller!(rs, Γ0, t, br.Ts, Δt, endEffector)
    # controller! = ZMProbot.torques_controller!(rs, torques, t, br.Ts, Δt, endEffector)
    # controller! = ZMProbot.acceleration_controller!(rs, br.Ts, t, q̈sim[end], Δt, endEffector)
    ts, qs, vs = RigidBodyDynamics.simulate(rs.state, br.Ts, controller!; Δt = Δt);

    # Store simulation results
    if t == 0.0
        push!(qsim, qs[1:end])
        push!(vsim, vs[1:end])
        push!(tsim, ts[1:end] .+ t)
    else
        push!(qsim, qs[2:end])
        push!(vsim, vs[2:end])
        push!(tsim, ts[2:end] .+ t)
    end
    push!(usim, τ)

    #####################
    # 5. Sensor Feedback 
    #####################

    # Update state for next iteration 
    local q0   = configuration(rs.state)
    global v0  = velocity(rs.state)
    global x0 .= vcat(q0, v0)
     
    println("• Actual configuration :", x0)

    ####################
    # 6. Additional Log
    ####################
    
    # Deviations from MPC's expected state
    push!(esim, x[:, 1] - x0)

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
ZMP_x = reduce(vcat, ZMP[1, :])
ZMP_y = reduce(vcat, ZMP[2, :])

# Simulation results
vsim            = reduce(hcat, (reduce(vcat, vsim)))'
q̈sim            = reduce(hcat, q̈sim)'
qsim            = reduce(hcat, (reduce(vcat, qsim)))'
tsim            = reduce(vcat, tsim)
usim            = reduce(hcat, usim)
torque_sim      = reduce(hcat, rs.torques)
CoMsim          = reduce(hcat, rs.CoM)
Contact_F1      = reduce(hcat, Contact_F1)
Contact_F2      = reduce(hcat, Contact_F2)

# Compute CoM, feet and ZMP value based on joint simulation measure. 
zmp_LIPM    = [0.0]
zmp_VHLIPM  = [0.0]
p_foot1     = [0.0; 0.0]
p_foot2     = [0.0; 0.0]
CȯMsim      = [0.0; 0.0; 0.0]
CöMsim      = [0.0; 0.0; 0.0]

# Estimate the corresponding ZMP using VH-LIPM
k = 1
for (i, q) in enumerate(eachrow(qsim[2:end, :]))
    if i > k * Int64.((size(vsim, 1)-1) / size(q̈sim, 1))
        global k+=1
    end 

    # Compute CoM position, Jacobian and Hessian 
    com  = f_CoM(q)
    Jcom = f_J_CoM(q)
    J̇com = f_J̇_CoM(q) 

    # Compute CoM velocity and acceleration 
    cȯm = Jcom * vsim[i, :]
    cöm = similar(cȯm)
    add = Jcom * q̈sim[k, :]
    for j in 1:3
        cöm[j] = vsim[i, :]' * J̇com[j] * vsim[i, :] + add[j]
    end 

    # Estimate ZMP using LIPM and VHIPM 
    x_zmp_vhlipm = com[1] - (com[3] / (9.81 + cöm[3])) * cöm[1]
    x_zmp_lipm   = com[1] - (com[3] / 9.81) * cöm[1]
    
    # Store results 
    push!(zmp_VHLIPM, x_zmp_vhlipm)
    push!(zmp_LIPM, x_zmp_lipm)
    global CȯMsim = hcat(CȯMsim, cȯm)
    global CöMsim = hcat(CöMsim, cöm)
    global p_foot1 = hcat(p_foot1, [f_F[1](q)[1]; f_F[1](q)[3]])
    global p_foot2 = hcat(p_foot2, [f_F[2](q)[1]; f_F[2](q)[3]])
end 

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
        plot!(
            plt_θ[joint, side_idx],
            tsim,
            qsim[:, (offset - 1) + (2 * joint  - 1 ) + side_idx]; # de 3 à 6
            lw = lw,
            label = "Simulated",
        )
        str = latexstring("q_{$(joint + 2)$(side_idx)}") * " [rad]"
        ylabel!(plt_θ[joint, side_idx], str)
        
        str = latexstring("q̇_{$(joint + 2)$(side_idx)}") * " [rad/s]"
        ylabel!(plt_ω[joint, side_idx], str)

        str = latexstring("τ_{$(joint + 2)$(side_idx)}") * " [Nm]"
        ylabel!(plt_τ[joint, side_idx], str)
        plot!(
            plt_τ[joint, side_idx],
            tplot[1:end-1],
            usim[(offset - 1) + (2 * joint  - 1 ) + side_idx, :];
            lw = lw,
            label = "Simulated",
            # title = "$(side) $(name)",
        )
        plot!(
            plt_ω[joint, side_idx],
            tsim,
            vsim[:, (offset - 1) + (2 * joint  - 1 ) + side_idx];
            lw = lw,
            label = "Simulated",
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
plt_com = plot(; xlabel = L"$t$ [s]", xlims = (0, tend), layout = (2, 1), dpi = dpi)
plot!(
    plt_com[1],
    tsim[1:len_t],
    CoMsim[1, 1:len_sim];
    lw = lw,
    label = "Simulated",
    ylabel =  L"x_c" *" [m]",
    #title = "CoMx",
)
plot!(
    plt_com[2],
    tsim[1:len_t],
    CoMsim[3, 1:len_sim];
    lw = lw,
    label = "Simulated",
    ylabel =  L"z_c" *" [m]",
    # title = "CoMz",
    lgend=false
)
plot!(plt_com[2], tplot, zc; label = "Reference", lw = lw)


plt_cȯm = plot(; xlabel = L"$t$ [s]", xlims = (0, tend), layout = (2, 1), dpi = dpi)
plot!(
    plt_cȯm[1],
    tsim[1:len_t],
    CȯMsim[1, 1:len_sim];
    lw = lw,
    label = "Simulated",
    ylabel =  L"\dot{x}_c" *" [m/s]",
    # title = "CȯMx",
)
plot!(
    plt_cȯm[2],
    tsim[1:len_t],
    CȯMsim[3, 1:len_sim];
    lw = lw,
    # label = "Simulated",
    ylabel =  L"\dot{z}_c" *" [m/s]",
    # title = "CȯMz",
    legend=false
)

plt_cöm = plot(; xlabel = L"$t$ [s]", xlims = (0, tend), layout = (2, 1), dpi = dpi)
plot!(
    plt_cöm[1],
    tsim[1:len_t],
    CöMsim[1, 1:len_sim];
    lw = lw,
    label = "Simulated",
    ylabel =  L"\ddot{x}_c" * " "*L"[m/$s^2$]",
    # title = "CöMx",
)
plot!(
    plt_cöm[2],
    tsim[1:len_t],
    CöMsim[3, :];
    lw = lw,
    # label = "Simulated",
    ylabel =  L"\ddot{z}_c" * " "*L"[m/$s^2$]",
    # title = "CöMz",
    legend=false
)

# ------------------
#     ZMP Plot 
# ------------------
plt_zmp = plot(; xlabel = L"$t$ [s]", xlims = (0, tend), layout = (1, 1), dpi = dpi)
plot!(
    plt_zmp,
    tsim[1:len_t],
    zmp_VHLIPM;
    lw = lw,
    label = latexstring("p_{sim, x}")*" Meas. with VHIPM",
    ylabel = L"$X$ [m]",
    # title = "ZMPx",
)
plot!(plt_zmp, tplot, ZMP_x; label=L"p_{\mathrm{ref, x}}", lw = lw)

# Computed actual stability support area to measure if simulated ZMP leaves it
zmplim = []
halftfootLength_x = br.footLength_x / 2
ε = 1e-3
for k in 1:round(Int, br.Ts/Δt):size(p_foot1, 2)
    pf1 = p_foot1[:, k]
    pf2 = p_foot2[:, k]
    if (pf1[2] <= ε && pf2[2] <= ε) # DSP 
        if pf1[1] > pf2[1]  # left foot in front 
            push!(zmplim, [pf1[1] + halftfootLength_x, pf2[1] - halftfootLength_x])
        else                # right foot in front 
            push!(zmplim, [pf2[1] + halftfootLength_x, pf1[1] - halftfootLength_x])
        end 
    elseif pf1[2] <= ε
        push!(zmplim, [pf1[1] + halftfootLength_x, pf1[1] - halftfootLength_x])
    else 
        push!(zmplim, [pf2[1] + halftfootLength_x, pf2[1] - halftfootLength_x])
    end
end 
zmplim = reduce(hcat, zmplim) 

plot!(plt_zmp, tplot, zmplim[1, :]; linestyle=:dash, color=:red, label="Sim. stability area")
plot!(plt_zmp, tplot, zmplim[2, :]; linestyle=:dash, color=:red, label=nothing)

# ------------------
#     Reaction Plot 
# ------------------
plt_Force = plot(; layout = (2, 1), dpi = dpi)
plot!(
        plt_Force[1],
        tplot[1:end-1],
        Contact_F1[1, :]; 
        label = "Left foot",
        ylabel = L"$f_{t,x}$"*" [N]",
        # title = "Ground Reaction Force [N]"
    )
plot!(
        plt_Force[1],
        tplot[1:end-1],
        Contact_F2[1, :];
        label = "Right foot",
)
plot!(
        plt_Force[2],
        tplot[1:end-1],
        Contact_F1[2, :];
        legend=false
)
plot!(
        plt_Force[2],
        tplot[1:end-1],
        Contact_F2[2, :];
        xlabel = L"$t$ [s]",
        ylabel=L"$f_{n}$"*" [N]",
        legend=false
)

# ------------------
#     Foot Plots 
# ------------------
plt_x_f = plot(; xlabel = L"$t$ [s]", layout = (2, 1), dpi = dpi)

plot!(
    plt_x_f[1],
    tsim[1:len_t],
    p_foot1[1, :];
    lw = lw,
    label ="Simulated",
    ylabel = L"$x_{f1}$ [m]",
    #title = "Foot 1",
)
plot!(plt_x_f[1], tplot, p_FL[1, :]; label = "Reference", lw = lw)
plot!(
    plt_x_f[2],
    tsim[1:len_t],
    p_foot2[1, :];
    lw = lw,
    #label ="Simulated",
    ylabel = L"$x_{f2}$ [m]",
    #title = "Foot 1",
    legend=false
)
plot!(plt_x_f[2], tplot, p_FR[1, :]; #label = "Reference",
 lw = lw, legend=false)

plt_z_f = plot(; xlabel = L"$t$ [s]", layout = (2, 1), dpi = dpi)

plot!(
    plt_z_f[1],
    tsim[1:len_t],
    p_foot1[2, :];
    lw = lw,
    label ="Simulated",
    ylabel = L"$z_{f1}$ [m]",
    #title = "Foot 1",
)
plot!(plt_z_f[1], tplot, p_FL[3, :]; label = "Reference", lw = lw)
plot!(
    plt_z_f[2],
    tsim[1:len_t],
    p_foot2[2, :];
    lw = lw,
    #label ="Simulated",
    ylabel = L"$z_{f2}$ [m]",
    #title = "Foot 1",
    legend=false
)
plot!(plt_z_f[2], tplot, p_FR[3, :]; #label = "Reference",
 lw = lw, legend=false)


# ------------------
#     Timing Plot 
# ------------------
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

if PLOT_SIMU 
    display(plt_θ)
    display(plt_τ)
    display(plt_ω)
    display(plt_com)
    display(plt_cȯm)
    display(plt_cöm)
    display(plt_zmp)
    display(plt_Force)  
    display(plt_x_f)
    display(plt_z_f)
    display(plt_timing)
end

if SAVE_FIG
    savefig(plt_zmp, save_dir * "zmp.png")
    savefig(plt_com, save_dir * "com.png")
    savefig(plt_cȯm, save_dir * "cȯm.png")
    savefig(plt_cöm, save_dir * "cöm.png")
    savefig(plt_θ, save_dir * "θ.png")
    savefig(plt_τ, save_dir * "τ.png")
    savefig(plt_ω, save_dir * "ω.png")
    savefig(plt_Force, save_dir * "plt_F.png")
    savefig(plt_z_f, save_dir * "plt_z_f.png")
    savefig(plt_x_f, save_dir * "plt_x_f.png")
    savefig(plt_timing, save_dir * "timing.png")
end 

if ZOOM 
    xlims!(plt_zmp, 10, 21)
    savefig(plt_zmp, save_dir * "zmp_walking_phase.png")

    xlims!(plt_zmp, 11, 13)
    ylims!(plt_zmp, -0.05, 0.3)
    savefig(plt_zmp, save_dir * "zmp_two_steps.png")

    xlims!(plt_com, 10, 21)
    # ylims!(plt_com[2], 0.191, 0.194)
    # ylims!(plt_com[2], 0.191, 0.205)
    savefig(plt_com, save_dir * "com_walking_phase.png")
    
    xlims!(plt_com, 11, 13)
    ylims!(plt_com[1], 0, 0.26)
    # ylims!(plt_com[2], 0.191, 0.194)
    # ylims!(plt_com[2], 0.191, 0.205)
    savefig(plt_com, save_dir * "com_two_steps.png")

    xlims!(plt_cȯm, 10, 21)
    savefig(plt_cȯm, save_dir * "cȯm_walking_phase.png")

    xlims!(plt_cȯm, 11, 13)
    ylims!(plt_cȯm[1], 0.0, 0.3)
    # ylims!(plt_cȯm[2], -0.1,0.1)
    savefig(plt_cȯm, save_dir * "cȯm_two_steps.png")

    xlims!(plt_cöm, 10, 21)
    # ylims!(plt_cöm[2], -0.5, 0.5)
    savefig(plt_cöm, save_dir * "cöm_walking_phase.png")

    xlims!(plt_cöm, 11, 13)
    # ylims!(plt_cöm[1], -2.5, 2.5)
    # ylims!(plt_cöm[1], -0.5, 0.5)
    # # ylims!(plt_cöm[2], -7, 10)
    # ylims!(plt_cöm[2], -0.4, 0.4)
    savefig(plt_cöm, save_dir * "cöm_two_steps.png")

    xlims!(plt_θ, 10, 21)
    # ylims!(plt_θ[1], -1.25, -0.3)
    # ylims!(plt_θ[2], -1.25, -0.3)
    # ylims!(plt_θ[3], 0.8, 1.45)
    # ylims!(plt_θ[4], 0.8, 1.45)
    savefig(plt_θ, save_dir * "θ_walking_phase.png")
    
    xlims!(plt_θ, 11, 13)
    savefig(plt_θ, save_dir * "θ_two_steps.png")

    xlims!(plt_τ, 10, 21)
    savefig(plt_τ, save_dir * "τ_walking_phase.png")
    
    xlims!(plt_τ, 11, 13)
    # ylims!(plt_τ[1], -3, 0.5)
    # ylims!(plt_τ[2], -3, 0.5)
    # ylims!(plt_τ[3], -3, 0.5)
    # ylims!(plt_τ[4], -3, 0.5)
    savefig(plt_τ, save_dir * "τ_two_steps.png")

    xlims!(plt_τ, 16, 18)
    # ylims!(plt_τ[1], -7, 0.5)
    # ylims!(plt_τ[2], -7, 0.5)
    # ylims!(plt_τ[3], -8, 0.5)
    # ylims!(plt_τ[4], -8, 0.5)
    savefig(plt_τ, save_dir * "τ_two_steps2.png")

    xlims!(plt_ω, 10, 21)
    savefig(plt_ω, save_dir * "ω_walking_phase.png")    

    xlims!(plt_ω, 11, 13)
    savefig(plt_ω, save_dir * "ω_two_steps.png")

    xlims!(plt_z_f, 11, 13)
    savefig(plt_z_f, save_dir * "z_f_two_steps.png")

    display(plt_θ)
    display(plt_τ)
    display(plt_ω)
    display(plt_com)
    display(plt_cȯm)
    display(plt_cöm)
    display(plt_zmp)
    display(plt_Force)  
    display(plt_x_f)
    display(plt_z_f)
    display(plt_timing)
end


###########################################################
#                      Save results                       #
###########################################################

if SAVE_CSV
    println("\n○ Saving results")
    # define headers for each column 
    ref_header = ["time" "q31" "q32" "q41" "q42" "τ31" "τ32" "τ41" "τ42"]

    # Extract reference joint position
    qref = []
    for k in 1:round(Int, br.Ts/Δt):size(qsim, 1)
        push!(qref, qsim[k, offset+1:offset+4])
    end 
    qref = reduce(hcat, qref)

    # combine arrays into a table
    ref_data = hcat(
        tplot[2:end],
        qref[1, 2:end],
        qref[2, 2:end],
        qref[3, 2:end],
        qref[4, 2:end],
        usim[offset+1, :],
        usim[offset+2, :],
        usim[offset+3, :],
        usim[offset+4, :]
    )
    ref_data2store = [ref_header; ref_data]

    # Save into a csv file 
    open(ref_fileName, "w") do file
        CSV.write(ref_fileName, Tables.table(ref_data2store); delim = ',')
        return println("✅ File saved at $(ref_fileName)")
    end 
end 

println("\n✅ Simulation successfully exectuted.\n")
