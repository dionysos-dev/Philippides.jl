"""
    RobotSimulator

# Description:
Defines a robot simulation environment.  
The `RobotSimulator` struct encapsulates all relevant simulation data,  
including the kinematic mechanism, internal state, and key dynamic quantities.

# Outputs:
A `RobotSimulator` provides the following simulated outputs:

1. `mechanism` : A `Mechanism` object that holds the robot model and structure.
2. `state`     : The current `MechanismState`, storing joint positions and velocities.
3. `CoM`       : Array storing the simulated Center of Mass (CoM) trajectory.
4. `CȯM`       : Array storing the simulated CoM velocity trajectory.
5. `CöM`       : Array storing the simulated CoM acceleration trajectory.
6. `torques`   : Array of simulated joint torques over time.
7. `index`     : Current simulation time index.
8. `time`      : Array of time steps corresponding to the simulation horizon.
"""
mutable struct RobotSimulator
    mechanism::Mechanism
    state::MechanismState
    CoM::Array
    CȯM::Array
    CöM::Array
    torques::Array
    index::Int64
    time::Array
end

"""
    RobotSimulator(; filePath, symbolic, use_urdf, T, add_contact_points, add_flat_ground, add_gravity, contactmodel, contactPoints, endEffector)

# Description:
Initializes a `RobotSimulator` object by loading the robot mechanism from a URDF file or symbolic definition.  
This constructor sets up the robot structure, state, and prepares all output fields for simulation (CoM, torques, etc.).

# Arguments:
- `filePath::String = "../../deps/ZMP_bipedalRobot.urdf"` : Path to the robot URDF file.
- `symbolic::Bool = false`                                : Use symbolic computation (e.g., for symbolic dynamics).
- `use_urdf::Bool = !symbolic`                            : Load from URDF if true, otherwise use manual definition.
- `T::Type = symbolic ? Num : Float64`                    : Scalar type for computation (e.g., `Float64` or `Num`).
- `add_contact_points::Bool = true`                       : Whether to add contact points to the robot model.
- `add_flat_ground::Bool = true`                          : Whether to include a flat ground plane in the model.
- `add_gravity::Bool = true`                              : Whether to add gravity to the model.
- `contactmodel = default_contact_model()`                : Contact model used for simulating foot-ground interaction.
- `contactPoints::Vector{Array} = []`                     : Optional list of custom contact points (in local body frame).
- `endEffector::Tuple = ()`                               : Optional end-effector specification (body, frame).

# Returns:
- `simulator::RobotSimulator` : A new instance of the robot simulator with initialized mechanism and empty simulation buffers.
"""
function RobotSimulator(;
    filePath::String = "../../deps/ZMP_bipedalRobot.urdf",
    symbolic::Bool = false,
    use_urdf::Bool = !symbolic,
    T::Type = symbolic ? Num : Float64,
    add_contact_points = true,
    add_flat_ground = true,
    add_gravity = true,
    contactmodel = contact_model(),
    contactPoints ::Vector{Array} = [],
    endEffector::Tuple = ()
)
    robot, state = getMechanism(;
        filePath,
        symbolic,
        use_urdf,
        T,
        add_contact_points,
        add_flat_ground,
        add_gravity,
        contactmodel,
        contactPoints=contactPoints,
        endEffector=endEffector
    )
    return RobotSimulator(robot, state, Array[], Array[], Array[], Array[], 1, Array[])
end

"""
    contact_model(; kn, α, n, μ, kt, λt)

# Description:
Returns a parameterized `SoftContactModel` combining:
- A Hunt-Crossley model for the normal force
- A Viscoelastic Coulomb model for the tangential force

Useful for fine-tuning the foot-ground interaction in simulation.

# Arguments:
- `kn::Float64 = 50e3`      : Normal stiffness coefficient [N/mⁿ].
- `α::Float64 = 0.2`        : Damping factor in normal direction.
- `n::Float64 = 3/2`        : Exponent for the normal force law (Hertzian = 1.5).
- `μ::Float64 = 0.8`        : Coefficient of friction.
- `kt::Float64 = 20e3`      : Tangential stiffness coefficient [N/m].
- `λt::Float64 = 100.0`     : Tangential damping coefficient [Ns/m].

# Returns:
- `model::SoftContactModel` : Combined soft contact model with specified parameters.
"""
function contact_model(;kn=50e3, α=0.2, n=3/2, μ=0.8, kt=20e3, λt=100.0)
    λn = 3/2 * α * kn 
    return SoftContactModel(
        HuntCrossleyModel(kn, λn, n), 
        ViscoelasticCoulombModel(μ, kt, λt),
    )
end 

"""
    getMechanism(; filePath, symbolic, use_urdf, T, add_contact_points, add_flat_ground, add_gravity, contactmodel, contactPoints, endEffector)

# Description:
Builds and returns a `Mechanism` and its initial state based on a URDF robot model.  
If `add_contact_points` is enabled, specified contact points are added to the selected end effectors.  
A flat ground can also be added as a half-space primitive for contact simulation.  
This function is primarily used to prepare the robot model for simulation or control.

# Arguments:
- `filePath::String = "../../deps/ZMP_bipedalRobot.urdf"` : Path to the URDF file.
- `symbolic::Bool = false`                                : Enable symbolic type (`Num`) if true.
- `use_urdf::Bool = !symbolic`                            : Parse from URDF if true (default unless symbolic).
- `T::Type = symbolic ? Num : Float64`                    : Scalar type used for configuration.
- `add_contact_points::Bool = true`                       : Add contact points to the feet if true.
- `add_flat_ground::Bool = true`                          : Add flat ground primitive for contact simulation.
- `add_gravity::Bool = true`                              : Apply gravity to the model.
- `contactmodel = default_contact_model()`                : Contact model to assign to the points.
- `contactPoints::Vector{Array} = []`                     : List of 3D contact point positions in world frame.
- `endEffector::Tuple = ()`                               : Names of end-effector links to attach contact points to.

# Returns:
- `mechanism::Mechanism`        : Robot mechanism as parsed from URDF and optionally modified.
- `state::MechanismState`       : Associated mechanism state with default configuration and zero velocity.
"""
function getMechanism(;
    filePath::String = "../../deps/ZMP_bipedalRobot.urdf",
    symbolic::Bool = false,
    use_urdf::Bool = !symbolic,
    T::Type = symbolic ? Num : Float64,
    add_contact_points = true,
    add_flat_ground = true,
    add_gravity = true,
    contactmodel = default_contact_model(),
    contactPoints ::Vector{Array} = [],
    endEffector::Tuple = ()
)
    # Define the robot mechanism
    if use_urdf
        mechanism = RigidBodyDynamics.parse_urdf(filePath)
        remove_fixed_tree_joints!(mechanism)

        # Initiate state to nominal - so the contact points are properly define
        state = MechanismState(mechanism)
        nq = num_velocities(mechanism)
        q0 = zeros(nq)
        set_configuration!(state, q0)
        zero_velocity!(state)

        if (add_gravity)
            mechanism.gravitational_acceleration =
                FreeVector3D(root_frame(mechanism), 0, 0, -9.81)
        else
            mechanism.gravitational_acceleration =
                FreeVector3D(root_frame(mechanism), 0, 0, 0)
        end
    else
       return error("not available !")
    end
    # Configuration of the contact points
    if !symbolic && add_contact_points && contactmodel !== nothing
        for effector in endEffector
            #foot body and frame 
            foot_link = findbody(mechanism, "$effector")
            frame = default_frame(foot_link)

            # Get rotation from world to foot
            T_world_to_foot = transform_to_root(state, frame)
            R_world_to_foot = rotation(T_world_to_foot)
            for cp in contactPoints
                # Vector in world frame
                cp_world = SVector(cp[1], cp[2], cp[3])  # example
                # Express it in foot frame: apply inverse rotation
                cp_foot = R_world_to_foot' * cp_world 
                point = Point3D(frame, cp_foot[1], cp_foot[2], cp_foot[3])
                add_contact_point!(foot_link, ContactPoint(point, contactmodel))
            end 
        end
    end
    # Configuration of the ground
    if !symbolic && add_flat_ground
        frame = root_frame(mechanism)
        ground =
            HalfSpace3D(Point3D(frame, 0.0, 0.0, 0.0), FreeVector3D(frame, 0.0, 0.0, 1.0))
        add_environment_primitive!(mechanism, ground)
    end
    # create new global state variable 
    state = MechanismState(mechanism)
    return mechanism, state
end

"""
    set_nominal!(rs::RobotSimulator, vis::MechanismVisualizer, nominal_config::Vector)

# Description:
Initializes the robot simulator with a nominal joint configuration.  
Sets the mechanism state, zeros all velocities, updates the visualizer, and initializes  
simulation output buffers with the corresponding CoM, torque, ZMP, and time data.

# Arguments:
- `rs::RobotSimulator`           : The robot simulator to configure.
- `vis::MechanismVisualizer`     : The visualizer to update with the robot's configuration.
- `nominal_config::Vector`       : Vector of joint positions for the nominal configuration.

# Returns:
- None. The simulator and visualizer are updated in-place.
"""
function set_nominal!(
    rs::RobotSimulator,
    vis::MechanismVisualizer,
    nominal_config::Vector
)
    set_configuration!(rs.state, nominal_config)
    zero_velocity!(rs.state)
    set_configuration!(vis, configuration(rs.state))

    nddl = num_velocities(rs.mechanism)
    tau = zeros(nddl)
    com = center_of_mass(rs.state).v
    push!(rs.torques, tau)
    push!(rs.CoM, com)
    push!(rs.time, [0.0])
    return update_visulizer!(rs, vis)
end


"""
    set_visulalizer(; mechanism, filePath)

# Description:
Creates a `MechanismVisualizer` for the given robot mechanism using the URDF visual mesh.

# Arguments:
- `mechanism::Mechanism`                                    : The robot mechanism to visualize.
- `filePath::String = "../../deps/ZMP_2DBipedRobot.urdf"`   : Path to the URDF file used for visual appearance.

# Returns:
- `vis::MechanismVisualizer` : Visualizer linked to the given mechanism and URDF visuals.
"""
function set_visulalizer(; mechanism::Mechanism, filePath::String = "../../deps/ZMP_2DBipedRobot.urdf")
    vis = MechanismVisualizer(mechanism, URDFVisuals(filePath))
    return vis
end

"""
    show_contact_point(state, vis, mechanism; endEffector, contactPoints)

# Description:
Displays contact points in the visualizer by projecting them from the world frame  
into the local frame of the corresponding foot (or end-effector). Each point is rendered  
as a small sphere in the visualization.

# Arguments:
- `state::MechanismState`                     : The current state of the robot.
- `vis::MechanismVisualizer`                  : Visualizer to which contact points will be added.
- `mechanism::Mechanism`                      : Robot mechanism containing the end-effector bodies.
- `endEffector::Tuple = ()`                   : Names of the end-effector bodies.
- `contactPoints::Vector{Array} = []`         : List of 3D points in local frame to be shown.

# Returns:
- None. The visualizer is updated in-place.
"""
function show_contact_point(state::MechanismState, vis::MechanismVisualizer, mechanism::Mechanism;  endEffector::Tuple = (), contactPoints ::Vector{Array} = [])
    for effector in endEffector

        #foot body and frame 
        foot_link = findbody(mechanism, "$effector")
        frame = default_frame(foot_link)

        # Get rotation from world to foot
        T_world_to_foot = transform_to_root(state, frame)
        R_world_to_foot = rotation(T_world_to_foot)

        for (i, cp) in enumerate(contactPoints)
            # Vector in world frame
            cp_world = SVector(cp[1], cp[2], cp[3])  # example
            # Express it in foot frame: apply inverse rotation
            cp_foot = R_world_to_foot' * cp_world 
            point = Point3D(frame, cp_foot[1], cp_foot[2], cp_foot[3])
            setelement!(vis, point, 0.005, "cp$(i)")
        end 
    end
end 

"""
    update_visulizer!(rs::RobotSimulator, vis::MechanismVisualizer)

# Description:
Updates the visualizer configuration to match the current state of the robot.  
It synchronizes the displayed robot pose with the internal simulator state.

# Arguments:
- `rs::RobotSimulator`         : Robot simulator containing the updated mechanism state.
- `vis::MechanismVisualizer`   : Visualizer to update.

# Returns:
- None. The visualizer is updated in-place.
"""
function update_visulizer!(rs::RobotSimulator, vis::MechanismVisualizer)
    return set_configuration!(vis, RigidBodyDynamics.configuration(rs.state))
end

"""
    show_frame!(rs::RobotSimulator, vis::MechanismVisualizer)

# Description:
Displays the coordinate frames of all robot bodies in the visualizer.  
Useful for debugging model structure and understanding joint orientations.

# Arguments:
- `rs::RobotSimulator`         : Robot simulator containing the mechanism.
- `vis::MechanismVisualizer`   : Visualizer in which to display the frames.

# Returns:
- None. Coordinate frames are added to the visualizer in-place.
"""
function show_frame!(rs::RobotSimulator, vis::MechanismVisualizer)
    # Show the frame of each bodies
    robot_bodies = RigidBodyDynamics.bodies(rs.mechanism)
    for body in robot_bodies
        frame = RigidBodyDynamics.default_frame(body)
        setelement!(vis, frame)
    end
end

"""
    position_controller!(
        rs::RobotSimulator,
        tend::Float64,
        t_actu::Float64,
        qref::Vector,
        q̇ref::Vector,
        q̈ref::Vector,
        Δt::Float64,
        pid::PID,
        B::Matrix{T},
        endEffector::Tuple{Vararg{String}};
        use_control::Bool = false,
        time_tol = 6
    ) where T <: Union{Int64, Float64}

# Description:
Defines a closed-loop position controller using a PID scheme with dynamics compensation.  
This function is meant to be passed as a controller to a dynamics simulator of RigidBodyDynamics (i.e., `simulate!`).  
It tracks a reference joint trajectory while ensuring torques respect the robot’s actuation constraints.

It also logs Center of Mass (CoM), torques, and time for later analysis.  
The function allows fallback to random open-loop torques if `use_control = false`.

# Arguments:
- `rs::RobotSimulator`                 : Simulator that stores the robot mechanism and logs.
- `tend::Float64`                      : End time of the control window [s].
- `t_actu::Float64`                    : Actuation delay before recording [s].
- `qref::Vector`                       : Desired joint positions [rad or m].
- `q̇ref::Vector`                      : Desired joint velocities.
- `q̈ref::Vector`                      : Desired joint accelerations.
- `Δt::Float64`                        : Sampling period of the controller [s].
- `pid::PID`                           : PID controller instance.
- `B::Matrix{T}`                       : Actuation matrix (maps active torques).
- `endEffector::Tuple{Vararg{String}}`: Names of the end-effector bodies (feet).

# Keyword Arguments:
- `use_control::Bool = false`         : If true, apply control torques; otherwise, use random input.
- `time_tol::Int = 6`                 : Number of digits for rounding simulation time (to avoid floating point artifacts).

# Returns:
- A `controller!` function to be passed to the simulator.
"""
function position_controller!(
    rs::RobotSimulator,
    tend::Float64,
    t_actu::Float64,
    qref::Vector,
    q̇ref::Vector,
    q̈ref::Vector,
    Δt::Float64,
    pid::PID,
    B::Matrix{<: Union{Int64, Float64}}, 
    endEffector::Tuple{Vararg{String}};
    use_control::Bool = false, 
    time_tol = 6              # avoid floating point mistakes
) 

    mechanism  = rs.mechanism
    dynamics_results = DynamicsResult(mechanism)
    sim_index = 0

    stop = false
    endEffector_body = RigidBody[] 

    # Store the end-effector bodies
    for elem in endEffector
        push!(endEffector_body, findbody(mechanism, elem))
    end 

    function controller!(τ, t, state)

        # Measure actual state 
        actual_q = configuration(state)
        actual_q̇ = velocity(state)

        ## Limit the dynamics velocities
        # for (i, joint) in enumerate(joints(mechanism))
        #     limit =  velocity_bounds(joint)[1]
        #     upper = limit.upper
        #     lower = limit.lower
        #     if upper !== Inf && lower !==-Inf
        #         actual_q̇[i] = clamp(actual_q̇[i], lower, upper)
        #     end
        # end 
        # set_velocity!(state, actual_q̇)
        # println("actula q", actual_q̇)
        # println(velocity(state))

        # Parameters
        t = round(t; digits = time_tol)
        v̇ = copy(velocity(state))

        # Position error 
        desired_q = vec(qref)
        Δq = desired_q - actual_q
        
        # Speed error
        desired_q̇ = vec(q̇ref)
        Δq̇ = vec(desired_q̇ .- actual_q̇)

        # PID 
        v̇_new = pid_control!(pid, Δq, Δq̇, Δt)
        v̇ .= v̇_new .+ q̈ref
        v̇ .= B * (B \ vec(v̇))   # Project to actuated space

        # Contact wrench estimation
        RigidBodyDynamics.contact_dynamics!(dynamics_results, state)
        external_wrenches = Dict{BodyID, Wrench{eltype(actual_q[1])}}()
        for (index, body) in enumerate(endEffector_body)
            sensor = RigidBodyDynamics.contact_wrench(dynamics_results, body)
            F_linear = sensor.linear
            if F_linear[3] >= 0.0
                push!(external_wrenches, body.id => sensor)
            end
        end

        # Reset all torques
        for joint in joints(mechanism)
            τ[velocity_range(state, joint)] .= 0 # no control
        end

        # Apply control 
        if (use_control == false)
            rand!(τ)
            if (length(τ) >= 8)
                τ[1:2] .= 0
            end
            τ .= (τ .- 0.5)
            τ[(end - 1):end] .= 0
        else
            u = B \ inverse_dynamics(state, v̇, external_wrenches)           
            τ   .= B * u
        end

        # Logging
        if (t >= (sim_index + 1) * Δt && t <= tend)
            sim_index = sim_index + 1
            com = center_of_mass(state).v
            push!(rs.CoM, com)
            push!(rs.torques, τ)
            push!(rs.time, [t + t_actu])            
        end

        # Final Logging (estimation of CoM velocity and acceleration)
        if (t == tend && stop == false)
            StartIndex = size(rs.CoM, 1) - (sim_index - 1)
            if(StartIndex == 2)                 # Compute initial speed and acceleration
                StartIndex =  StartIndex - 1
            end 
            measureCȯM!(rs, StartIndex)
            measureCöM!(rs, StartIndex)
            rs.index = StartIndex
            stop = true
        end

        return nothing
    end
end

function acceleration_controller!(
    rs::RobotSimulator,
    tend::Float64,
    t_actu::Float64,
    q̈ref::Vector,
    Δt::Float64,
    endEffector::Tuple{Vararg{String}};
    use_control::Bool = false, 
    disturbance::Bool = false,
    time_tol = 6              # avoid floating point mistakes
) 
    mechanism  = rs.mechanism
    dynamics_results = DynamicsResult(mechanism)
    sim_index = 0

    stop = false
    endEffector_body = RigidBody[] 

    for elem in endEffector
        push!(endEffector_body, findbody(mechanism, elem))
    end 

    v̇ = copy(velocity(MechanismState(rs.mechanism)))

    println("Reference acceleration :", q̈ref)

    function controller!(τ, t, state)

        v̇ .= q̈ref
        t = round(t; digits = time_tol)

        # Retrieve sensor values
        RigidBodyDynamics.contact_dynamics!(dynamics_results, state)
        external_wrenches = Dict{BodyID, Wrench{eltype(configuration(state)[1])}}()
        for (index, body) in enumerate(endEffector_body)
            sensor = RigidBodyDynamics.contact_wrench(dynamics_results, body)
            F_linear = sensor.linear
            if F_linear[3] >= 0.0
                push!(external_wrenches, body.id => sensor)
            end
        end

        # Reset torque
        for joint in joints(mechanism)
            τ[velocity_range(state, joint)] .= 0 # no control
        end
        u = inverse_dynamics(state, v̇)#, external_wrenches)
        τ .= u

        if (t >= (sim_index + 1) * Δt && t <= tend)
            sim_index = sim_index + 1
            com = center_of_mass(state).v
            push!(rs.CoM, com)
            push!(rs.torques, τ)
            push!(rs.time, [t + t_actu])
            measureZMP(rs, dynamics_results, state, endEffector)
            
        end
        if (t == tend && stop == false)
            StartIndex = size(rs.CoM, 1) - (sim_index - 1)
            if(StartIndex == 2)                 # Compute initial speed and acceleration
                StartIndex =  StartIndex - 1
            end 
            measureCȯM!(rs, StartIndex)
            measureCöM!(rs, StartIndex)
            rs.index = StartIndex
            stop = true
        end
        return nothing
    end
end

"""
    torques_controller!(
        rs::RobotSimulator,
        torques::AbstractArray{<:Real},
        t_actu::Real,
        tend::Float64,
        Δt::Float64,
        endEffector::Tuple{Vararg{String}};
    )

# Description:
Defines an open-loop torque controller that applies a predefined torque signal to the robot.  
The function logs the Center of Mass (CoM), torques, and time during the simulation.

It is designed to be passed as a `controller!` function into a simulation loop.

# Arguments:
- `rs::RobotSimulator`                 : Simulator containing the robot and log buffers.
- `torques::AbstractArray{<:Real}`    : Predefined vector of torques to apply to the joints.
- `t_actu::Real`                       : Actuation delay to shift time logging [s].
- `tend::Float64`                      : Final simulation time [s].
- `Δt::Float64`                        : Sampling period of the simulation [s].
- `endEffector::Tuple{Vararg{String}}`: Names of the end-effector bodies (e.g., feet) for ZMP measurement.

# Returns:
- A `controller!` function that can be passed to the simulator (e.g., `simulate!(...)`).
"""
function torques_controller!(
        rs::RobotSimulator, 
        torques::AbstractArray{<:Real}, 
        t_actu::Real,  
        tend::Float64, 
        Δt::Float64,  
        endEffector::Tuple{Vararg{String}};
    )

    mechanism  = rs.mechanism
    dynamics_results = DynamicsResult(mechanism)
    sim_index = 0

    function controller!(τ, t, state)
        # Compute current index based on time step
        index = floor(Int, t / Δt) + 1
    
        # Prevent going past final index
        if t > tend 
            index = sim_index
        end 
        
        # Apply predefined torque command 
        τ .= torques

        # Log values if we're on a new time step
        if (index > sim_index && t < tend)
            sim_index = sim_index + 1
            com = center_of_mass(state).v
            push!(rs.CoM, com)
            push!(rs.torques, τ)
            
            # Avoid exact zero time (numerical safety)
            if(t+t_actu == 0.0)
                t += 1e-6
            end 
            push!(rs.time, [t + t_actu])
        end
    end 
end 

"""
LQR_controller!

Define a finite-horizons discrete-time LQR controller that computes the torque input `τ` based on the current robot state and reference trajectory.
This controller operates within a simulation loop and stores key data (CoM, torque, time) at each discrete step.

# Arguments
- `rs::RobotSimulator`  : Simulator containing the robot and log buffers.
- `xref::Array{T}`      : Full reference state trajectory (position, velocity, acceleration) over time.
- `K::Array{M}`         : List of time-varying LQR gain matrices, one per timestep.
- `BD::Array{T}`        : Discretized input matrix (Bd + Dd/Δt) used to compute control input.
- `Dd::Array{T}`        : Discretized feedthrough matrix (Dd), used for derivative control.
- `Γ_prev::Array{T}`    : Previous control input (used for continuity).
- `q̇_prev::Array{T}`    : Previous joint velocities (used to estimate acceleration).
- `Δt::Float64`         : Control time step duration.
- `tend::Float64`       : Final time of the simulation.
- `t_actu::Float64`     : Actual time at start of simulation interval.

# Returns
- Defines and returns an inner controller function `controller!(τ, t, state)` that updates torques `τ` based on the simulation time `t` and robot `state`.

# Notes
- Control is piecewise constant over each sampling interval Δt.
- Saves simulation data (CoM, torque, time) for post-processing.
- At the end of the simulation, it computes CoM velocity and acceleration for further analysis.
"""
function LQR_controller!(
    rs::RobotSimulator,
    xref::Array{T},
    K::Array{M}, 
    BD::Array{T},
    Dd::Array{T},
    Γ_prev::Array{T},
    q̇_prev::Array{T}, 
    Δt::Float64,
    tend::Float64,
    t_actu::Float64
    ) where {T <: Union{Int64, Float64}, M}

    sim_index = 0   # Simulation step index
    stop = false    # Flag to trigger final computations once

    function controller!(τ, t, state)
        # Get control index based on current time
        index = floor(Int, t / Δt) + 1
        if t > tend 
            index = sim_index   # Prevent out-of-bounds indexing after final time
        end 
        
        # Retrieve LQR gain matrix at current time step 
        k = K[index]

        # Extract actual robot states
        actual_q = configuration(state)
        actual_q̇ = velocity(state)
        estimated_q̈ = (actual_q̇ - q̇_prev) ./ Δt

        # Compute deviation from reference state 
        x̃ = vcat(actual_q, vcat(actual_q̇,  estimated_q̈)) - xref

        # Apply LQR control law
        v = -k * x̃

        # Compute control input Γ based on system matrices
        Γ = BD \ (Dd * Γ_prev ./ Δt) + v

        # Keep control constant within the interval [t_k, t_k + Δt)
        if index == sim_index 
            Γ = Γ_prev
        end 

        # Set output torque and store control
        τ .= Γ
        Γ_prev .= Γ # Store for continuity in next loop

        # Log values if we're on a new time step
        if (index > sim_index && t < tend)
            q̇_prev .= actual_q̇ 
            sim_index = sim_index + 1
            com = center_of_mass(state).v
            push!(rs.CoM, com)
            push!(rs.torques, τ)
            if(t+t_actu == 0.0)
                t += 1e-6
            end 
            push!(rs.time, [t + t_actu])
        end

        # At simulation end, compute CoM velocity and acceleration
        if (t >= tend && stop == false)
            StartIndex = size(rs.CoM, 1) - (sim_index - 1)
            if(StartIndex == 2)                 # Compute initial speed and acceleration
                StartIndex =  StartIndex - 1
            end 
            measureCȯM!(rs, StartIndex)
            measureCöM!(rs, StartIndex)
            rs.index = StartIndex
            stop = true
        end
    end 
end 

"""
    measureCȯM!(rs::RobotSimulator, StartIndex::Int64)

# Description:
Estimates the velocity of the Center of Mass (CoM) using finite differences.  
Velocity is computed from the logged CoM positions and time steps in the simulator.  
Uses:
- Forward difference at the start
- Backward difference at the end
- Central difference otherwise

# Arguments:
- `rs::RobotSimulator`     : Robot simulator containing CoM and time logs.
- `StartIndex::Int64`      : Index from which to start computing CoM velocities.

# Returns:
- None. Appends estimated CȯM values to `rs.CȯM`.
"""
function measureCȯM!(
        rs::RobotSimulator,
        StartIndex::Int64
    )
    CoM  = rs.CoM
    time = rs.time
    for i in StartIndex:length(CoM)
        if(i == 1)
            push!(rs.CȯM, (CoM[i+1] .- CoM[i]) ./ (time[i+1] .- time[i]))
        elseif (i == length(CoM))
            push!(rs.CȯM, (CoM[i] .- CoM[i-1]) ./ (time[i] .- time[i-1]))
        else
            push!(rs.CȯM, (CoM[i+1] .- CoM[i-1]) ./ (time[i+1] .- time[i-1]))
        end 
    end 
end 

"""
    measureCöM!(rs::RobotSimulator, StartIndex::Int64)

# Description:
Estimates the acceleration of the Center of Mass (CoM) using finite differences.  
Acceleration is computed from the CoM velocities and time steps.  
Uses:
- Forward difference at the start
- Backward difference at the end
- Central difference otherwise

# Arguments:
- `rs::RobotSimulator`     : Robot simulator containing CȯM and time logs.
- `StartIndex::Int64`      : Index from which to start computing CöM.

# Returns:
- None. Appends estimated CöM values to `rs.CöM`.
"""
function measureCöM!(
        rs::RobotSimulator,
        StartIndex::Int64
    )
    CȯM  = rs.CȯM
    time = rs.time
    for i in StartIndex:length(CȯM)
        if(i == 1)
            push!(rs.CöM, (CȯM[i+1] .- CȯM[i]) ./ (time[i+1] .- time[i]))
        elseif (i == length(CȯM))
            push!(rs.CöM, (CȯM[i] .- CȯM[i-1]) ./ (time[i] .- time[i-1]))
        else
            push!(rs.CöM, (CȯM[i+1] .- CȯM[i-1]) ./ (time[i+1] .- time[i-1]))
        end 
    end 
end 