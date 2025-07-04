###########################################################
#                    Linear Model                         #
###########################################################

"""
    stateTransitionFunction(A, B)

# Description:
Returns a closure `f(x, u)` that implements a **discrete-time linear state-space system** of the form:

    xₖ₊₁ = A * xₖ + B * uₖ

The function adapts internally to handle scalar, vector, or matrix inputs based on the dimensions of `A` and `B`, supporting both elementwise and matrix operations as needed.

# Arguments:
- `A` : State transition matrix (can be scalar, vector, or matrix).
- `B` : Control input matrix (same flexibility as `A`).

# Returns:
- A function `f(x, u)` that computes the next state based on the system dynamics.

# Logic:
- Checks the shape of A and B to determine if scalar or matrix operations should be applied.
- Uses:
    - `.*` for elementwise multiplication when A or B is a scalar.
    - `*` for standard matrix-vector multiplication otherwise.
"""
function stateTransitionFunction(A, B)
    # Size parameter
    n, m = size(A, 2), size(B, 2)
    
    # Case 1: both A and B are scalars (1×1) 
    if(n == 1 && m == 1)
        return f = (x, u) -> A .* x .+ B .* u
    
    # Case 2: A is scalar (1×1), B is vector/matrix
    elseif(n == 1)
        return f = (x, u) -> A .* x .+ B * u
    
    # Case 3: B is scalar (1×1), A is vector/matrix
    elseif (m == 1)           
        return f = (x, u) -> A * x .+ B .* u  
    
    # Case 4: general case — matrix operations
    else               
        return f = (x, u) -> A * x .+ B * u
    end 
end 

"""
    observationFunction(C)

# Description:
Returns an observation function `g(x)` based on the observation matrix `C`.  
This function computes the **observed output** from the system state `x`, using either elementwise or matrix multiplication depending on the shape of `C`.

Formally:
    y = C * x   or   y = C .* x

# Arguments:
- `C::Union{Number, AbstractArray}`: Observation matrix (or scalar). Can be a scalar, vector, or matrix.

# Returns:
- A function `g(x)` that computes the observation `y` from state `x`.
"""
function observationFunction(C)
    # Case 1: C is scalar (1x1)
    if(size(C, 2) == 1)
        return g = (x) -> C .* x

    # Case 2: general case — matrix operation
    else
        return g = (x) -> C * x
    end 
end

"""
    objectiveFunction(Q, R)

# Description:
Returns a cost function `obj(x, u, y, ref)` that computes a **quadratic cost** over a prediction horizon. The total cost includes:
- State/output tracking error (weighted by `Q`)
- Control effort penalty (weighted by `R`)

# Arguments:
- `Q::AbstractMatrix`: Weight matrix for output tracking error (positive semi-definite).
- `R::AbstractMatrix`: Weight matrix for control input effort (positive definite preferred).

# Returns:
- A function `obj(x, u, y, ref)` that computes the scalar cost:
  - `x`: State trajectory (not used in cost, but kept for extensibility)
  - `u`: Control inputs (matrix with control vectors in columns)
  - `y`: Output trajectory (matrix with output vectors in columns)
  - `ref`: Reference output trajectory (same size as `y`)

# Logic:
- Computes tracking error `e = y - ref`
- Adds a term `e'Qe` for each time step
- Adds a term `u'Ru` for each control input
"""
function objectiveFunction(Q, R)
    function obj(x, u, y, ref)
        e    = (y - ref)
        cost = 0.0
        
        # Output tracking cost over the horizon
        for col in eachcol(e)
            cost += col' * Q * col
        end 
        
        # Control effort cost over the horizon
        for col in eachcol(u)
            cost += col' * R * col 
        end 
        return cost
    end 
    return obj
end

"""
    objectiveFunction(Q, R₁, R₂)

# Description:
Returns an objective function `obj(x, u, y, ref)` that computes a **quadratic cost** over a prediction horizon. The total cost includes:
- State/output tracking error (weighted by `Q`)
- Control effort penalty (weighted by `R₁`)
- Change in control input Δu (smoothness penalty, weighted by `R₂`)

# Arguments:
- `Q::AbstractMatrix`   : Weight matrix for output tracking error `y - ref` (positive semi-definite).
- `R₁::AbstractMatrix`  : Weight matrix for control input magnitude `u` (positive definite preferred).
- `R₂::AbstractMatrix`  : Weight matrix for control input variation `Δu = uₖ - uₖ₋₁` (positive semi-definite).

# Returns:
- A function `obj(x, u, y, ref)` that computes the total cost:
    - `x`: State trajectory (not used, reserved for future extensions)
    - `u`: Control input matrix, shape (nu × N)
    - `y`: Output trajectory matrix, shape (ny × N)
    - `ref`: Reference output trajectory (same size as `y`)

# Logic:
1. Compute tracking error: `e = y - ref`
2. Compute control variation: `Δuₖ = uₖ - uₖ₋₁` for all k ∈ 2:N
3. Accumulate cost:
   - ∑ eᵢ' Q eᵢ
   - ∑ uᵢ' R₁ uᵢ
   - ∑ Δuᵢ' R₂ Δuᵢ
"""
function objectiveFunction(Q, R₁, R₂)
    function obj(x, u, y, ref)
        # ref = round.(ref, digits=9)
        e = (y - ref)
        total = 0.0

        # Compute Δu = uₖ - uₖ₋₁ for k = 2 to N
        Δu = []
        for k in 2:size(u, 2)
            push!(Δu, u[k] - u[k-1])
        end 
        Δu = reduce(hcat, Δu)

        # Tracking error cost
        for col in eachcol(e)
            total += col' * Q * col
        end 

        # Control effort cost
        for col in eachcol(u)
            total += col' * R₁ * col 
        end 

        # Control variation (Δu) cost
        for col in eachcol(Δu)
            total += col' *  R₂ * col 
        end 
        
        return total 
    end 
    return obj
end 

###########################################################
#              RigidBodyDynamics.jl Model                 #
###########################################################
# not compatible yet with JuMP, use the next section

function RGB_stateTransitionFunction(
    mechanism::Mechanism, 
    endEffector::Tuple{Vararg{String}}, 
    B::Matrix{T}
) where {T <: Union{Float64, Int64}}

    endEffector_body = RigidBody[] 

    for elem in endEffector
        push!(endEffector_body, findbody(mechanism, elem))
    end 

    function dynamics(x::AbstractVector{T}, u::AbstractVector{F}) where {T, F}

        # Define state and dynamic result structure (ensure type compatibility)
        state = MechanismState{T}(mechanism)  
        dynamics_results = DynamicsResult{T}(mechanism)

        # Problem size (Number of joints)
        n = length(configuration(state)) 

        # set the actual state of the robot
        set_configuration!(state, x[1:n])
        set_velocity!(state, x[n+1:end])

        # Retrieve sensor values
        RigidBodyDynamics.contact_dynamics!(dynamics_results, state)
        external_wrenches = Dict{BodyID, Wrench{T}}()
        for body in endEffector_body
            sensor = convert(Wrench{T}, RigidBodyDynamics.contact_wrench(dynamics_results, body))
            push!(external_wrenches, body.id => sensor)
        end 

        # Compute system dynamics including contact wrenches
        M = Matrix{T}(mass_matrix(state))  
        N = Vector{T}(RigidBodyDynamics.dynamics_bias(state, external_wrenches))   

        # Compute velocities and acceleration
        q̇ = x[n+1:end]         # Velocities remain the same
        q̈ = M \ (B * u - N)    # Compute acceleration

        return vcat(q̇, q̈)  # Return state derivative
    end
    return dynamics  
end


function RGD_observationFunction(mechanism::Mechanism, endEffector::Tuple{Vararg{String}}, localVec::Vector{Vector{T}}) where {T<: Union{Float64, Int64}}
    endEffector_body = RigidBody[] 

    for elem in endEffector
        push!(endEffector_body, findbody(mechanism, elem))
    end 

    d = 0.0045 # half foot height
        
    function referenceVector(x::AbstractVector{T}) where T

        # Define state and dynamic result structure (ensure type compatibility)
        state = MechanismState{T}(mechanism)  
        dynamics_results = DynamicsResult{T}(mechanism)

        # Problem size (Number of joints)
        n = length(configuration(state)) 

        # set the actual state of the robot
        set_configuration!(state, x[1:n])
        set_velocity!(state, x[n+1:end])

        # ------------------------------
        # ZMP estimation & feet position 
        # ------------------------------

        # Retrieve sensor values
        RigidBodyDynamics.contact_dynamics!(dynamics_results, state)
        external_wrenches = Vector{T}[]
        contact_Force = Vector{T}[]
        p_Feet = Vector{T}[]
        for (index, body) in enumerate(endEffector_body)
            # Feet position 
            push!(p_Feet, local2world(state, body, localVec[index]))

            # ZMP estimation 
            sensor = convert(Wrench{T}, RigidBodyDynamics.contact_wrench(dynamics_results, body))
            contact_torque = sensor.angular
            contact_force = sensor.linear
            if contact_force[3] > 0
                pr_x =
                (-contact_torque[2] - contact_force[1] * d) / contact_force[3]
                pr_y =
                (contact_torque[1] - contact_force[2] * d) / contact_force[3]
                push!(external_wrenches, [pr_x; pr_y])
                push!(contact_Force, [contact_force[3]])
            end 
        end 

        external_wrenches = reduce(hcat, external_wrenches)
        contact_Force     = reduce(vcat, contact_Force)
        p_Feet            = reduce(vcat, p_Feet)

        if length(contact_Force) == 1
            return vcat(external_wrenches, p_Feet)
        elseif length(contact_Force) == 0
            println("The robot is in Flight phase")
            exit(1)  # Exit with failure as for know no implementation for fligth
        else
            px = external_wrenches[1, 1] * contact_Force[1]
            py = external_wrenches[2, 1] * contact_Force[1]
            for i in 2:length(contact_Force)
                px +=  external_wrenches[1, i] * contact_Force[i]
                py +=  external_wrenches[2, i] * contact_Force[i]
            end 
            F_tot = sum(contact_Force)
            px /= F_tot
            py /= F_tot

            return vcat([px; py], p_Feet)
        end 
    end
end 

###########################################################
#                   Non Linear Model                      #
###########################################################

"""
    NL_stateTransitionFunction_5(nq, N, fk, fd)

# Description:
Constructs a nonlinear state transition function for a predictive control model of a 5-link bipedal robot.
The function returns a callable dynamics function that defines the system's evolution over `N` steps, based on full-body dynamics and joint constraints.

# Arguments:
- `nq::Int64`  : Number of generalized coordinates (e.g., joint angles).
- `N::Int64`   : Prediction horizon (number of time steps).
- `fk::ForwardKinematics`   : Object holding kinematics function.
- `fd::ForwardDynamics`     : Object providing the mass matrix and bias forces function.

# Returns:
- `dynamics(model, x_mpc, u_mpc)` : A JuMP-compliant function that, when called inside an optimization model, applies system constraints and enforces state evolution using dynamic equations.

"""
function NL_stateTransitionFunction(
    nq::Int64, 
    N::Int64,
    fk::ForwardKinematics, 
    fd::ForwardDynamics
)

    # Dynamic model access 
    Mass_Matrix = fd.Mass_Matrix 
    Bias_term = fd.Bias

    # Dynamics function to return
    function dynamics(model::Model, x_mpc::AbstractVecOrMat{T}, u_mpc::AbstractVecOrMat{F}) where {T, F}

         # Create acceleration decision variable
        JuMP.@variable(model, q̈_mpc[1:nq, 1:N])
        JuMP.set_start_value.(q̈_mpc, zeros(nq, N))

        # === STATE BOUNDS ===
        for i in 2:N+1
            # Joint position bounds 
            JuMP.set_upper_bound.(x_mpc[3, i], pi/4) 
            JuMP.set_lower_bound.(x_mpc[3, i], -3pi/4) 

            JuMP.set_upper_bound.(x_mpc[4, i], pi/4)  
            JuMP.set_lower_bound.(x_mpc[4, i], -3pi/4) 

            JuMP.set_upper_bound.(x_mpc[5, i], pi)  
            JuMP.set_lower_bound.(x_mpc[5, i], 0.0-1e-6) 

            JuMP.set_upper_bound.(x_mpc[6, i], pi)  
            JuMP.set_lower_bound.(x_mpc[6, i], 0.0-1e-6) 

            JuMP.set_upper_bound.(x_mpc[7, i], pi)  
            JuMP.set_lower_bound.(x_mpc[7, i], -pi) 

            JuMP.set_upper_bound.(x_mpc[8, i], pi)  
            JuMP.set_lower_bound.(x_mpc[8, i], -pi)

            # Cartesian bounds (e.g. height constraint to prevent flying)
            JuMP.set_upper_bound.(x_mpc[2, i], 0.0)
            JuMP.set_lower_bound.(x_mpc[2, i], -0.4225)
            
            # Joint velocity bounds
            JuMP.set_upper_bound.(x_mpc[11:12, i], 3.1415)  
            JuMP.set_lower_bound.(x_mpc[11:12, i], -3.1415)
            JuMP.set_upper_bound.(x_mpc[13:14, i], 5.23)
            JuMP.set_lower_bound.(x_mpc[13:14, i], -5.23)
        end

        # Time step duration [s]
        Ts = 0.02

        # State transition loop 
        for k in 1:N          

            # Current control input
            u      = u_mpc[:, k]

            # Resulting state 
            q      = x_mpc[1:nq, k+1]
            q̇      = x_mpc[nq+1:end, k+1]
            q̈      = q̈_mpc[:, k]

            # Previous state 
            q_prev = x_mpc[1:nq, k]
            q̇_prev = x_mpc[nq+1:end, k]

            # Compute robot dynamics
            M      = Mass_Matrix(q_prev)  
            N_bias = Bias_term(vcat(q_prev, q̇_prev))

            # Perpendicular to floor feet
            JuMP.@constraint(model, q[7] == -(q[3] + q[5])) # Left 
            JuMP.@constraint(model, q[8] == -(q[4] + q[6])) # Rigth 

            # Discretized dynamics: Forward Euler 
            Mq̈ = M * q̈
            for i in 1:nq
                JuMP.@constraint(model, Mq̈[i] == u[i] - N_bias[i])
                JuMP.@constraint(model, q̇[i]  == q̇_prev[i] + Ts * q̈[i])
                JuMP.@constraint(model, q[i]  == q_prev[i] + Ts * q̇[i])
            end 
        end
    end 

    return dynamics 
end

"""
    NL_observationFunction(
        N::Int64,
        PositionEnd::Array,     # [f_left(q), f_right(q)]
        CoM::Function,
        J_com::Function,
        J̇_com::Function,
        Contact_Jac::Array,     # [J_left(q), J_right(q)]
        Contact_Jȧc::Array,     # [J̇_left(q), J̇_right(q)]
        nq::Int64,
        p_feetL::Matrix,
        p_feetR::Matrix,
        v_feetL::Matrix,
        v_feetR::Matrix,
        a_feetL::Matrix,
        a_feetR::Matrix,
    )

# Description:
This function returns a closure `observation(model, x_mpc)` that constructs the **nonlinear observation model** over the prediction horizon.
The function computes and returns a matrix of reference signals (e.g. ZMP and CoM height) while enforcing foot position constraints through equality constraints.

# Arguments:
- `N::Int64`                : Prediction horizon.
- `PositionEnd::Array`      : List of functions giving left and right foot positions in world coordinates.
- `CoM::Function`           : Function returning the Center of Mass (CoM) position.
- `J_com::Function`         : Function returning the Jacobian of the CoM.
- `J̇_com::Function`         : Function returning the time derivative of the CoM Jacobian.
- `Contact_Jac::Array`      : [J_left(q), J_right(q)], contact Jacobians for both feet.
- `Contact_Jȧc::Array`      : [J̇_left(q), J̇_right(q)], time derivatives of foot Jacobians.
- `nq::Int64`              : Number of joint positions.
- `p_feetL/R::Matrix`      : Desired foot positions over time (columns are time steps).
- `v_feetL/R::Matrix`      : Desired foot velocities over time.
- `a_feetL/R::Matrix`      : Desired foot accelerations over time.

# Returns:
- A closure `observation(model::Model, x_mpc::AbstractVecOrMat)` which:
    - Computes ZMP using LIPM assumptions
    - Computes vertical CoM
    - Enforces position constraints on foot end-effectors
    - Returns the reference signal matrix over the horizon
"""
function NL_observationFunction(
    N::Int64,
    PositionEnd::Array, # to find feet position in world 
    CoM::Function,
    J_com::Function, 
    J̇_com::Function, 
    Contact_Jac::Array,
    Contact_Jȧc::Array,
    nq::Int64, 
    p_feetL::Matrix, 
    p_feetR::Matrix, 
    v_feetL::Matrix, 
    v_feetR::Matrix, 
    a_feetL::Matrix, 
    a_feetR::Matrix, 
)
    Index = 2
    positionleft = PositionEnd[1]
    positionrigth = PositionEnd[2]
    nmax = size(p_feetL, 2)
    f_J1 = Contact_Jac[1]
    f_J2 = Contact_Jac[2]
    f_J̇1 = Contact_Jȧc[1]
    f_J̇2 = Contact_Jȧc[2]
    function observation(model::Model, x_mpc::AbstractVecOrMat{T}) where T
        total_ref = []
        index = Index

        for i in 1:N 
            ref = []
            q      = x_mpc[1:nq, i+1]
            q̇      = x_mpc[nq+1:end, i+1]
            q̈ = similar(q)

            # Get the decision variable for acceleration at step i
            for j in 1:nq
                q̈[j] = JuMP.variable_by_name(model, "q̈_mpc[$j,$(i)]")   # attention valide avec la premiere et derniere approche 
            end 
   
            # Compute CoM and its derivatives
            Jcom = J_com(q)
            J̇com = J̇_com(q)
            com = CoM(q)
            cȯm = Jcom * q̇
            cöm = similar(cȯm)
            add = Jcom * q̈
            for k in 1:3
                cöm[k] = q̇' * J̇com[k] * q̇ + add[k]
            end 

            # LIPM
            x_zmp = com[1] - (com[3] / 9.81) * cöm[1]
            # x_zmp = com[1] - (0.1924541398385977 / 9.81) * cöm[1]

            # VH-LIPM
            # x_zmp = com[1] - (com[3] / (9.81 + cöm[3])) * cöm[1]

            # Append ZMP and CoM height to reference
            push!(ref, [x_zmp])
            push!(ref, [com[3]])
            
            # Select foot reference states
            if index < nmax
                pFL = p_feetL[:, index]
                pFR = p_feetR[:, index]
                vFL = v_feetL[:, index]
                vFR = v_feetR[:, index]
                aFL = a_feetL[:, index]
                aFR = a_feetR[:, index]
            else
                pFL = p_feetL[:, end]
                pFR = p_feetR[:, end]
                vFL = v_feetL[:, end]
                vFR = v_feetR[:, end]
                aFL = a_feetL[:, end]
                aFR = a_feetR[:, end]
            end 

            # Compute Foot jacobian 
            J1 = f_J1(q)
            J2 = f_J2(q)
            J̇1 = f_J̇1(q) 
            J̇2 = f_J̇2(q) 

            # Compute foot position in world 
            p_left = positionleft(q)
            p_right = positionrigth(q)
            v_left = (J1*q̇)
            v_right = (J2*q̇)
            a_left = [(q̇' *  J̇1[1] * q̇ + (J1 * q̈)[1]) (q̇' *  J̇1[2] * q̇ + (J1 * q̈)[2]) (q̇' *  J̇1[3] * q̇ + (J1 * q̈)[3])]
            a_right = [(q̇' *  J̇2[1] * q̇ + (J2 * q̈)[1]) (q̇' *  J̇2[2] * q̇ + (J2 * q̈)[2]) (q̇' *  J̇2[3] * q̇ + (J2 * q̈)[3])]

            # Enforce foot position constraints (x and z only)
            for i in 1:2:3
                JuMP.@constraint(model, p_left[i] == pFL[i])
                JuMP.@constraint(model, p_right[i] == pFR[i])
                # JuMP.@constraint(model, v_left[i] == vFL[i])
                # JuMP.@constraint(model, v_right[i] == vFR[i])
                # JuMP.@constraint(model, a_left[i] == aFL[i])
                # JuMP.@constraint(model, a_right[i] == aFR[i])
            end 

            # Append all references for this step
            ref = reduce(vcat, ref)
            push!(total_ref, ref)
            index += 1
        end 

        Index += 1
        return reduce(hcat, total_ref)
    end 
end 

"""
    getAddValue(model::Model, nq::Int64, q̈_vec::Array)

# Description:
Extracts the computed numerical values of joint accelerations `q̈` at each timestep from the solved JuMP model and appends them to an external container `q̈_vec`.

This is useful for post-processing or logging the results of an MPC optimization problem.

# Arguments:
- `model::Model`         : JuMP optimization model containing the solution.
- `nq::Int64`            : Number of joints (i.e., size of the configuration space).
- `q̈_vec::Array`        : An array (typically `Vector{Vector{Float64}}`) to store acceleration vectors across iterations or time steps.

# Behavior:
- Loops through each joint index `i ∈ 1:nq`
- Retrieves the JuMP variable named `"q̈_mpc[i,1]"` from the model
- Extracts its solved value using `JuMP.value`
- Appends the resulting vector `q̈` to the `q̈_vec` array (mutable, passed by reference)
"""
function getAddValue(model::Model, nq::Int64,  q̈_vec::Array)
    q̈ = []
    for i in 1:nq
        push!(q̈, JuMP.value.(JuMP.variable_by_name(model, "q̈_mpc[$i,1]")))
    end 
    push!(q̈_vec, q̈)
end 