"""
 -----------------------------------------------------------------------------------------------------------
| The Inverse Kinematics scheme determines joint translates high-level motion objectives                    |
| into a corresponding joint space representations. In this file, the inverse kinematics is computed using  |
| a series of constrained optimisation problem.                                                             |
|                                                                                                           |
| For further explaination, please refer to TODO link (link to the thesis) optimal control inputs           |
 -----------------------------------------------------------------------------------------------------------
"""


"""
    mutable struct InverseKinematics

Container for inverse kinematics error targets and joint bounds. 
"""
mutable struct InverseKinematics
    err_p_com   # Position error of the CoM
    err_v_com   # Velocity error of the CoM
    err_a_com   # Acceleration error of the CoM

    err_p_f1    # Position error of foot 1
    err_v_f1    # Velocity error of foot 1
    err_a_f1    # Acceleration error of foot 1

    err_p_f2    # Position error of foot 2
    err_v_f2    # Velocity error of foot 2
    err_a_f2    # Acceleration error of foot 2

    q_bounds    # Joint position bounds (e.g., min/max angles)
    q̇_bounds    # Joint velocity bounds
    q̈_bounds    # Joint acceleration bounds
end 


"""
    define_IK(q_bounds, q̇_bounds, q̈_bounds)

# Description:
Initializes an `InverseKinematics` instance with zero tracking error vectors and specified joint bounds.

# Arguments:
- `q_bounds::Dict{Int64, Tuple{Float64, Float64}}`  : Position bounds for each joint (min, max).
- `q̇_bounds::Dict{Int64, Tuple{Float64, Float64}}`  : Velocity bounds for each joint (min, max).
- `q̈_bounds::Dict{Int64, Tuple{Float64, Float64}}`  : Acceleration bounds for each joint (min, max).

# Returns:
- `ik::InverseKinematics` : Fully initialized inverse kinematics struct.
"""
function define_IK(
    q_bounds::Dict{Int64, Tuple{Float64, Float64}},
    q̇_bounds::Dict{Int64, Tuple{Float64, Float64}},
    q̈_bounds::Dict{Int64, Tuple{Float64, Float64}})

    init = zeros(3, 1)                  # Initial error: zero for any 3D quantity
    return InverseKinematics(
        init, init, init,               # CoM errors
        init, init, init,               # Foot 1 errors
        init, init, init,               # Foot 2 errors
        q_bounds, q̇_bounds, q̈_bounds    # Joint bounds
    )
end 

"""
    inverse_kinematic(ik, fk, joints_initial, CoM_reference, feet1_reference, feet2_reference;
        optn=(), check=false)

# Description:
Computes joint trajectories (`q`, `q̇`, `q̈`) that track desired CoM and foot trajectories
using a series of nonlinear constrained optimisation problem, given forward kinematics and joint bounds.

# Arguments:
- `ik::InverseKinematics`                           : IK container to store tracking errors and joint limits.
- `fk::ForwardKinematics`                           : Structure containing kinematic functions (positions, Jacobians, Hessians).
- `joints_initial::Matrix{<:Union{Float64, Int64}}` : Initial joint configuration `[q, q̇, q̈]` in columns.
- `CoM_reference::Matrix{<:Union{Float64, Int64}}`  : Desired CoM reference state (3 × 3).
- `foot1_reference::Matrix{<:Union{Float64, Int64}}`: Desired foot 1 reference state (3 × 3).
- `foot2_reference::Matrix{<:Union{Float64, Int64}}`: Desired foot 2 reference state (3 × 3).

# Optional Arguments: 
- `optn::Tuple{Vararg{Tuple{String, Any}}}`        : Options passed to the optimizer
- `check::Bool`                                     : If true, computes and stores tracking errors in `ik`.

# Returns:
- `q::Vector{<:Union(Float64, Int64)}`: Joint position vector.
- `q̇::Vector{<:Union(Float64, Int64)}`: Joint velocity vector.
- `q̈::Vector{<:Union(Float64, Int64)}`: Joint acceleration vector.
"""
function inverse_kinematic(
    ik::InverseKinematics,
    fk::ForwardKinematics, 
    joints_initial::Matrix{<:Union{Float64, Int64}},
    CoM_reference::Matrix{<:Union{Float64, Int64}},
    foot1_reference::Matrix{<:Union{Float64, Int64}},
    foot2_reference::Matrix{<:Union{Float64, Int64}}; 
    optn::Tuple{Vararg{Tuple{String, Any}}}=(),
    check::Bool = false)

    # Joint bounds 
    bounds = Dict{Int, Tuple{Float64, Float64}}()
    
    # --------------------------------------
    # 1. Retrieve kinematic functions
    # --------------------------------------
    # Retrieve position Functions 
    CoM   = fk.CoM
    f1    = fk.F[1]
    f2    = fk.F[2]
    # Retrieve Jacobian Functions 
    J_CoM = fk.J_CoM
    J_f1  = fk.J_F[1]
    J_f2  = fk.J_F[2]
    # Retrieve Hessian Functions
    J̇_CoM = fk.J̇_CoM
    J̇_f1  = fk.J̇_F[1]
    J̇_f2  = fk.J̇_F[2]

    # --------------------------------------
    # 2. Extract initial joint state
    # --------------------------------------
    q_init = joints_initial[:, 1] 
    q̇_init = joints_initial[:, 2] 
    q̈_init = joints_initial[:, 3] 

    # --------------------------------------
    # 3. Extract references
    # --------------------------------------
    # CoM reference states 
    CoM_ref = CoM_reference[1, :]
    CȯM_ref = CoM_reference[2, :] 
    CöM_ref = CoM_reference[3, :]

    # Foot 1 reference states 
    f1_ref = foot1_reference[1, :]
    ḟ1_ref = foot1_reference[2, :]
    f̈1_ref = foot1_reference[3, :]

    # Foot 2 reference states 
    f2_ref = foot2_reference[1, :]
    ḟ2_ref = foot2_reference[2, :]
    f̈2_ref = foot2_reference[3, :]

    # --------------------------------------
    # 4. Solve inverse kinematics
    # --------------------------------------
    q = compute_joint_positions(q_init, CoM, f1, f2, CoM_ref, f1_ref, f2_ref, bounds, optn)
    q̇ = compute_joint_velocities(q, q̇_init, J_CoM, J_f1, J_f2, CȯM_ref, ḟ1_ref, ḟ2_ref, ik.q̇_bounds, optn)
    q̈ = compute_joint_accelerations(q, q̇, q̈_init, J_CoM, J̇_CoM, J_f1, J̇_f1, J_f2, J̇_f2, CöM_ref, f̈1_ref, f̈2_ref, ik.q̈_bounds, optn)

    # --------------------------------------
    # 5. Compute and store tracking errors (optional)
    # --------------------------------------
    if check
        # Position evaluations
        p_CoM = CoM(q)  
        p_f1  = f1(q)
        p_f2  = f2(q)

        # Velocity evaluations
        v_CoM = J_CoM(q) * q̇
        v_f1  = J_f1(q)  * q̇
        v_f2  = J_f2(q)  * q̇

        # Acceleration evaluations
        a_CoM, a_f1, a_f2 = [], [], []
        for i in 1:3
            push!(a_CoM, q̇' * J̇_CoM(q)[i] * q̇ + (J_CoM(q) * q̈)[i])
            push!(a_f1 , q̇' * J̇_f1(q)[i]  * q̇ + (J_f1(q)  * q̈)[i])
            push!(a_f2 , q̇' * J̇_f2(q)[i]  * q̇ + (J_f2(q)  * q̈)[i])
        end 
        
        # Compute tracking errors
        e_com = CoM_ref - p_CoM
        ė_com = CȯM_ref - v_CoM
        ë_com = CöM_ref - a_CoM

        e_f1 = f1_ref - p_f1
        ė_f1 = ḟ1_ref - v_f1
        ë_f1 = f̈1_ref - a_f1

        e_f2 = f2_ref - p_f2
        ė_f2 = ḟ2_ref - v_f2
        ë_f2 = f̈2_ref - a_f2

        # Store errors in InverseKinematics struct
        ik.err_p_com = hcat(ik.err_p_com, e_com)
        ik.err_v_com = hcat(ik.err_v_com, ė_com)
        ik.err_a_com = hcat(ik.err_a_com, ë_com)

        ik.err_p_f1  = hcat(ik.err_p_f1, e_f1)
        ik.err_v_f1  = hcat(ik.err_v_f1, ė_f1)
        ik.err_a_f1  = hcat(ik.err_a_f1, ë_f1)

        ik.err_p_f2  = hcat(ik.err_p_f2, e_f2)
        ik.err_v_f2  = hcat(ik.err_v_f2, ė_f2)
        ik.err_a_f2  = hcat(ik.err_a_f2, ë_f2)
    end 

    return q, q̇, q̈
end

"""
    compute_joint_positions(q_init, CoM, f1, f2, CoM_ref, f1_ref, f2_ref, bounds, optn)

# Description:
Solves a nonlinear optimization problem to compute joint positions `q` that minimize
the tracking error between the predicted CoM and foot positions w.r.t. the given references.

# Arguments:
- `q_init::Vector{<:Union{Float64, Int64}}` : Initial guess for joint positions (warm-start).  
- `CoM::Function`                           : Forward kinematics function for CoM.  
- `f1::Function`                            : Forward kinematics functions for foot 1. 
- `f2::Function`                            : Forward kinematics functions for foot 2.  
- `CoM_ref::Vector{<:Union{Float64, Int64}}`: Reference CoM position (3×1).  
- `f1_ref::Vector{<:Union{Float64, Int64}}` : Reference foot 1 position (3×1).
- `f2_ref::Vector{<:Union{Float64, Int64}}` : Reference foot 2 position (3×1).  
- `bounds::Dict{Int64, Tuple{Float64, Float64}}`: Dictionary of joint index → (min, max) tuples.
- `optn::Tuple{Vararg{Tuple{String, Any}}}`    : Solver options to pass to Ipopt.

# Returns:
- `q::Vector{<:Union{Float64, Int64}}` : Optimised joint positions.
"""
function compute_joint_positions(
    q_init::Vector{<:Union{Float64, Int64}}, 
    CoM::Function, 
    f1::Function, 
    f2::Function, 
    CoM_ref::Vector{<:Union{Float64, Int64}}, 
    f1_ref::Vector{<:Union{Float64, Int64}}, 
    f2_ref::Vector{<:Union{Float64, Int64}}, 
    bounds::Dict{Int64, Tuple{Float64, Float64}}, 
    optn::Tuple{Vararg{Tuple{String, Any}}})

    # Problem size 
    nq = length(q_init)

    # Initialize JuMP model and variables
    # model, q = defineOptimization(Ipopt.Optimizer, nq, 1)

    # # Set bounds on variables
    # for (i, (l, u)) in bounds
    #      setBounds(q[i], l, u)
    # end

    model = JuMP.Model(Ipopt.Optimizer)

    
    # # Define decision variables, i.e., robot's generalised coordinates
    JuMP.@variable(model, x1)
    JuMP.@variable(model,  -1.0   <= z1 <= 1.0)
    JuMP.@variable(model, -pi/4  <= q1  <= pi/4)
    JuMP.@variable(model, -3pi/4 <= q31 <= pi/4)
    JuMP.@variable(model, -3pi/4 <= q32 <= pi/4)
    JuMP.@variable(model,  0.0   <= q41 <= pi)
    JuMP.@variable(model,  0.0   <= q42 <= pi)
    JuMP.@variable(model, -3pi/4 <= q51 <= 3pi/4)
    JuMP.@variable(model, -3pi/4 <= q52 <= 3pi/4)

    # Constraint: Locked hip ? 
    if nq == 8
        JuMP.@NLconstraint(model, q1 == 0.0)
        JuMP.@NLconstraint(model, q51 == -(q31+q41))
        JuMP.@NLconstraint(model, q52 == -(q32+q42))
        q = [x1; z1; q31; q32; q41; q42; q51; q52]
    else 
        q = [x1; z1; q1; q31; q32; q41; q42; q51; q52]
    end
    
    # Compute FK outputs
    p_CoM = CoM(q)  
    p_f1  = f1(q)
    p_f2  = f2(q)
    
    # Define cost (quadratic tracking error)
    JuMP.@NLexpression(model, obj, (CoM_ref[1] - p_CoM[1])^2 
                                 + (CoM_ref[2] - p_CoM[2])^2
                                 + (CoM_ref[3] - p_CoM[3])^2
                                 + (f1_ref[1]  - p_f1[1] )^2
                                 + (f1_ref[2]  - p_f1[2] )^2
                                 + (f1_ref[3]  - p_f1[3] )^2
                                 + (f2_ref[1]  - p_f2[1] )^2
                                 + (f2_ref[2]  - p_f2[2] )^2
                                 + (f2_ref[3]  - p_f2[3] )^2
    )

    # Set cost
    defineObjective(model, "Min", obj)

    # Set solver options
    set_optimizer_option(model; optn=optn)    

    # Set warm-start values in the model
    JuMP.set_start_value.(q, q_init)

    # Solve the optimization problem
    JuMP.optimize!(model)

    # Warn if the solver fails to return a feasible solution
    if !is_solved_and_feasible(model)
        return @error("⚠️ Inverse Kinematics: The model was not solved during the q computation.") 
    end

    # Retrieve the optimised joint positions 
    sol = [JuMP.value.(var) for var in q]
    return sol 
end 


"""
    compute_joint_velocities(q, q̇_init, J_CoM, J_f1, J_f2, CȯM_ref, ḟ1_ref, ḟ2_ref, bounds, optn)

# Description:
Solves a nonlinear optimization problem to compute joint velocities `q̇` that minimize
the tracking error between the predicted CoM and foot velocities w.r.t. the given references.

# Arguments:
- `q::Vector{<:Union{Float64, Int64}}`      : Optimised joint positions.
- `q̇_init::Vector{<:Union{Float64, Int64}}` : Initial guess for joint velocities (warm-start).  
- `J_CoM::Function`                         : Function that returns the Jacobian of the CoM with respect to q.  
- `J_f1::Function`                          : Function that returns the Jacobian of the foot 1 with respect to q.   
- `J_f2::Function`                          : Function that returns the Jacobian of the foot 2 with respect to q.  
- `CȯM_ref::Vector{<:Union{Float64, Int64}}`: Reference CoM velocity (3×1).  
- `ḟ1_ref::Vector{<:Union{Float64, Int64}}` : Reference foot 1 velocity (3×1).
- `ḟ2_ref::Vector{<:Union{Float64, Int64}}` : Reference foot 2 velocity (3×1).  
- `bounds::Dict{Int64, Tuple{Float64, Float64}}`: Dictionary of joint index → (min, max) tuples.
- `optn::Tuple{Vararg{Tuple{String, Any}}}`    : Solver options to pass to Ipopt.

# Returns:
- `q̇::Vector{<:Union{Float64, Int64}}` : Optimised joint velocities.
"""
function compute_joint_velocities(
    q::Vector{<:Union{Float64, Int64}}, 
    q̇_init::Vector{<:Union{Float64, Int64}},
    J_CoM::Function, 
    J_f1::Function, 
    J_f2::Function, 
    CȯM_ref::Vector{<:Union{Float64, Int64}}, 
    ḟ1_ref::Vector{<:Union{Float64, Int64}}, 
    ḟ2_ref::Vector{<:Union{Float64, Int64}}, 
    bounds::Dict{Int64, Tuple{Float64, Float64}}, 
    optn::Tuple{Vararg{Tuple{String, Any}}})

    # Problem size 
    nq = length(q)

    # Initialize JuMP model and variables
    model, q̇ = defineOptimization(Ipopt.Optimizer, nq, 1)

    # Set bounds on variables
    for (i, (l, u)) in bounds
         setBounds(q̇[i], l, u)
    end

    # Compute FK outputs
    v_CoM = J_CoM(q) * q̇
    v_f1  = J_f1(q)  * q̇
    v_f2  = J_f2(q)  * q̇
    
    # Define cost (quadratic tracking error)
    JuMP.@NLexpression(model, obj, (v_CoM[1] - CȯM_ref[1])^2 
                                 + (v_CoM[2] - CȯM_ref[2])^2
                                 + (v_CoM[3] - CȯM_ref[3])^2
                                 + (v_f1[1]  - ḟ1_ref[1] )^2
                                 + (v_f1[2]  - ḟ1_ref[2] )^2
                                 + (v_f1[3]  - ḟ1_ref[3] )^2
                                 + (v_f2[1]  - ḟ2_ref[1] )^2 
                                 + (v_f2[2]  - ḟ2_ref[2] )^2
                                 + (v_f2[3]  - ḟ2_ref[3] )^2
    )

    # Set cost
    defineObjective(model, "Min", obj)

    # Set solver options
    set_optimizer_option(model; optn=optn)    

    # Set warm-start values in the model
    JuMP.set_start_value.(q̇, q̇_init)

    # Solve the optimization problem
    JuMP.optimize!(model)

    # Warn if the solver fails to return a feasible solution
    if !is_solved_and_feasible(model)
        return @error("⚠️ Inverse Kinematics: The model was not solved during the q̇ computation.") 
    end

    # Retrieve the optimised joint velocities
    sol = vec(JuMP.value.(q̇))
    return sol 
end 


"""
    compute_joint_accelerations(q, q̇, q̈_init, J_CoM,  J̇_CoM, J_f1, J̇_f1, J_f2, J̇_f2, CöM_ref, f̈1_ref, f̈2_ref, bounds, optn)

# Description:
Solves a nonlinear optimization problem to compute joint accelerations `q̈` that minimize
the tracking error between the predicted CoM and foot accelerations w.r.t. the given references.

# Arguments:
- `q::Vector{<:Union{Float64, Int64}}`      : Optimised joint positions.
- `q̇::Vector{<:Union{Float64, Int64}}`      : Optimised joint velocities.
- `q̈_init::Vector{<:Union{Float64, Int64}}` : Initial guess for joint accelerations (warm-start).  
- `J_CoM::Function`                         : Function that returns the Jacobian of the CoM with respect to q.  
- `J̇_CoM::Function`                         : Function that returns the Hessian of the CoM with respect to q, q̇.  
- `J_f1::Function`                          : Function that returns the Jacobian of the foot 1 with respect to q.  
- `J̇_f1::Function`                          : Function that returns the Hessian of the foot 1 with respect to q, q̇. 
- `J_f2::Function`                          : Function that returns the Jacobian of the foot 2 with respect to q.
- `J̇_f2::Function`                          : Function that returns the Hessian of the foot 2 with respect to q, q̇.     
- `CöM_ref::Vector{<:Union{Float64, Int64}}`: Reference CoM acceleration (3×1).  
- `f̈1_ref::Vector{<:Union{Float64, Int64}}` : Reference foot 1 acceleration (3×1).
- `f̈2_ref::Vector{<:Union{Float64, Int64}}` : Reference foot 2 acceleration (3×1).  
- `bounds::Dict{Int64, Tuple{Float64, Float64}}`: Dictionary of joint index → (min, max) tuples.
- `optn::Tuple{Vararg{Tuple{String, Any}}}`    : Solver options to pass to Ipopt.

# Returns:
- `q̈::Vector{<:Union{Float64, Int64}}` : Optimised joint accelerations.
"""
function compute_joint_accelerations(
    q::Vector{<:Union{Float64, Int64}}, 
    q̇::Vector{<:Union{Float64, Int64}}, 
    q̈_init::Vector{<:Union{Float64, Int64}}, 
    J_CoM::Function, 
    J̇_CoM::Function, 
    J_f1::Function, 
    J̇_f1::Function, 
    J_f2::Function, 
    J̇_f2::Function, 
    CöM_ref::Vector{<:Union{Float64, Int64}}, 
    f̈1_ref::Vector{<:Union{Float64, Int64}}, 
    f̈2_ref::Vector{<:Union{Float64, Int64}}, 
    bounds::Dict{Int64, Tuple{Float64, Float64}}, 
    optn::Tuple{Vararg{Tuple{String, Any}}})

    # Problem size 
    nq = length(q)

    # Initialize JuMP model and variables
    model, q̈ = defineOptimization(Ipopt.Optimizer, nq, 1)

    # Set bounds on variables
    for (i, (l, u)) in bounds
         setBounds(q̈[i], l, u)
    end

    # Compute FK outputs
    a_CoM, a_f1, a_f2  = [], [], []
    for i in 1:3
        push!(a_CoM, q̇' * J̇_CoM(q)[i] * q̇ .+ (J_CoM(q) * q̈)[i])
        push!(a_f1 , q̇' * J̇_f1(q)[i]  * q̇ .+ (J_f1(q)  * q̈)[i])
        push!(a_f2 , q̇' * J̇_f2(q)[i]  * q̇ .+ (J_f2(q)  * q̈)[i])
    end 

    # Define cost (quadratic tracking error)
    JuMP.@NLexpression(model, obj, (a_CoM[1] - CöM_ref[1])^2 
                                 + (a_CoM[2] - CöM_ref[2])^2 
                                 + (a_CoM[3] - CöM_ref[3])^2 
                                 + (a_f1[1]  - f̈1_ref[1] )^2
                                 + (a_f1[2]  - f̈1_ref[2] )^2
                                 + (a_f1[3]  - f̈1_ref[3] )^2
                                 + (a_f2[1]  - f̈2_ref[1] )^2
                                 + (a_f2[2]  - f̈2_ref[2] )^2
                                 + (a_f2[3]  - f̈2_ref[3] )^2
    )

    # Set cost
    defineObjective(model, "Min", obj)

    # Set solver options
    set_optimizer_option(model; optn=optn)    

    # Set warm-start values in the model
    JuMP.set_start_value.(q̈, q̈_init)

    # Warn if the solver fails to return a feasible solution
    JuMP.optimize!(model)
    if !is_solved_and_feasible(model)
        return @error("⚠️ Inverse Kinematics: The model was not solved during the q̈ computation.") 
    end

    # Retrieve the optimised joint accelerations
    sol = vec(JuMP.value.(q̈))
    return sol 
end 

"""
    setBounds(x, lower, upper)

# Description:
Applies lower and upper bounds to a JuMP decision variable.

# Arguments:
- `x::JuMP.VariableRef` : The JuMP variable to constrain.
- `lower::Float64`      : The lower bound.
- `upper::Float64`      : The upper bound.
"""
function setBounds(x::JuMP.VariableRef, lower::Float64, upper::Float64)
    JuMP.set_lower_bound(x, lower)  
    JuMP.set_upper_bound(x, upper)  
end 

"""
    defineOptimization(optimizer, n, N)

# Description:
Initializes a JuMP optimization model with decision variables defined over a prediction horizon.

# Arguments:
- `optimizer`   : A JuMP-compatible optimizer (e.g., `Ipopt.Optimizer`, `OSQP.Optimizer`, etc.)
- `n::Int64`    : Number of variables per time step.
- `N::Int64`    : Number of time steps.

# Returns:
- `model::Model`                            : The JuMP optimization model.
- `var::AbstractMatrix{<:JuMP.VariableRef}` : A 2D array of JuMP decision variables of size `n × N`.
"""
function defineOptimization(optimizer, n::Int64, N::Int64)
    # Create a JuMP model using the given optimizer
    model = JuMP.Model(optimizer)

    # Define decision variables: a matrix of size (n × N)
    JuMP.@variable(model, var[1:n, 1:N])

    return model, var
end 

"""
    defineObjective(model, mode, objExpression)

# Description:
Set a nonlinear objective function to a JuMP model, either for minimization or maximization.

# Arguments:
- `model::JuMP.Model`             : The JuMP optimization model.
- `mode::String`                  : Either `"Min"` or `"Max"` to set the optimization direction.
- `objExpression`                 : An expression compatible with `@NLobjective`.

# Notes:
- Throws an error if mode is invalid.
"""
function defineObjective(model::JuMP.Model, mode::String, objExpression)
    if (mode == "Min")
        JuMP.@NLobjective(model, Min, objExpression)
    elseif (mode == "Max")
        JuMP.@NLobjective(model, Max, objExpression)
    else
        return error("⚠️ Inverse Kinematics: The selected objective mode is Invalid. Please choose \"Min\" or \"Max\".")
    end 
end


"""
    set_optimizer_option(model; optn=())

# Description:
Applies optional optimizer-specific settings (attributes) to a JuMP model.

# Arguments:
- `model::JuMP.Model`                       : The JuMP model to which the attributes will be applied.
- `optn::Tuple{Vararg{Tuple{String, Any}}}` : A keyword-style tuple of options like `("tol", 1e-6)`.
"""
function set_optimizer_option(model::JuMP.Model; optn::Tuple{Vararg{Tuple{String, Any}}}=())
    if (length(optn) != 0)
        for (key, value) in optn
            JuMP.set_optimizer_attribute(model, key, value)
        end 
    end
end 



###########################################################
#                      Plot results                       #
###########################################################

function plot_IK(ik::InverseKinematics, tplot, dpi, lw; save::Bool=false, savePath::String="")

    tend = tplot[end]
    err_p_com = ik.err_p_com
    err_v_com = ik.err_v_com
    err_a_com = ik.err_a_com
    err_p_f1  = ik.err_p_f1
    err_v_f1  = ik.err_v_f1
    err_a_f1  = ik.err_a_f1
    err_p_f2  = ik.err_p_f2
    err_v_f2  = ik.err_v_f2
    err_a_f2  = ik.err_a_f2

    # Position plots
    # -------------- 
    plt_err_px = plot(; xlabel = L"$t$ [s]", xlims = (0, tend), layout = (1, 1), dpi = dpi)

    plot!(
        plt_err_px,
        tplot,
        err_p_com[1, :];
        lw = lw,
        label = L"$x_c$",
        ylabel = L"$Error$ [m]",
    )

    plot!(
        plt_err_px,
        tplot,
        err_p_f1[1, :];
        lw = lw,
        label = L"$x_{f1}$",
    )

    plot!(
        plt_err_px,
        tplot,
        err_p_f2[1, :];
        lw = lw,
        label = L"$x_{f2}$",
        title = "Position Inverse Kinematics Errors on x-axis",
    )

    plt_err_pz = plot(; xlabel = L"$t$ [s]", xlims = (0, tend), layout = (1, 1), dpi = dpi)

    plot!(
        plt_err_pz,
        tplot,
        err_p_com[3, :];
        lw = lw,
        label = L"$z_c$",
        ylabel = L"$Error$ [m]",
    )

    plot!(
        plt_err_pz,
        tplot,
        err_p_f1[3, :];
        lw = lw,
        label = L"$z_{f1}$"
    )

    plot!(
        plt_err_pz,
        tplot,
        err_p_f2[3, :];
        lw = lw,
        label = L"$z_{f2}$",
        linestyle=:dash,
        title = "Position Inverse Kinematics Errors on z-axis",
    )

    # Velocity plots
    # -------------- 
    plt_err_vx = plot(; xlabel = L"$t$ [s]", xlims = (0, tend), layout = (1, 1), dpi = dpi)

    plot!(
        plt_err_vx,
        tplot,
        err_v_com[1, :];
        lw = lw,
        label = L"$\dot{x}_c$",
        ylabel = L"$Error$ [m/s]",
    )

    plot!(
        plt_err_vx,
        tplot,
        err_v_f1[1, :];
        lw = lw,
        label = L"$\dot{x}_{f1}$",
    )

    plot!(
        plt_err_vx,
        tplot,
        err_v_f2[1, :];
        lw = lw,
        label = L"$\dot{x}_{f2}$",
        title = "Velocity Inverse Kinematics Errors on x-axis",
    )

    plt_err_vz = plot(; xlabel = L"$t$ [s]", xlims = (0, tend), layout = (1, 1), dpi = dpi)

    plot!(
        plt_err_vz,
        tplot,
        err_v_com[3, :];
        lw = lw,
        label = L"$\dot{z}_c$",
        ylabel = L"$Error$ [m/s]",
    )

    plot!(
        plt_err_vz,
        tplot,
        err_v_f1[3, :];
        lw = lw,
        label = L"$\dot{z}_{f1}$"
    )

    plot!(
        plt_err_vz,
        tplot,
        err_v_f2[3, :];
        lw = lw,
        label = L"$\dot{z}_{f2}$",
        linestyle=:dash,
        title = "Velocity Inverse Kinematics Errors on z-axis",
    )

    # Acceleration plots
    # -------------- 
    plt_err_ax = plot(; xlabel = L"$t$ [s]", xlims = (0, tend), layout = (1, 1), dpi = dpi)

    plot!(
        plt_err_ax,
        tplot,
        err_a_com[1, :];
        lw = lw,
        label = L"$\ddot{x}_c$",
        ylabel = L"$Error$ [m/s²]",
    )

    plot!(
        plt_err_ax,
        tplot,
        err_a_f1[1, :];
        lw = lw,
        label = L"$\ddot{x}_{f1}$",
    )

    plot!(
        plt_err_ax,
        tplot,
        err_a_f2[1, :];
        lw = lw,
        label = L"$\ddot{x}_{f2}$",
        title = "Acceleration Inverse Kinematics Errors on x-axis",
    )

    plt_err_az = plot(; xlabel = L"$t$ [s]", xlims = (0, tend), layout = (1, 1), dpi = dpi)

    plot!(
        plt_err_az,
        tplot,
        err_a_com[3, :];
        lw = lw,
        label = L"$\ddot{z}_c$",
        ylabel = L"$Error$ [m/s²]",
    )

    plot!(
        plt_err_az,
        tplot,
        err_a_f1[3, :];
        lw = lw,
        label = L"$\ddot{z}_{f1}$"
    )

    plot!(
        plt_err_az,
        tplot,
        err_a_f2[3, :];
        lw = lw,
        label = L"$\ddot{z}_{f2}$",
        linestyle=:dash,
        title = "Acceleration Inverse Kinematics Errors on z-axis",
    )

    display(plt_err_px)
    display(plt_err_pz)
    display(plt_err_vx)
    display(plt_err_vz)
    display(plt_err_ax)
    display(plt_err_az)
    
    if save 
        savefig(plt_err_px, savePath*"plt_err_px.png")
        savefig(plt_err_pz, savePath*"plt_err_pz.png")
        savefig(plt_err_vx, savePath*"plt_err_vx.png")
        savefig(plt_err_px, savePath*"plt_err_px.png")
        savefig(plt_err_vz, savePath*"plt_err_vz.png")
        savefig(plt_err_ax, savePath*"plt_err_ax.png")
        savefig(plt_err_az, savePath*"plt_err_az.png")
    end 
end 