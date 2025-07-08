using Symbolics
using SymPy
using RigidBodyDynamics

function print_step(msg::AbstractString)
    println("🔷" ^ 3 * " STEP: " * msg * " " * "🔷" ^ 3 * "\n")
end

"""
    compute_dynamics(mechanism::Mechanism, Contactbody::Tuple{Vararg{String}})

# Description:
Execute the numerical computation of kinematics and dynamics described in the UCLouvain thesis 
"Model Predictive Control Approach to Enhance Stable Walking for Planar Bipedal Robots". 

# Arguments:
- `mechanism::Mechanism`                 : Contains the RigidBodyDynamics mechanism of the parse URDF
- `Contactbody::Tuple{Vararg{String}}`   : Tuple containing the name of the end-effector bodies

# Returns:
- `f_Mass_Matrix::Function` : Callable that returns the joint-space mass matrix, i.e., M(q).
- `f_Bias::Function`        : Callable that returns the nonlinear bias forces, i.e., n(q, q̇) = C(q, q̇) * q̇ + G(q).
- `f_CoM::Function`         : Callable that computes the center of mass (CoM) position in the world frame.
- `f_J_com::Function`       : Callable that returns the Jacobian of the CoM position.
- `f_J̇_com::Function`       : Callable that returns the time derivative of the CoM Jacobian.
- `feetPosition::Vector{RuntimeGeneratedFunction}`  : Vector of functions computing each foot's position in the world frame.
- `contactJac::Vector{RuntimeGeneratedFunction}`    : Vector of functions returning each contact point's Jacobian.
- `contactJ̇ac::Vector{RuntimeGeneratedFunction}`    : Vector of functions returning the time derivative of each contact Jacobian.

# Note
- The ordering in `feetPosition`, `contactJac`, and `contactJ̇ac` matches the order of names provided in `Contactbody`.
"""
function compute_dynamics(mechanism::Mechanism, Contactbody::Tuple{Vararg{String}})
    println("\n○ Start numerical derivations of kinematics and dynamics \n")

    # --------------------
    # SymPy.jl derivation 
    # --------------------
    print_step("Generating SymPy Kinematics and Dynamics expressions")

    # Define symbolic type used for computation
    T = eltype(SymPy.symbols("_"))

    # Retrieve contact bodies from the mechanism
    Contact_body = RigidBody[] 
    for elem in Contactbody
        push!(Contact_body, findbody(mechanism, elem))
    end 

    # Initialize symbolic state
    state = MechanismState{T}(mechanism) 

    # Define Sympy symbolic generalized coordinates
    n = length(configuration(state))                        # Number of joints
    q = SymPy.symbols(["q[$i]" for i in 1:n], real=true)    # Positions
    q̇ = SymPy.symbols(["q̇[$i]" for i in 1:n], real=true)    # Velocities

    # Set symbolic state with current symbolic coordinates
    set_configuration!(state, q)
    set_velocity!(state, q̇)
    setdirty!(state)

    # Get base/root body (assumed to be the first body)
    bodies_list = bodies(mechanism)
    base_link   = first(bodies_list)

    # Compute end-effectors positions 
    Endeffector_pos = []
    for body in Contact_body
        P_end = ZMProbot.local2world(state, body, [0.0; 0.0; -0.009])
        P_endf = SymPy.lambdify(P_end, q)  
        push!(Endeffector_pos, P_endf)
    end 

    # Compute system dynamics, Ensure type compatibility
    M   = Matrix{T}(mass_matrix(state))  
    N   = Vector{T}(RigidBodyDynamics.dynamics_bias(state))  
    CoM =  center_of_mass(state)
    
    # Simplify convert to callable functions
    print_step("Converting the SymPy expressions into callable functions")
    M    = SymPy.simplify(M)
    Mf   = SymPy.lambdify(M, q) 

    N    = SymPy.simplify(N)
    Nf   = SymPy.lambdify(N, vcat(q, q̇))

    CoM  = SymPy.simplify(CoM)
    CoMf = SymPy.lambdify(collect(CoM.v), q)  

    # -------------------------------------------------------
    # Symbolics.jl for CSE, Jacobian and Hessian expressions
    # -------------------------------------------------------

    # Define symbolic generalized coordinates
    Symbolics.@variables q_sym[1:n] # Positions
    Symbolics.@variables q̇_sym[1:n] # Velocities
    vars = vcat(q_sym, q̇_sym)       # Concatenate state vector 

    # Symbolics Dynamics expressions 
    print_step("Generating Symbolics Kinematics and Dynamics expressions")
    Mass_Matrix_expr  = first(Symbolics.build_function(Mf(q_sym...), q_sym, expression=Val{false}, cse=true))
    Bias_expr         = first(Symbolics.build_function(Nf(vars...), vars, expression=Val{false}, cse=true))
    CoM_expr          = first(Symbolics.build_function(CoMf(q_sym...), q_sym, expression=Val{false}, cse=true))
    p_foot1_expr      = first(Symbolics.build_function(Endeffector_pos[1](q_sym...), q_sym, expression=Val{false}, cse=true))
    p_foot2_expr      = first(Symbolics.build_function(Endeffector_pos[2](q_sym...), q_sym, expression=Val{false}, cse=true))
   
    # Evaluate expressions into functions 
    print_step("Evaluating Kinematics and Dynamics expressions into functions")
    f_Mass_Matrix  = eval(Mass_Matrix_expr) ; Mass_Matrix_expr  = nothing 
    f_Bias         = eval(Bias_expr)        ; Bias_expr         = nothing
    f_CoM          = eval(CoM_expr)         ; CoM_expr          = nothing
    f_foot1        = eval(p_foot1_expr)     ; p_foot1_expr      = nothing 
    f_foot2        = eval(p_foot2_expr)     ; p_foot2_expr      = nothing 

    # Group foot position functions
    feetPosition   = [f_foot1, f_foot2]

    ## End-Effector Jacobians 
    print_step("Evaluating End-effector Jacobians")

    # Compute symbolic Jacobians w.r.t. q
    J_foot1_expr = Symbolics.jacobian(f_foot1(q_sym), q_sym) # ⚠️ If an error occurs at this step, try closing the terminal, opening a new one, and re-running the simulation.
    J_foot2_expr = Symbolics.jacobian(f_foot2(q_sym), q_sym)
    
    # Evaluate expressions into functions 
    f_J_foot1 = eval(first(Symbolics.build_function(J_foot1_expr, q_sym, expression=Val{false}, cse=true)))
    f_J_foot2 = eval(first(Symbolics.build_function(J_foot2_expr, q_sym, expression=Val{false}, cse=true)))
    
    # Group foot Jacobians functions
    contactJac   = [f_J_foot1, f_J_foot2]

    ## End-Effector Hessians 
    print_step("Evaluating End-effector Hessians")

    # Compute directional derivatives (∂J/∂q)
    J̇_foot1_expr = [Symbolics.jacobian(J_foot1_expr[i, :], q_sym) for i in 1:3]
    J̇_foot2_expr = [Symbolics.jacobian(J_foot2_expr[i, :], q_sym) for i in 1:3]

    # Evaluate expressions into functions 
    f_J̇_foot1 = eval(first(Symbolics.build_function(J̇_foot1_expr, q_sym, expression=Val{false}, cse=true)))
    f_J̇_foot2 = eval(first(Symbolics.build_function(J̇_foot2_expr, q_sym, expression=Val{false}, cse=true)))
    
    # Group foot Hessians functions
    contactJ̇ac   = [f_J̇_foot1, f_J̇_foot2]

    ## CoM Jacobian and Hessian 
    print_step("Evaluating CoM Jacobian")

    # CoM Jacobian
    J_com_expr = Symbolics.jacobian(CoMf(q_sym...), q_sym)
    f_J_com = eval(first(Symbolics.build_function(J_com_expr, q_sym, expression=Val{false}, cse=true)))

    # Compute directional derivatives (∂J/∂q)
    print_step("Evaluating CoM Hessian")
    J̇_com_expr = [Symbolics.jacobian(J_com_expr[i, :], q_sym) for i in 1:3] # ⚠️ If an error occurs at this step, try closing the terminal, opening a new one, and re-running the simulation.
    f_J̇_com = eval(first(Symbolics.build_function(J̇_com_expr, q_sym, expression=Val{false}, cse=true)))

    # Clear unused symbolic expressions
    J̇_com_expr1 = nothing
    J̇_com_expr2 = nothing 
    J̇_com_expr3 = nothing 
    J̇_com_expr  = nothing 
    J_com_expr  = nothing 

    println("✅ Numerical derivations.")
    return f_Mass_Matrix, f_Bias, f_CoM, f_J_com, f_J̇_com, feetPosition, contactJac, contactJ̇ac  
end 


f_Mass_Matrix, f_Bias, f_CoM, f_J_CoM, f_J̇_CoM, f_F, f_J_F, f_J̇_F = compute_dynamics(rs.mechanism, endEffector)
