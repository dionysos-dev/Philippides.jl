"""
The Model predictive Control (MPC) scheme determines optimal control inputs
by minimising a user-defined cost function over a finite prediction horizon. 

The model can generally be expressed in the following state space form:

    - ẋ = f(x, u)
    - y = g(x).

Here x ∈ ℜⁿ, u ∈ ℜᵐ, y ∈ ℜᵖ. The function 'f(∙)' is called the state space transition 
function while 'g(∙)' is the output/observation function.

The MPC aims at optimising the evolution of x, and so y, based on the value of u, over 
'N' discrete step of the 'Time' vector, to follow the reference vector 'ref ∈ ℜᵖˣᴺ'.

The objective function to minimize must be define as o(x, u, y, ref).
"""
mutable struct MpcParameter
    # Model size
    n::Int64
    m::Int64

    # Model functions
    stateTransition::Function  
    Output::Function 
    
    # Time vector and index
    Time::Array                          # Time vector 
    index::Int64                         # Previous time index
    N::Int64                             # Prediction length 

    # Reference to follow
    ref::Matrix    
    
    # Objective function
    obj::Function
    
    # Mode of optimization, i.e., :Min or :Max
    mode::Symbol                        

    # Optimizer otion(s)
    optn::Tuple{Vararg{Tuple{String, Any}}}   
    
    # Type 
    Type::Int64
end 

"""
    defineMpcParameter(...)

# Description:
Construct an `MpcParameter` instance with the given model definition, reference trajectory, 
objective function, and solver configuration.

# Arguments:
- `n::Int64`    : Number of states
- `m::Int64`    : Number of control inputs
- `F::Function` : State transition function f(x, u)
- `G::Function` : Output function g(x)
- `Time::Array` : Time vector for planned references
- `N::Int64`    : Prediction horizon length
- `ref::Matrix`  : Reference output trajectory over the horizon
- `obj::Function`   : Objective function o(x, u, y, ref)
- `mode::Symbol`    : Optimization mode (:Min or :Max)
- `optn::Tuple{Vararg{Tuple{String, Any}}}` : Options passed to the optimizer
- `Type::Int`       : Optimisation type indicator. Use `0` for linear problems, and `1` for nonlinear models. 

# Returns:
- An instance of `MpcParameter`
"""
function defineMpcParameter(
    n::Int64, 
    m::Int64, 
    F::Function, 
    G::Function,
    Time::Array, 
    N::Int64,
    ref::Matrix, 
    obj::Function,
    mode::Symbol;
    optn::Tuple{Vararg{Tuple{String, Any}}}, 
    Type::Int=0
    )
    return MpcParameter(n, m, F, G, Time, 0, N, ref, obj, mode, optn, Type)
end

"""
    updateMpcParameter(mpc, time, MPC_ref)

# Description:
Update the internal time index and reference trajectory of the given `mpc` object.

# Arguments:
- `mpc::MpcParameter`           : The `MpcParameter` instance containing the reference on horizon to update.
- `time::Union{Float64, Int64}` : Current simulation or real-world time (not used in simulation mode).
- `MPC_ref::Matrix{<:Real}`     : The full reference trajectory matrix, mathcing size with the pre-planned `mpc.Time` vector .

# Notes:
- In simulation, the internal index is incremented directly by 1.
- In real-time mode, consider using `get_index()` to compute the time index from `time` and `mpc.Time`.
- The reference horizon is truncated to match the prediction horizon `N` and prevent indexing beyond bounds.
"""
function updateMpcParameter(mpc::MpcParameter, time::Union{Float64, Int64}, MPC_ref::Matrix{<:Real})
   # Update time index (simulation mode)
    mpc.index += 1

    # For real-time mode, replace above line with:
    # mpc.index = get_index(time, mpc.Time, mpc.index)

    # Indexing variables
    k     = mpc.index                   # Current time step
    N     = mpc.N                       # Prediction horizon length
    kmax  = length(mpc.Time)            # Total number of time steps

    # Update reference horizon
    mpc.ref = truncVector(MPC_ref, k, kmax, N)

    # Optional debug print:
    # println("• Updated MPC reference at step $k (prediction length = $N):"); println(mpc.ref); println("\n")
end

"""
    computeMPControl(mpc, x_init, u0;
             addConstraintFunctions=Tuple{Function, Tuple}[], 
             valueFunctions=Tuple{Function, Tuple}[]
    )

# Description:
Execute the MPC optimization scheme over the prediction horizons.

# Arguments:
- `mpc::MpcParameter`       : The `MpcParameter` instance containing model parameters, references, and solver options.
- `x_init::Array{<:Real}`   : Initial state vector at the beginning of the horizon.
- `u0::Array{<:Real}`       : Initial guess for the control inputs (used for warm start).

# Optional Arguments: 
- `addConstraintFunctions::Vector{Tuple{<:Function, Tuple}}`    : A list of user-defined hard constraint functions and their arguments, each given as a tuple `(f, args)`.  
                                                                Each function must follow the signature: `f(model, x_mpc, u_mpc, N, args...)`.
- `valueFunctions::Vector{Tuple{<:Function, Tuple}}`            : A list of functions used to extract or compute values from the optimised model after solving.  
                                                                Each function must follow the signature: `f(model, args...)`.
Returns:
- `x::Array{Real}`  : The optimised state vector over the horizon (excluding the initial state).
- `u::Array{Real}`  : The optimised control inputs over the horizon.

# Notes:
- `addConstraintFunctions` and `valueFunctions` are passed as vectors of tuples `(function, arguments...)`.
- Each function is expected to operate on the JuMP model and can modify or extract constraints and values.
"""
function computeMPControl(mpc::MpcParameter, x_init::Array{<:Real}, u0::Array{<:Real};
    addConstraintFunctions=[],
    valueFunctions=[])

    # Retrieve problem dimensions and MPC components
    n = mpc.n                   # State dimension
    m = mpc.m                   # Control input dimension
    f = mpc.stateTransition     # State transition function
    g = mpc.Output              # Output function
    o = mpc.obj                 # Cost function
    N = mpc.N                   # Prediction horizon
    mode = mpc.mode             # Constraint mode (i.e., :min/:max)  
    optn = mpc.optn             # Solver options
    Type = mpc.Type             # Problem type (i.e., linear/nonlinear)
    ref  = mpc.ref              # Reference trajectory

    # Initialize JuMP model and variables
    model, x_mpc, u_mpc = define_mpc_model(n, m, N, Type)

    # Apply solver options if specified
    if (length(optn) != 0)
        for (key, value) in optn
            JuMP.set_optimizer_attribute(model, key, value)
        end 
    end
    #=
    set_optimizer_attribute(model, "rho", 0.01)       # Primal-dual scaling,  Default: 0.1
    set_optimizer_attribute(model, "alpha", 1.8)     # Relaxation parameter, Default: 1.6
    set_optimizer_attribute(model, "max_iter", 4000)  # Max iterations,      Default: 4000
    set_optimizer_attribute(model, "eps_abs", 1e-6)  # Absolute tolerance,   Default: 1e-4
    set_optimizer_attribute(model, "eps_rel", 1e-6)  # Relative tolerance,   Default: 1e-4
    set_optimizer_attribute(model, "verbose", v)     # Print solver output,  Default: true
    set_optimizer_attribute(model, "warm_start", false) # Default: true
    =#
    
    # Define model dynamics and cost in JuMP
    set_ModelConstraints!(model, x_mpc, u_mpc, f, x_init, N, Type)
    set_SoftConstraint!(model, x_mpc, u_mpc, g, ref, N, o, Type; mode=mode)

    # Apply additional user-defined constraints if provided
    if !isempty(addConstraintFunctions)
        for (constraint_fn, args) in addConstraintFunctions
            constraint_fn(model, x_mpc, u_mpc, N, args...)
        end
        println("✅ Additional MPC constraints added.")
    end

    # Initialize warm start values for optimization
    x0     = copy(x_init)
    u_init = copy(u0)
    for i in 1:N
        x0 = hcat(x0, x_init)
        if i < N 
            u0 = hcat(u0, u_init)
        end 
    end 

    # Set warm-start values in the model
    JuMP.set_start_value.(x_mpc, x0)
    JuMP.set_start_value.(u_mpc, u0)

    # Solve the optimization problem
    JuMP.optimize!(model)

    println("• MPC Objective value: ", objective_value(model))

    # Warn if the solver fails to return a feasible solution
    if !is_solved_and_feasible(model)
        @warn("⚠️ MPC: The model was not solved correctly.")
        return
    end

    # Execute value extraction functions after optimization, if provided
    if !isempty(valueFunctions)
        for (value_fn, args) in valueFunctions
            value_fn(model, args...)
        end
    end

    # Retrieve and assemble the optimized state and input vectors
    x_opt = copy(x_init)
    u_opt = Array{Real}[]
    for k in 1:N
        u = JuMP.value.(u_mpc[:, k])        
        x_opt = hcat(x_opt, Array(JuMP.value.(x_mpc[:, k+1])))
        push!(u_opt, u)
    end
    u_opt = reduce(hcat, u_opt)

    # Return state and control trajectory (excluding initial state)
    return x_opt[:, 2:end], u_opt
end

"""
    define_mpc_model(n, m, N, T)

# Description:
Create a generique MPC model with 'n' states and 'm' control input over an N*dt horizons

# Arguments:
- `n::Int64` : Number of state variables.
- `m::Int64` : Number of control input variables.
- `N::Int64` : Number of prediction steps (horizon).
- `T::Int64` : Solver selector (0 for OSQP, otherwise NLP/Ipopt).

# Returns:
- `model::JuMP.Model`                       : JuMP model object to be optimized.
- `x::AbstractMatrix{<:JuMP.VariableRef}`   : State variable array over the prediction horizon.
- `u::AbstractMatrix{<:JuMP.VariableRef}`   : Control variable array over the prediction horizon.
"""
function define_mpc_model(n::Int64, m::Int64, N::Int64, T::Int64)
    # Select solver based on problem type:
    # - OSQP for quadratic programming (T == 0)
    # - Ipopt for nonlinear programming (T ≠ 0)
    if T == 0
        model = JuMP.Model(OSQP.Optimizer)
    else
        model = JuMP.Model(Ipopt.Optimizer)
    end 

    # Define control variables: u[1:m, 1:N]
    # Each column corresponds to control inputs at a given time step
    JuMP.@variable(model, u[1:m, 1:N])
   
    # Define state variables: x[1:n, 1:N+1]
    # Includes actual state and N predicted future states
    JuMP.@variable(model, x[1:n, 1:N+1]) 

    return model, x, u
end

"""
    set_ModelConstraints!(model, x, u, f, x_init, N, T)

# Description:
Adds the system dynamics constraints to the MPC model.

- For linear models (`T == 0`), enforces:
    - `x₀ = x_init`
    - `xₖ₊₁ = A * xₖ + B * uₖ` through the function `f(x, u) = A*x + B*u`.

- For nonlinear models (`T ≠ 0`), delegates to a user-defined function `f(model, x, u)` that adds constraints to `model`.

# Arguments:
- `model::JuMP.Model`                       : JuMP model to which constraints are added.
- `x::AbstractMatrix{<:JuMP.VariableRef}`   : State variable array over the prediction horizon.
- `u::AbstractMatrix{<:JuMP.VariableRef}`   : Control input variable array over the prediction horizon.
- `f::Function`                             : - If `T == 0`: a state transition function (e.g., `(x, u) -> A*x + B*u`).
                                              - If `T ≠ 0` : a nonlinear constraint function with signature `f(model, x, u)`.
- `x_init::Vector{<:Real}`                  : Initial condition for the state.
- `N::Int64`                                : Number of prediction steps (horizon).
- `T::Int64`                                : Type selector (0 for linear dynamics, ≠ 0 for nonlinear dynamics).
"""
function set_ModelConstraints!(model::JuMP.Model,
    x::AbstractMatrix{<:JuMP.VariableRef},
    u::AbstractMatrix{<:JuMP.VariableRef},
    f::Function, x_init::Vector{<:Real}, N::Int64, T::Int64)

    # Fix initial condition to the model
    for i in eachindex(x_init)
        JuMP.fix(x[i, 1], x_init[i])
    end 

    # Dynamics constraints
    if T == 0
        # Linear dynamics: xₖ₊₁ = f(xₖ, uₖ)
        for k in 1:N
            JuMP.@constraint(model,  x[:, k+1] .==  f(x[:, k], u[:, k]))
        end 
    else 
        # Nonlinear dynamics: handle by user define function f(model, x, u)
        f(model, x, u) 
    end 
end

"""
    function set_SoftConstraint!(model, x, u, g, ref, N, obj, T;
    mode= :Min)
    
# Description:
Adds the soft constraint objective to the MPC model, using the observation function `g` and a cost function `obj`.

# Arguments:
- `model::JuMP.Model`                       : JuMP model to optimize.
- `x::AbstractMatrix{<:JuMP.VariableRef}`   : State variable array over the prediction horizon.
- `u::AbstractMatrix{<:JuMP.VariableRef}`   : Control input variable array over the prediction horizon.
- `g::Function`                             : Observation function `g(x)` or `g(model, x)` depending on problem type.
- `ref::Matrix{<:Real}`                     : Reference trajectory to track (dimensions should match `g(∙)`).
- `N::Int64`                                : Prediction horizon.
- `obj::Function`                           : Objective function with signature `obj(x, u, y, ref)`.
- `T::Int`                                  : Type selector (0 = linear, otherwise nonlinear).

# Optional Arguments: 
- `mode::Symbol = :Min` : Optimization direction (`:Min` or `:Max`).

# Notes: 
- If `T == 0`   : the problem is linear, and `g(x)` is used in expressions.
- If `T ≠ 0`    : the problem is nonlinear, and the user define `g(model, x)` function is evaluated.
- The `obj` function computes the objective using `x`, `u`, `y`, and `ref`.
"""
function set_SoftConstraint!(
    model::JuMP.Model,
    x::AbstractMatrix{<:JuMP.VariableRef},
    u::AbstractMatrix{<:JuMP.VariableRef},
    g::Function,
    ref::Matrix{<:Real},
    N::Int64,
    obj::Function,
    T::Int;
    mode::Symbol = :Min
    )
    # Validate objective mode
    if mode ∉ [:Min, :Max]
        error("MPC: Invalid objective mode: $mode. Use :Min or :Max.")
    end 

    # Objective function definition 
    if T == 0
        # Linear case: compute predicted outputs y = g(x)
        JuMP.@expression(model, y[1:size(ref, 1), i = 1:N], g(x[:, i+1]))
        y_mat = reduce(hcat, y)
        # Cost function expression using x, u, y, ref
        JuMP.@expression(model, obj_cost, obj(x[:, 2:N+1], u, y_mat, ref))
    else 
        # Nonlinear case: user-defined g(model, x) computes output
        y_nl = g(model, x)
        # Call cost function (should return a scalar JuMP expression)
        obj_cost = obj(x[:, 2:N+1], u, y_nl, ref)
    end 

    # Set the optimization objective
    if mode == :Min
        JuMP.@objective(model, Min, obj_cost)
    else
        JuMP.@objective(model, Max, obj_cost)
    end
end

"""
    truncVector(vec, k, kmax, N) where T<:Real 

# Description:
Truncates the reference matrix `vec` to match the prediction horizon `N`, starting at index `k+1`.
If `k + N > kmax`, it pads the remaining columns with the last available column of `vec`.

# Arguments:
- `vec::Matrix{T}`          : Reference matrix of size (p, Time).
- `k::Int64`                : Current time index.
- `kmax::Int64`             : Maximum valid column index in `vec`.
- `N::Int64`                : Prediction horizon length.

# Returns:
- `truncated_vec::Matrix{T}` : Matrix of shape (p, N) suitable for the horizon.
"""
function truncVector(vec::Matrix{<:Real}, k::Int64, kmax::Int64, N::Int64)
    if(k + N <= kmax)
        # Simple case: all predicted values are available in vec
        return vec[:, k+1:k+N] 
    else
        # Padding case: replicate the last column as needed
        n = k + N - kmax
        outside_Pred = ones(size(vec, 1), n)
        for i in size(vec, 1)
            outside_Pred[i, :] = outside_Pred[i, :] * vec[i, kmax]
        end 
        return hcat(vec[:, k+1:kmax], outside_Pred)
    end
end 

"""
    get_index(value, vec, init_index=1) where T<:Real

# Description:
Finds the first index `i` in `vec` such that `vec[i] ≥ value`.
Starts from `init_index` and searches forward.

# Arguments:
- `value::T`            : The target value to compare.
- `vec::Vector{T}`      : A vector of ordered values.
- `init_index::Int=1`   : Optional index to start the search from.

# Returns:
- `index::Int` : First index such that `vec[index] ≥ value`, or `last index` if none is found.
"""
function get_index(value::T, vec::Vector{T}, init_index::Int64=1) where T<:Real
    index = init_index
    n     = length(vec)

    # Stop before reaching out-of-bounds
    while(value > vec[index] && index < n)
        index += 1
    end

    return index
end
