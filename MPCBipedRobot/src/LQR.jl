"""
Compute_K_M

Compute the linearize Jacobian of the inverse mass matrix `M⁻¹(q)` with respect to `q`. 
and return a function `K(q̇) = = J_M⁻¹(q) ⋅ q̇ = q  δM⁻¹(q)/δq ⋅ q̇`.

# Inputs
- `mechanism::Mechanism`: Robot model from RigidBodyDynamics.jl.
- `q::AbstractVector{T}`: Joint configuration vector.

# Outputs
- `K(q̇)`: Function that returns ∂M(q)/∂q ⋅ q̇ in matrix form.
- `M`: Mass matrix M(q).

# Notes
Used for computing the time derivative of M⁻¹(q) via:
    ∂M⁻¹/∂t = - M⁻¹ ⋅ ∂M/∂t ⋅ M⁻¹
"""
function Compute_K_M(mechanism::Mechanism, q::AbstractVector{T}) where T

    function Mass(q::AbstractVector{K}) where K
        state = MechanismState{K}(mechanism)
        set_configuration!(state, q)
        return mass_matrix(state)
    end

    # Compute jacobian of M(q)
    jac = ForwardDiff.jacobian(x -> Mass(x), q)  # δM(q)/δq 
    M   = Mass(q)

    # Problem size
    nq = size(M, 1)

    function K(q̇::AbstractVector{K}) where K 
        k = Matrix{K}(undef, nq, nq)
        for j in 1:nq
            for i in 1:nq
                k[i, j] = jac[j*i, :]' * q̇
            end 
        end 
        return k
    end 

    return K, M
    
end

"""
Compute_JN_JṄ

Compute the Jacobians of the system bias term force `n(q, q̇)` with respect to `q` and `q̇`.

# Inputs
- `mechanism::Mechanism`: The robot mechanism.
- `q::AbstractVector{T}`: Current joint positions.
- `q̇::AbstractVector{T}`: Current joint velocities.

# Outputs
- `J_N`: Jacobian of n(q, q̇) with respect to `q`.
- `J_Ṅ`: Jacobian of n(q, q̇) with respect to `q̇`.
"""
function Compute_JN_JṄ(mechanism::Mechanism, 
    q::AbstractVector{T}, 
    q̇::AbstractVector{T}) where T

    function getJacobian(q::AbstractVector, q̇::AbstractVector) 

        # type
        if eltype(q) <: ForwardDiff.Dual
            K = eltype(q)
        else 
            K = eltype(q̇)
        end 

        # Initialize structure
        state = MechanismState{K}(mechanism)
    
        # Set joint positions and velocities (preserving the number type)
        set_configuration!(state, q)
        set_velocity!(state, q̇)
       
        # Compute system dynamics
        N = Vector{K}(RigidBodyDynamics.dynamics_bias(state))     
        return N
    end

    # Compute jacobian 
    J_N = ForwardDiff.jacobian(x -> getJacobian(x, q̇), q)
    J_Ṅ = ForwardDiff.jacobian(x -> getJacobian(q, x), q̇)

    return J_N, J_Ṅ
end 

"""
LinearizedAugmentedDynamics

Compute the linearized augmented dynamics matrices of a robotic system:
    ẋ = Al ⋅ x + Bl ⋅ u + Dl ⋅ u̇

# Inputs
- `mechanism::Mechanism`: The robot mechanism from RigidBodyDynamics.jl.
- `qr::Vector{K}`: Reference joint positions.
- `q̇r::Vector{K}`: Reference joint velocities.
- `q̈r::Vector{K}`: Reference joint accelerations.

# Outputs
- `Al`: Linearized state transition matrix.
- `Bl`: Control input matrix (w.r.t u).
- `Dl`: Derivative input matrix (w.r.t u̇).

# Notes
The returned matrices represent the dynamics:
    ẋ = [q̇; q̈; q_dddot] = Al ⋅ x + Bl ⋅ u + Dl ⋅ u̇
"""
function LinearizedAugmentedDynamics(
        mechanism::Mechanism, 
        qr::AbstractVector{K}, 
        q̇r::AbstractVector{K}, 
        q̈r::AbstractVector{K},
    ) where K

    nq       = length(qr)
    Kq̇, M    = Compute_K_M(mechanism, qr)
    J_N, J_Ṅ = Compute_JN_JṄ(mechanism, qr, q̇r)
    
    function Dynamics(x::AbstractVector{T}, u::AbstractVector, u̇::AbstractVector) where T
        
        state            = MechanismState{T}(mechanism)  # Ensure compatibility with ForwardDiff
    
        # Set joint positions and velocities (preserving the number type)
        set_configuration!(state, x[1:nq])
        set_velocity!(state, x[nq+1:2*nq])

        # Compute system dynamics including contact wrenches
        M = Matrix{T}(mass_matrix(state))  
        N = Vector{T}(RigidBodyDynamics.dynamics_bias(state))   
        M_inv = inv(M)
    
        # Solve for acceleration
        dq   = x[nq+1:2*nq]    # Velocities remain the same
        ddq  = x[2*nq+1:3*nq]   

        J_invM = - M_inv * Kq̇(dq) * M_inv
        dddq = M_inv * (u̇ - J_N * dq - J_Ṅ * ddq) + J_invM * (u - N)    
        return vcat(dq, ddq, dddq)  # Return state derivative
    end

    # Create ref vector
    x_eq = vcat(qr, q̇r, q̈r)
    u_eq = zeros(nq)#zeros(size(B, 2))
    u̇_eq = copy(u_eq)

    # Compute linearized systems 
    Al = ForwardDiff.jacobian(x -> Dynamics(x, u_eq, u̇_eq), x_eq)
    Bl = ForwardDiff.jacobian(u -> Dynamics(x_eq, u, u̇_eq), u_eq)
    Dl = ForwardDiff.jacobian(u̇ -> Dynamics(x_eq, u_eq, u̇), u̇_eq)

    return Al, Bl, Dl 
end 

"""
LQR_discretisation

Discretize continuous-time linear system:
    ẋ = Al ⋅ x + Bl ⋅ u + Dl ⋅ u̇

# Inputs
- `A::Matrix{T}`: Continuous-time state matrix.
- `B::Matrix{T}`: Continuous-time control matrix.
- `D::Matrix{T}`: Continuous-time feedthrough derivative matrix.
- `Δt::Float64` : Sampling time.

# Outputs
- `Ad`: Discrete-time state transition matrix.
- `Bd`: Discrete-time input matrix for `u`.
- `Dd`: Discrete-time input matrix for `u̇`.

# Notes
If A is singular (not full rank), Taylor expansion is used for approximation. 
If A is invertible, matrix exponential-based exact formulas are used.
"""
function LQR_discretisation(
        A::AbstractMatrix{T}, 
        B::AbstractVecOrMat{T}, 
        D::AbstractMatrix{T}, 
        Δt::Union{Int64, Float64}
    ) where {T <: Real}
        # Compute discrete-time A_d using matrix exponential
        Ad = exp(A * Δt)
    
        # Compute discrete-time B_d using truncated Taylor series if A is singular
        if rank(A) < size(A, 1)  # If A is not full-rank
            Bd = B *  Δt + 
                 (1/2) * A * B * Δt^2 + 
                 (1/6) * A^2 * B * Δt^3 + 
                 (1/24) * A^3 * B * Δt^4  # Using more terms for better accuracy
            Dd = D *  Δt + 
                 (1/2) * A * D * Δt^2 + 
                 (1/6) * A^2 * D * Δt^3 + 
                 (1/24) * A^3 * D * Δt^4  # Using more terms for better accuracy
            
        else
            # Exact solution for B_d if A is invertible
            Bd = A \ (Ad - I) * B  # Equivalent to (e^{A Ts} - I) A^{-1} B
            Dd = A \ (Ad - I) * D 
        end
        return Ad, Bd, Dd
end

"""
Riccati_DTFH

Solve the discrete-time finite-horizon Riccati recursion to compute the optimal 
feedback gains `K[k]` for time-varying LQR control.

# Inputs
- `AD::Array{T}`: Discrete-time system matrix A.
- `BD::Array{T}`: Discrete-time input matrix B.
- `Ts::T`       : Total time horizon.
- `Δt::T`       : Sampling interval.
- `Qd::Array{T}`: Stage cost weight matrix on states.
- `Qdf::Array{T}`   : Terminal cost matrix on final state.
- `Rd::Array{T}`    : Stage cost weight matrix on control input.

# Output
- `K`: Array of optimal feedback gain matrices K[k] over the finite horizon.

# Notes
Used to compute time-varying LQR gains for constrained control problems over finite horizons.
"""
function Riccati_DTFH(
        AD::Array{T},
        BD::Array{T},
        Ts::T, 
        Δt::T, 
        Qd::Array{T}, 
        Qdf::Array{T},
        Rd::Array{T} 
    ) where T <: Union{Int64, Float64}

    # Finite horizon
    N = round(Int, Ts / Δt)  # Number of time steps

    println(N)
    # Initialize Riccati recursion
    P = Array{Matrix{Float64}}(undef, N+1)  # Store P_k
    P[N+1] = Qdf  # Terminal cost

    # Backward Riccati recursion
    K = Array{Matrix{Float64}}(undef, N)  # Store K_k

    for k in N:-1:1
        Pk1 = P[k+1]
        S = Rd + BD' * Pk1 * BD
        K[k] = inv(S) * BD' * Pk1 * AD
        P[k] = Qd + AD' * Pk1 * AD - AD' * Pk1 * BD * K[k]
    end
    return K
end 