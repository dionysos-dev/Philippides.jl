"""

    This function compute the linearized jacobian around q jacobian of the inverse matrix: 

        δM⁻¹(q)/δt = δM⁻¹(q)/δq * q̇

        where δM⁻¹(q)/δq = - M⁻¹(q) * δM(q)/δq * q̇ * M⁻¹(q) = - M⁻¹(q) * K(q̇) * M⁻¹(q)

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

function Compute_JN_JṄ(mechanism::Mechanism, 
    q::AbstractVector{T}, 
    q̇::AbstractVector{T}, 
    endEffector::Tuple{Vararg{String}}) where T

    endEffector_body = RigidBody[] 

    for elem in endEffector
        push!(endEffector_body, findbody(mechanism, elem))
    end 

    function getJacobian(q::AbstractVector, q̇::AbstractVector) 

        # type
        if eltype(q) <: ForwardDiff.Dual
            K = eltype(q)
        else 
            K = eltype(q̇)
        end 

        # Initialize structure
        state = MechanismState{K}(mechanism)
        dynamics_results = DynamicsResult{K}(mechanism)

        # Set joint positions and velocities (preserving the number type)
        set_configuration!(state, q)
        set_velocity!(state, q̇)
       
        # Retrieve sensor values
        RigidBodyDynamics.contact_dynamics!(dynamics_results, state)
        external_wrenches = Dict{BodyID, Wrench{K}}()
        for body in endEffector_body
            sensor = convert(Wrench{K}, RigidBodyDynamics.contact_wrench(dynamics_results, body))
            push!(external_wrenches, body.id => sensor)
        end 

        # Compute system dynamics including contact wrenches
        # N = Vector{K}(RigidBodyDynamics.dynamics_bias(state, external_wrenches))
        N = Vector{K}(RigidBodyDynamics.dynamics_bias(state))     
        return N
    end

    # Compute jacobian 
    J_N = ForwardDiff.jacobian(x -> getJacobian(x, q̇), q)
    J_Ṅ = ForwardDiff.jacobian(x -> getJacobian(q, x), q̇)

    return J_N, J_Ṅ
end 



# function LinearizedAugmentedDynamics(mechanism::Mechanism, 
#     qr::AbstractVector{K}, 
#     q̇r::AbstractVector{K}, 
#     q̈r::AbstractVector{K},
#     B::Array{K},
#     Δt::Real,
#     endEffector::Tuple{Vararg{String}}) where K

#     nq       = length(qr)
#     Kq̇, M    = Compute_K_M(mechanism, qr)
#     J_N, J_Ṅ = Compute_JN_JṄ(mechanism, qr, q̇r, endEffector)

#     endEffector_body = RigidBody[] 
#     for elem in endEffector
#         push!(endEffector_body, findbody(mechanism, elem))
#     end 
    
#     function Dynamics(x::AbstractVector{T}, u::AbstractVector) where T
        
#         state            = MechanismState{T}(mechanism)  # Ensure compatibility with ForwardDiff
#         dynamics_results = DynamicsResult{T}(mechanism)
    
#         # Set joint positions and velocities (preserving the number type)
#         set_configuration!(state, x[1:nq])
#         set_velocity!(state, x[nq+1:2*nq])

#         # Retrieve sensor values
#         RigidBodyDynamics.contact_dynamics!(dynamics_results, state)
#         external_wrenches = Dict{BodyID, Wrench{T}}()
#         for body in endEffector_body
#             sensor = convert(Wrench{T}, RigidBodyDynamics.contact_wrench(dynamics_results, body))
#             push!(external_wrenches, body.id => sensor)
#         end 

#         # Compute system dynamics including contact wrenches
#         M = Matrix{T}(mass_matrix(state))
#         N = Vector{T}(RigidBodyDynamics.dynamics_bias(state, external_wrenches))   
#         # N = Vector{T}(RigidBodyDynamics.dynamics_bias(state))   
#         M_inv = inv(M)
    
#         # Solve for acceleration
#         dq   = x[nq+1:2*nq]    # Velocities remain the same
#         ddq  = M_inv * (B * u - N)

#         J_invM = - M_inv * Kq̇(dq) * M_inv
#         dddq = M_inv * (- J_N * dq - J_Ṅ * ddq) + J_invM * (B * u - N) + (M_inv * (B ./ Δt) * u)
    
#         return vcat(dq[3:nq-2], ddq[3:nq-2], dddq[3:nq-2])  # Return state derivative
#     end

#     # Create ref vector
#     x_eq = vcat(qr, q̇r, q̈r)
#     u_eq = zeros(size(B, 2))

#     # Compute linearized systems 
#     Aa = ForwardDiff.jacobian(x -> Dynamics(x, u_eq), x_eq)
#     Ba = ForwardDiff.jacobian(u -> Dynamics(x_eq, u), u_eq)
#     Da = M \ (B ./ Δt)
#     # Da = vcat(zeros(2*nq, length(u_eq)), Da)

#     Aa = [Aa[:, 3:nq-2] Aa[:, nq+3:2*nq-2] Aa[:, 2*nq+3:3*nq-2]]
#     Da = Da[3:end-2, :]
#     Da = vcat(zeros((2*nq-8), length(u_eq)), Da)

#     return Aa, Ba, Da 
# end 



function LinearizedAugmentedDynamics(mechanism::Mechanism, 
    qr::AbstractVector{K}, 
    q̇r::AbstractVector{K}, 
    q̈r::AbstractVector{K},
    B::Array{K},
    Δt::Real,
    endEffector::Tuple{Vararg{String}}) where K

    nq       = length(qr)
    Kq̇, M    = Compute_K_M(mechanism, qr)
    J_N, J_Ṅ = Compute_JN_JṄ(mechanism, qr, q̇r, endEffector)

    endEffector_body = RigidBody[] 
    for elem in endEffector
        push!(endEffector_body, findbody(mechanism, elem))
    end 
    
    function Dynamics(x::AbstractVector{T}, u::AbstractVector, u̇::AbstractVector) where T
        
        state            = MechanismState{T}(mechanism)  # Ensure compatibility with ForwardDiff
        dynamics_results = DynamicsResult{T}(mechanism)
    
        # Set joint positions and velocities (preserving the number type)
        set_configuration!(state, x[1:nq])
        set_velocity!(state, x[nq+1:2*nq])

        # Retrieve sensor values
        RigidBodyDynamics.contact_dynamics!(dynamics_results, state)
        external_wrenches = Dict{BodyID, Wrench{T}}()
        for body in endEffector_body
            sensor = convert(Wrench{T}, RigidBodyDynamics.contact_wrench(dynamics_results, body))
            push!(external_wrenches, body.id => sensor)
        end 

        # Compute system dynamics including contact wrenches
        M = Matrix{T}(mass_matrix(state))
        # N = Vector{T}(RigidBodyDynamics.dynamics_bias(state, external_wrenches))   
        N = Vector{T}(RigidBodyDynamics.dynamics_bias(state))   
        M_inv = inv(M)
    
        # Solve for acceleration
        dq   = x[nq+1:2*nq]    # Velocities remain the same
        ddq  = x[2*nq+1:3*nq]#M_inv * (B * u - N)

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

    # retrieve controllable part 
    # Al = [Al[:, 3:nq-2] Al[:, nq+3:2*nq-2] Al[:, 2*nq+3:3*nq-2]]

    # M_inv  = inv(M)
    # J_invM = - M_inv * Kq̇( x_eq[nq+1:2*nq]  ) * M_inv
    # println("Dl", Dl)
    # println("Bl", Bl)
    # println("expt Dl ", (M_inv * B)[3:6, :])
    # println("expt Bl ", (J_invM * B)[3:6, :])
    # println((M_inv * B)[3:6, :] .== Dl[end-3:end, :])
    # println((J_invM * B)[3:6, :] .== Bl[end-3:end, :])

    # Simplifie
    return Al, Bl, Dl 
end 

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
            println("not full rank")
            Bd = B *  Δt + 
                 (1/2) * A * B * Δt^2 + 
                 (1/6) * A^2 * B * Δt^3 + 
                 (1/24) * A^3 * B * Δt^4  # Using more terms for better accuracy
            Dd = D *  Δt + 
                 (1/2) * A * D * Δt^2 + 
                 (1/6) * A^2 * D * Δt^3 + 
                 (1/24) * A^3 * D * Δt^4  # Using more terms for better accuracy
            
        else
            println("full rank")
            # Exact solution for B_d if A is invertible
            Bd = A \ (Ad - I) * B  # Equivalent to (e^{A Ts} - I) A^{-1} B
            Dd = A \ (Ad - I) * D 
        end
        return Ad, Bd, Dd
end

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