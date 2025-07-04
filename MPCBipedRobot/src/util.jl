
"""
    cartTableModel(zc, g)

# Description:
Returns the continuous-time state-space representation of the cart-table model used for ZMP-based control.  
The model is derived from the Linear Inverted Pendulum (LIPM) with constant Center of Mass (CoM) height `zc`.

# Source: 
1. Kajita S, Kanehiro F, Kaneko K, Fujiwara K, Harada K, Yokoi K, et al. 
Biped walking pattern generation by using preview control of zero-moment point. 
In: 2003 IEEE International Conference on Robotics and Automation (Cat No03CH37422) [Internet]. 
Taipei, Taiwan: IEEE; 2003 [cité 20 déc 2022]. p. 1620‑6. 
Available at: http://ieeexplore.ieee.org/document/1241826/

# Arguments:
- `zc::Union{Int64, Float64}` : Constant height of the Center of Mass (CoM).
- `g::Union{Int64, Float64}`  : Gravitational acceleration.

# Returns:
- `A::Matrix{Float64}` : State transition matrix of size (3, 3).
- `B::Vector{Float64}` : Input matrix of size (3, 1).
- `C::Vector{Float64}` : Output matrix mapping state to ZMP output (1, 3).
"""
function cartTableModel(zc::Union{Int64, Float64}, g::Union{Int64, Float64})
    A = [
        0 1.0 0
        0 0 1.0
        0 0 0
    ]
    B = [
        0.0
        0.0
        1.0
    ]
    C = [1 0 -zc/g]
    return A, B, C
end

"""
    DCMModel(zc, g)

# Description:
Returns the continuous-time state-space representation of the Divergent Component of Motion (DCM) model.  
This simplified linear system captures the unstable part of the Linear Inverted Pendulum dynamics,  
assuming a constant Center of Mass (CoM) height `zc`.

# Arguments:
- `zc::Union{Int64, Float64}` : Constant height of the Center of Mass (CoM).
- `g::Union{Int64, Float64}`  : Gravitational acceleration.

# Returns:
- `A::Matrix{Float64}` : State transition matrix of size (2, 2).
- `B::Vector{Float64}` : Input matrix of size (2,).
- `C::Vector{Float64}` : Output matrix of size (1, 2), used to compute the DCM.
"""
function DCMModel(zc::Union{Int64, Float64}, g::Union{Int64, Float64})
    # State vector: x = [xc; ẋc]
    # where:
    #   xc  = horizontal CoM position
    #   ẋc  = horizontal CoM velocity

    # Input: u  = ẍc                      (horizontal CoM acceleration)
    # Output: y = xc + sqrt(zc/g) * ẋc    (Divergent Component of Motion)

    A = [
        0 1.0 
        0 0 
    ]
    B = [
        0.0
        1.0
    ]
    C = [1 sqrt(zc/g)]
    return A, B, C
end


"""
    continuous2discrete(A, B, C, Ts) 

# Description:
Converts a continuous-time state-space model (A, B, C) to its discrete-time counterpart (A_d, B_d, C_d) 
using the sample time `Ts`. The matrix exponential is used to compute `A_d`.  
For `B_d`, the method handles both full-rank and rank-deficient `A` matrices.

In particular, for the cart-table model where `rank(A) < 3`, the inverse of `A` does not exist,  
so the discrete input matrix `B_d` is approximated using a Taylor series expansion up to 4th order.

# Arguments:
- `A::AbstractMatrix{T}`    : Continuous-time state transition matrix.
- `B::AbstractVecOrMat{T}`  : Continuous-time input matrix.
- `C::AbstractMatrix{T}`    : Continuous-time output matrix.
- `Ts::Union{Int64, Float64}`   : Discretization sampling period.

# Returns:
- `A_d::Matrix{T}` : Discrete-time state transition matrix.
- `B_d::Matrix{T}` : Discrete-time input matrix.
- `C_d::Matrix{T}` : Discrete-time output matrix (unchanged).
"""
function continuous2discrete(
    A::AbstractMatrix{T}, 
    B::AbstractVecOrMat{T}, 
    C::AbstractMatrix{T}, 
    Ts::Union{Int64, Float64}
) where {T <: Real}
    # Compute discrete-time A_d using matrix exponential
    Ad = exp(A * Ts)

    # Compute discrete-time B_d using truncated Taylor series if A is singular
    if rank(A) < size(A, 1)  # If A is not full-rank
        Bd = B * Ts + 
             (1/2) * A * B * Ts^2 + 
             (1/6) * A^2 * B * Ts^3 + 
             (1/24) * A^3 * B * Ts^4  # Using more terms for better accuracy
    else
        # Exact solution for B_d if A is invertible
        Bd = A \ (Ad - I) * B  # Equivalent to (e^{A Ts} - I) A^{-1} B
    end

    # Output matrix remains unchanged
    Cd = C 

    return Ad, Bd, Cd
end

"""
    eye(dims; T::Type=Bool)

# Description:
Creates an identity matrix of size `dims × dims` with optional element type `T`.  
By default, the matrix is of type `Bool`, which is useful for logical indexing or mask creation.

# Arguments:
- `dims::Int64`     : Dimension of the square identity matrix.
- `T::Type=Bool`    : (Optional) Element type of the matrix. Defaults to `Bool`.

# Returns:
- `I_mat::Matrix{T}` : Identity matrix of type `T` and size `(dims, dims)`.
"""
function eye(dims::Int64; T::Type=Bool)
    return Matrix{T}(I, dims, dims)
end

"""
    getSplineCoeff(tStart, tEnd, xStart, xEnd)

# Description:
Solves for the coefficients of a cubic polynomial spline:

    x(t) = a₀ + a₁·t + a₂·t² + a₃·t³

based on position and velocity boundary conditions.  
This spline is used to smoothly interpolate a ZMP trajectory along an axis over a time segment.

# Boundary Conditions:
- `x(tStart)    = xStart`   : Initial position
- `x(tEnd)      = xEnd`     : Final position
- `x'(tStart)   = 0`        : Initial velocity (zero)
- `x'(tEnd)     = 0`        : Final velocity (zero)

# Arguments:
- `tStart::Float64` : Start time.
- `tEnd::Float64`   : End time.
- `xStart::Float64` : Position at start time.
- `xEnd::Float64`   : Position at end time.

# Returns:
- `coeffs::Vector{Float64}` : Coefficients `[a₀, a₁, a₂, a₃]` of the cubic polynomial satisfying the constraints.
"""
function getSplineCoeff(tStart::Float64, tEnd::Float64, xStart::Float64, xEnd::Float64)
    A = [
        1 tStart tStart^2 tStart^3
        1 tEnd tEnd^2 tEnd^3
        0 1 2*tStart 3*tStart^2
        0 1 2*tEnd 3*tEnd^2
    ]
    b = [xStart; xEnd; 0; 0]
    return A \ b
end

"""
    spline(t, coeff)

# Description:
Evaluates a polynomial (e.g., cubic spline) at one or more time values `t`,  
given a vector of polynomial coefficients `coeff`.

The spline is defined as:

    x(t) = a₀ + a₁·t + a₂·t² + a₃·t³ + ... + aₙ·tⁿ

where `coeff = [a₀, a₁, ..., aₙ]`.

# Arguments:
- `t::Union{Vector, StepRangeLen, Float64}` : Time value(s) at which to evaluate the spline.
- `coeff::Vector{<:Real}`                   : Polynomial coefficients of increasing order.

# Returns:
- `x::Vector{Float64}` : Evaluated spline value(s) corresponding to each `t`.
"""
function spline(t::T, coeff::Vector) where {T <: Union{Vector, StepRangeLen, Float64}}
    sum = zeros(length(t))
    for i in 1:length(coeff)
        sum = sum .+ coeff[i] * t .^ (i - 1)
    end
    return sum
end

"""
    openCSV(filename)

# Description:
Opens a CSV file and returns its contents as a `DataFrame` in table format.  
This function assumes the data starts from the **second row** (`header = [2]`),  
which is useful if the first row contains metadata or units.

# Arguments:
- `filename::String` : Path to the CSV file to be loaded.

# Returns:
- `data::DataFrame` : Parsed data in table format from the CSV file.
"""
function openCSV(filename::String)
    # Define the path to the CSV file
    csvpath() = joinpath("$(filename)")
    # Read the CSV file into a DataFrame
    data = (CSV.read(csvpath(), DataFrame; header = [2], delim = ','))
    return data
end


"""
    differentiate(p, Δt; init_nul::Bool=false)

# Description:
Computes the numerical derivative of a vector `p` using finite difference schemes:
- Central difference for interior points
- Forward difference at the first point
- Backward difference at the last point

Optionally, the first value of the derivative can be set to zero using the `init_nul` flag.

# Arguments:
- `p::Vector`             : Input signal to differentiate (e.g., position or ZMP).
- `Δt::Float64`           : Time step between samples.
- `init_nul::Bool=false`  : If true, sets the first derivative value to 0.0.

# Returns:
- `ṗ::Vector{Float64}`   : Approximated time derivative of the input signal.
"""
function differentiate(p::Vector, Δt::Float64; init_nul::Bool=false)
    N = length(p)
    ṗ = Vector{Float64}(undef, N)

    for i in 1:N
        if i == 1
            ṗ[i] = (p[i+1] - p[i]) / Δt  # forward difference
        elseif i == N
            ṗ[i] = (p[i] - p[i-1]) / Δt  # backward difference
        else
            ṗ[i] = (p[i+1] - p[i-1]) / (2 * Δt)  # central difference
        end
    end

    if init_nul
        ṗ[1] = 0.0
    end

    return ṗ
end

"""
    local2world(state, body, localVec)

# Description:
Transforms a vector `localVec` expressed in the local frame of a given rigid `body`  
to the world frame, using the kinematic `state` of the robot mechanism.

This is useful, for example, to compute the global position of a foot point, sensor,  
or any local feature attached to a body.

# Arguments:
- `state::MechanismState` : Current kinematic state of the mechanism.
- `body::RigidBody`       : The rigid body to which the local vector is attached.
- `localVec::Array`       : 3D position vector expressed in the local body frame.

# Returns:
- `worldVec::Vector{Float64}` : The input vector expressed in the world frame.
"""
function local2world(state::MechanismState, body::RigidBody, localVec::Array)
    # Get the transformation from the body frame to the world (root) frame
    tf_world_to_body = transform_to_root(state, default_frame(body))

    # Extract translation and rotation from the transform
    trans   = translation(tf_world_to_body)
    Rot     = rotation(tf_world_to_body)

    # Apply the transformation to the local vector
    return Vector(trans + Rot * localVec)
end 
