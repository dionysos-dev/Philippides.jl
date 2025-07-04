"""
    struct ForwardKinematics

Container for forward kinematics function. 
"""
struct ForwardKinematics
    CoM::Function                   # Function to compute the CoM position
    J_CoM::Function                 # Function to compute the Jacobian of the CoM w.r.t joint configuration
    J̇_CoM::Function                 # Function to compute the Hessian of the CoM w.r.t joint configuration

    F::Vector{<:Function}           # Function to compute both feet positions
    J_F::Vector{<:Function}         # Function to compute the Jacobian of both feet
    J̇_F::Vector{<:Function}         # Function to compute the Hessian of both feet
end 

"""
    struct ForwardDynamics

Container for forward dynamics function. 
"""
struct ForwardDynamics
    Mass_Matrix::Function           # Function to compute the mass/inertia matrix M(q)
    Bias::Function                  # Function to compute Coriolis, centrifugal, and gravity terms: C(q, q̇)q̇ + G(q)
end 

"""
    define_ForwardDynamics(Mass_Matrix, Bias)

# Description:
Constructs a `ForwardDynamics` instance that groups together functions related to the system's dynamics.

# Arguments:
- `Mass_Matrix::Function` : A function that returns the joint-space mass matrix M(q), based on the robot's configuration.
- `Bias::Function`        : A function that computes the nonlinear bias terms (e.g., Coriolis, centrifugal, and gravity), typically denoted as C(q, q̇)q̇ + G(q).

# Returns:
- `ForwardDynamics` : A structure containing the provided dynamics functions.
"""
function define_ForwardDynamics(Mass_Matrix::Function, Bias::Function)
    return ForwardDynamics(Mass_Matrix, Bias)
end 

"""
    define_ForwardKinematics(CoM, J_CoM, J̇_CoM, F, J_F, J̇_F)

# Description:
Constructs a `ForwardKinematics` instance that groups together functions related to the robot’s kinematics,
including the CoM, left and right foot positions, their Jacobians, and their Hessians.

# Arguments:
- `CoM::Function`   : Function that computes the CoM position from joint configuration `q`.
- `J_CoM::Function` : Function that computes the Jacobian of the CoM w.r.t. `q`.
- `J̇_CoM::Function` : Function that computes the Hessian of the CoM w.r.t. `q`.
- `F::Function`     : Function that computes both feet positions from `q`.
- `J_F::Function`   : Function that computes the Jacobian of both feet w.r.t. `q`.
- `J̇_F::Function`   : Function that computes the Hessian of both feet w.r.t. `q`.

# Returns:
- `ForwardKinematics` : A struct containing all provided kinematic functions, suitable for use in simulation,
  control, or optimization frameworks.
"""
function define_ForwardKinematics(
    CoM::Function, J_CoM::Function,
    J̇_CoM::Function, F::Vector{<:Function},
    J_F::Vector{<:Function}, J̇_F::Vector{<:Function})

    return ForwardKinematics(CoM, J_CoM, J̇_CoM, F, J_F, J̇_F)
end 