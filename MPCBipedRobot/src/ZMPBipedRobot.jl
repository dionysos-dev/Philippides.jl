module ZMPBipedRobot
using LinearAlgebra
using StructArrays
using Plots
using RigidBodyDynamics
using RigidBodyDynamics.Contact
using StaticArrays
using Symbolics
using MeshCat, MeshCatMechanisms, Blink
using MechanismGeometries
# using LightXML
using GeometryTypes
using Random
using DataStructures
using LaTeXStrings
using DataFrames
using JuMP
using OSQP
# using Rotations
using ForwardDiff
using ControlSystems
using SymPy
using Ipopt

packagepath() = joinpath(@__DIR__, "..", "deps")

include("BipedRobot.jl") # Include the file into the module
export BipedRobot        # Export the function, i.e, means this function is callable when the module is call.

include("util.jl")
export cartTableModel, continuous2discrete, eye, getSplineCoeff, spline, differentiate, local2world, DCMModel

include("FootsPlacement.jl")
export FootPlanner, computeFootsPlacement

include("ZMPTrajectoryGenerator.jl")
export ZMPTrajectory, computeZMPTrajectory

include("SwingFootTrajectoryGenerator.jl")
export SwingFootTrajectory, computeSwingFootTrajectory

include("positionControl.jl")
export pid_control!, define_PID

include("RobotSimulator.jl")
export RobotSimulator,
    getMechanism,
    set_nominal!,
    set_initialbody!,
    update_visulizer!,
    show_frame!,
    trajectory_controller!,
    position_controller!,
#     simulate,
    measureZMP, 
    contact_model, 
    LQR_controller!, 
    torques_controller!, 
    acceleration_controller!, 
    show_contact_point

include("Forward_Dynamics_and_Kinematics.jl")
export define_ForwardDynamics, define_ForwardKinematics

include("InverseKinematics.jl")
export inverse_kinematic, define_IK,  plot_IK

include("ModelPredictiveControl.jl")
export computeMPControl,
       defineMpcParameter, 
       updateMpcParameter

include("MPCBipedRobot.jl")
       stateTransitionFunction,
       observationFunction, 
       objectiveFunction, 
       NL_stateTransitionFunction, 
       NL_observationFunction, 
       getAddValue

include("LQR.jl")
       export Compute_K_M, Compute_JN_JṄ, LinearizedAugmentedDynamics, Riccati_DTFH, LQR_discretisation

end 

# module ZMPBipedRobot
