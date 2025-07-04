
mutable struct BipedRobot
    # Global parameters
    Ts::Union{Int64, Float64}
    zc::Union{Int64, Float64}
    ΔCoMz::Float64
    g::Union{Int64, Float64}

    # Robot description 
    zmax::Union{Int64, Float64}
    # Δz::Union{Int64, Float64}
    offset_hip_to_motor::Union{Int64, Float64}
    offset_ankle_to_foot::Union{Int64, Float64}
    footLength_x::Union{Int64, Float64}

    # Foot pattern parameters
    Lmax::Union{Int64, Float64}
    θ_max::Union{Int64, Float64}
    d::Float64
    initial_position::Vector
    isLeftSupport::Bool
    xPath::Union{Vector, StepRangeLen}
    yPath::Union{Vector, StepRangeLen}

    # ZMP trajectory generator parameters
    Tstep::Union{Int64, Float64}
    Tdelay::Union{Int64, Float64}
    Twait::Union{Int64, Float64}
    δ::Float64

    # MPC parameters
    StartIndex::Int64
    predictionTime::Union{Float64, Int64}

    # Swing Foot trajectory parameters
    hstep::Float64
    Tver::Union{Int64, Float64}
end

"""

Constructor 
"""
function BipedRobot(;
    paramFileName::String = "param.jl",
)
    
    # urdfpath() = joinpath(packagepath(), URDFfileName)
    include(joinpath(packagepath(), paramFileName))
    return BipedRobot(
            Ts,
            zc,
            ΔCoMz,
            g,
            zmax,
            offset_hip_to_motor,
            offset_ankle_to_foot,
            footLength_x,
            Lmax,
            θ_max,
            d,
            initial_position,
            isLeftSupport,
            xPath,
            yPath,
            Tstep,
            Tdelay,
            Twait,
            δ,
            StartIndex,
            predictionTime,
            hstep,
            Tver,
        )
end