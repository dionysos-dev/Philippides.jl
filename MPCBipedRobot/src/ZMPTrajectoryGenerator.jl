"""
    struct ZMPTrajectory

Represents a time-discretized Zero Moment Point (ZMP) trajectory, typically used
in walking pattern generation and stability analysis.

# Source
1. Vukobratović M, Stepanenko J. On the stability of anthropomorphic systems. Mathematical Biosciences. oct 1972;15(1‑2):1‑37. 

# Fields
- `ZMP::Vector{<:AbstractVector}` :
    List of 2D vectors representing the ZMP position at each time step.

- `timeVec::Vector{Float64}` :
    Time vector corresponding to each ZMP point. Must be same length as `ZMP`.

- `isDSP::Vector{Bool}` :
    Boolean vector indicating if each time step is in Double Support Phase (DSP).

- `Limx::Vector{Tuple{Float64, Float64}}` :
    List of x-axis ZMP bounds (e.g., support polygon limits) for each time step.
"""
struct ZMPTrajectory
    ZMP::Array
    timeVec::Array
    isDSP::Array
    Limx::Array
end

"""
    struct ContactForce

Stores the binary contact state (in contact or not) of each foot over time.

# Fields
- `Contact_left::Vector{Float64}` :
    Contact indicator values for the left foot (1.0 for contact, 0.0 for no contact) at each time step.

- `Contact_right::Vector{Float64}` :
    Contact indicator values for the right foot (1.0 for contact, 0.0 for no contact) at each time step.
"""
struct ContactForce
    Contact_left::Vector{Float64}
    Contact_rigth::Vector{Float64}
end 

"""
    ZMPTrajectory(; br::BipedRobot, fp::FootPlanner, check::Bool = false, save::Bool = false, savePath::String = "")

Generates a ZMP trajectory and corresponding contact phase information for a given bipedal robot and footstep plan.

# Arguments
- `br::BipedRobot`  : Robot model containing trajectory and path data.
- `fp::FootPlanner` : Footstep planner object containing left/right step sequences and centerline.
- `check::Bool`     : If `true`, plots the resulting ZMP trajectory for visual inspection (default: `false`).
- `save::Bool`      : If `true`, saves the plot as a PNG to `savePath` (default: `false`).
- `savePath::String`: Output folder path for saving the figure (should end with `/` if used).

# Returns
- `ZMPTrajectory`   : Struct containing the ZMP reference trajectory, time vector, support phase flags, and lateral limits.
- `ContactForce`    : Struct containing binary contact sequences for the left and right foot.

# Notes
- Internally calls `computeZMPTrajectory(br, fp)` which must return:
  `(ZMP, timeVec, isDSP, Limx, Contact_left, Contact_right)`
"""
function ZMPTrajectory(; br::BipedRobot, fp::FootPlanner, check::Bool = false, save::Bool=false, savePath::String="")
    ZMP, timeVec, isDSP, Limx, Contact_left, Contact_rigth = computeZMPTrajectory(br::BipedRobot, fp::FootPlanner)
    if (check)
        ZMPplot = reduce(hcat, ZMP)
        timeplot = reduce(vcat, timeVec)
        plt_zmp =
            plot(; title = "ZMP Trajectory",
             xlabel = L"$X$ [m]", ylabel = L"$Y$ [m]", dpi = 600)
        xpath = br.xPath
        ypath = br.yPath
        right_plot = reduce(hcat, fp.right)
        left_plot = reduce(hcat, fp.left)
        center = fp.center
        plot!(xpath, ypath; label = "Reference path", lw = 2)
        scatter!(left_plot[1, :], left_plot[2, :]; shape = :rect, label = "Left")
        scatter!(right_plot[1, :], right_plot[2, :]; shape = :rect, label = "Right")
        scatter!(
            getindex.(center, 1),
            getindex.(center, 2);
            label = "Center",
            mc = :black,
            markershape = :xcross,
        )
        plot!(ZMPplot[1, :], ZMPplot[2, :]; label = L"\textbf{p}_{ref}", lw = 2)
        display(plt_zmp)
        if save
            savefig(plt_zmp, savePath*"plt_ZMPTrajectoryGenerator.png")
        end 
    end
    return ZMPTrajectory(ZMP, timeVec, isDSP, Limx), ContactForce(Contact_left, Contact_rigth)
end

"""
    computeZMPTrajectory(br::BipedRobot, fp::FootPlanner)

Constructs the reference ZMP trajectory by combining three main parts:
1. An initial delay phase at the beginning of walking.
2. A double support phase (DSP) where both feet are in contact with the ground.
3. A single support phase (SSP) or swing phase, where one foot is in contact.

# Details
- The initial delay holds the ZMP fixed at the initial center of mass position.
- For each walking step:
  - A **cubic spline** interpolates the ZMP during the DSP, ensuring smooth transitions with zero velocity at endpoints.
  - The SSP maintains a **constant ZMP** position, aligned with the supporting foot.

# Returns
- `ZMP`     : Vector of 2D ZMP positions over time.
- `timeVec` : Time vector corresponding to the ZMP trajectory.
- `isDSP`   : Boolean vector indicating DSP (true) or SSP (false).
- `Limx`    : Bounds on the ZMP in the x-direction during each phase.
- `Contact_left`    : Binary vector (Float64) indicating left foot contact over time.
- `Contact_right`   : Binary vector (Float64) indicating right foot contact over time.
"""
function computeZMPTrajectory(br::BipedRobot, fp::FootPlanner)
    isLeftSupport = br.isLeftSupport
    δ = br.δ
    Tstep = br.Tstep
    Ts = br.Ts
    Tdelay = br.Tdelay
    Twait = br.Twait
    right = fp.right
    left = fp.left
    center = fp.center
    halftfootLength_x = br.footLength_x / 2

    DSPtime = (1:(δ * (Tstep / Ts))) * Ts
    SSPtime = (1:((1 - δ) * (Tstep / Ts))) * Ts
    stepNum = 1                                             # step index
    left_flag = ~isLeftSupport                              # Initiate the move foot 

    timeVec = Array{Float64}[]
    ZMP = Array{Float64}[]
    isDSP = Array{Bool}[]
    Limx = Array{}[]
    Contact_left  = Array{Float64}[]
    Contact_rigth = Array{Float64}[]

    # Delay vector 
    nDelay = Int(round((Tdelay + Twait) / Ts))
    t = (0:nDelay) * Ts
    zmp = [center[1][1]; center[1][2]] .* ones(2, nDelay + 1)

    # Initiate 
    lastZMP = [center[1][1]; center[1][2]]
    lastTime = t[end]
    push!(isDSP, [true for _ in 0:nDelay])
    # Construct limit stability vector 
    push!(Limx, [zmp[1, end] .+ halftfootLength_x; zmp[1, end] .- halftfootLength_x] .* ones(2, nDelay + 1))
    # Construct normal force vector 
    push!(Contact_left, [1.0 for _ in 0:nDelay])
    push!(Contact_rigth, [1.0 for _ in 0:nDelay])

    while (stepNum <= length(center))
        if (left_flag == true)
            nextZMP = right[stepNum][1:2] # ZMP ref of the next step 
        else
            nextZMP = left[stepNum][1:2]  # ZMP of the next step       
        end
        left_flag = ~left_flag    # Change support foot 

        # Creating a cubic spline in the X direction for connecting 2 differents ZMPx 
        xSplineX = DSPtime
        coeff = getSplineCoeff(xSplineX[1], xSplineX[end], lastZMP[1], nextZMP[1])                                     # X position of the point for the spline (time data) 
        ySplineX = spline(xSplineX, coeff)

        # Same of y-coordinate
        xSplineY = DSPtime
        coeff = getSplineCoeff(xSplineX[1], xSplineX[end], lastZMP[2], nextZMP[2])                                     # X position of the point for the spline (time data) 
        ySplineY = spline(xSplineY, coeff)

        # Contruct the first part with a spline for the ZMP value 
        t = vcat(t, lastTime .+ DSPtime)  # Construct the time vector 
        lastTime = t[end]                  # get the last value of t
        zmp = vcat(zmp', hcat(ySplineX, ySplineY)) # Construct the ZMP vector 
        zmp = zmp'
        push!(isDSP, [true for _ in 1:length(DSPtime)])
        push!(Limx, [zmp[1, end] .+ halftfootLength_x; ySplineX[1] .- halftfootLength_x] .* ones(2, length(DSPtime)))
        # Contact force on both feet 
        push!(Contact_left, [1.0 for _ in 1:length(DSPtime)])
        push!(Contact_rigth, [1.0 for _ in 1:length(DSPtime)])

        # Construct the last part for the time vector 
        t = vcat(t, lastTime .+ SSPtime)

        # For the last part, the ZMP stay constant 
        zmp = vcat(zmp', nextZMP' .* ones(length(SSPtime), 2))# Construct the ZMP vector 
        zmp = zmp'
        push!(isDSP, [false for _ in 1:length(SSPtime)])
        # Contact force on stand feet 
        if ~left_flag
            push!(Contact_rigth, [1.0 for _ in 1:length(SSPtime)])
            push!(Contact_left, [0.0 for _ in 1:length(SSPtime)])
        else 
            push!(Contact_left, [1.0 for _ in 1:length(SSPtime)])
            push!(Contact_rigth, [0.0 for _ in 1:length(SSPtime)])
        end 
      
        # Construct limit stability vector 
        push!(Limx, [zmp[1, end] .+ halftfootLength_x; zmp[1, end] .- halftfootLength_x] .* ones(2, length(SSPtime)))



        push!(timeVec, t)      # add to the variable, first vector in it is the wait and delay plus the first DSB and SSP
        push!(ZMP, zmp)        # add to the variable 

        lastZMP = zmp[1:2, end]     # Get last value 
        lastTime = t[end]               # Get last value 
        t = []                 # Reset the vector 
        zmp = reshape([], 2, 0) # Reset the vector 

        stepNum = stepNum + 1  # next step 
    end
    # Both feet in contact at end 
    Contact_left[end][end]  = 1.0
    Contact_rigth[end][end] = 1.0
    return ZMP, timeVec, reduce(vcat, isDSP), reduce(hcat, Limx), reduce(vcat, Contact_left), reduce(vcat, Contact_rigth)
end
