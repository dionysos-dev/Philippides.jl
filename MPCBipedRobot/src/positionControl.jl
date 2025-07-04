mutable struct PID
    Kp::Real
    Ki::Real
    Kd::Real
    ∫edt::Vector 
end

function define_PID(Kp::Real,
    Ki::Real,
    Kd::Real,
    ∫edt::Vector)
    return PID(Kp, Ki, Kd, zero(∫edt))
end 

function pid_control!(pid::PID, Δq::Vector, Δq̇::Vector, Δt::Float64)
    pid.∫edt .= pid.∫edt .+ (Δq .* Δt)
    output = pid.Kp .* Δq .+ pid.Ki .* pid.∫edt .+ pid.Kd .* Δq̇
    return output
end

# Cascaded PID Controller Function
function cascaded_pid(q_ref, q, q̇_ref, q̇, q̈_ref, q̈, dt, pid_pos, pid_vel, pid_acc)
    # Outer loop: Position -> Velocity reference
    vel_ref = update_pid!(pid_pos, q_ref - q, dt)

    # Middle loop: Velocity -> Acceleration reference
    acc_ref = update_pid!(pid_vel, vel_ref - dq, dt)

    # Inner loop: Acceleration -> Torque control
    torque = update_pid!(pid_acc, acc_ref - ddq, dt)

    return torque
end




