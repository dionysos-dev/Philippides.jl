using Plots
using LaTeXStrings
using DelimitedFiles            
include("utils_conversion.jl")

#----------------------------------------------------------------------------
#                       Folder DETAILS
#----------------------------------------------------------------------------

F1 = false
f1 = "Robot_50Hz"

F2 = true
f2 = "Robot_200Hz"

#----------------------------------------------------------------------------
#                       FILE DETAILS
#----------------------------------------------------------------------------
freq = 200.0            # Frequency of measurements
goalfreq = 1e3          # Goal frequency after padding /!\ has to be a mulitple of freq

interval = (0.0,2.0)      # Plot interval

# [t,HL,KL,HR,KR] (LabView) -> [t,HL,HR,KL,KR] (Code)
# H = Hip, K = Knee, R = Right, L = Left, t = Time
permutation = [(1,1,1.0),(2,2,1.0), (3,4,1.0),(4,3,-1.0),(5,5,-1.0)]  

Δt = 1/freq                # 1/freq
extension_factor = Int64(div(goalfreq, freq))  # This will give you 2 as an Int64
# Padding between two values
max_lines = 20001       # Limit the number of lines after padding
remove_temp_file = true # removes non permutated files
#----------------------------------------------------------------------------

#----------------------------------------------------------------------------
# Folder process frunction
#----------------------------------------------------------------------------
function folder_full_process(folder_name::String)

    path = joinpath(@__DIR__, "..", "data", folder_name)
 # Raw data files
    raw_position = joinpath(path, "Raw", "Position.txt")
    raw_velocity = joinpath(path, "Raw", "Velocity.txt")
    raw_voltage  = joinpath(path, "Raw", "Voltage.txt")

 # Input data files
    in_position = joinpath(path, "Inputs", "Position.txt")
    in_velocity = joinpath(path, "Inputs", "Velocity.txt")
    in_voltage  = joinpath(path, "Inputs", "Voltage.txt")

    in_position_temp = joinpath(path, "Inputs", "Position_temp.txt")
    in_velocity_temp = joinpath(path, "Inputs", "Velocity_temp.txt")
    in_voltage_temp  = joinpath(path, "Inputs", "Voltage_temp.txt")

    out_torque_v_om  = joinpath(path, "Outputs", "Torque.txt")

 # Simulation data files
    simu_torque_v_om = joinpath(path, "Simulations", "Torque.txt")

 # Preprocessing (1/2)
    compute_transform(raw_position, in_position_temp, transform_position, freq)
    compute_transform(raw_velocity, in_velocity_temp, transform_velocity, freq)
    compute_transform(raw_voltage , in_voltage_temp , transform_voltage , freq)

 # Preprocessing (2/2)
    apply_permutation(in_position_temp, in_position, permutation, remove_temp_file)
    apply_permutation(in_velocity_temp, in_velocity, permutation, remove_temp_file)
    apply_permutation(in_voltage_temp , in_voltage , permutation, remove_temp_file)

 # Processing
    compute_model(in_voltage , in_velocity, out_torque_v_om, to_torque_model)

 # Post-processing  
    extend_data(out_torque_v_om, simu_torque_v_om, Δt, extension_factor; max_lines = max_lines) 
end
#----------------------------------------------------------------------------

#----------------------------------------------------------------------------
# FLODER PROCESSING
#----------------------------------------------------------------------------
if(F1)
   folder_full_process(f1)
end
if(F2)
   folder_full_process(f2)
end
#----------------------------------------------------------------------------