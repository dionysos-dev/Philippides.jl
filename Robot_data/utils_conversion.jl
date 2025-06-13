using LaTeXStrings
using CSV
using DataFrames

#-----------------------------------------------------------------------------------------------------------------
# PREPROCESSING (1/2)
# These functions allow to recover the numerical values associated to measurements made on the BipedalRobot
#  - line format - (d = data, v = value)
# raw_file.txt   : index d1 d2 d3 D4
# input_file.txt : time v1 v2 v3 v4
#-----------------------------------------------------------------------------------------------------------------

function toInt16(numbers::Vector{Int})::Vector{Int16}
    # Numbers are 16-bit unsigned integers (UInt16)
    # Reinterpret numbers as Int16
    return reinterpret(Int16, UInt16.(numbers))
end

function transform_position(numbers::Vector{Int16}, frequency::Float64)::Vector{Float64}
    # Convert Index (first number on the a line) into time
    result = [numbers[1] / frequency]  
    # Transform to °
    # For the position, the data is in [ticks]
    # The maximum value is 4095 ticks/revolution
    append!(result, (numbers[2:end] .*(360 / 4095) .- 90)*(π/180.0))
    return result
end

function transform_velocity(numbers::Vector{Int16}, frequency::Float64)::Vector{Float64}
    # Convert Index (first number on the a line) into time
    result = [numbers[1] / frequency]  
    # Transform to rad/s
    # The input data is an adimensionalisation of RPMs by 0.229rmp
    # To transform RMP into RPS (rotation per second), divide it by 60
    # Multiply then by 2π to get the rad/s
    append!(result, numbers[2:end] .*(0.229*2π / 60.0)) 
    return result
end

function transform_voltage(numbers::Vector{Int16}, frequency::Float64)::Vector{Float64}
    # Convert Index (first number on the a line) into time
    result = [numbers[1] / frequency]   
    # Transform to PWM [%]
    # A PWM of 100% corresponf to 885 -> we just need to divide what we have by 885
    # Multiply by the nominal voltage to have the voltage
    Un = 12.0 # Nominal Voltage
    append!(result, numbers[2:end] .*((1.0 / 885.0) * Un))
    return result
end

function transform_current(numbers::Vector{Int16}, frequency::Float64)::Vector{Float64}
    # Convert Index (first number on the a line) into time
    result = [numbers[1] / frequency]   
    # The current is measured by increments of 2.69 mA
    append!(result, numbers[2:end] .*(2.69/1000.0))
    return result
end

function compute_transform(input_file::String, output_file::String, func::Function, frequency::Float64)
    """
    Reformats and applies func to the raw datas from the dynamixels
    func is chosen among:
    transform_velocity, transform_voltage, transform_current and transform_position
    """
    open(input_file, "r") do infile
        open(output_file, "w") do outfile
            for line in eachline(infile)
                # Replace commas with periods to standardize decimal notation
                line = replace(line, "," => ".")
                # Parse numbers from the line, round to Int, and process
                numbers = round.(Int, parse.(Float64, split(line)))  # Parse and round to integers
                # Transform from Int to Int16 (reinterpret) and then process
                int16_numbers = toInt16(numbers)
                # Apply the transformation function
                processed_numbers = func(int16_numbers, frequency)
                # Write results to output file
                write(outfile, join(processed_numbers, " "), "\n")
            end
        end
    end
end

#-----------------------------------------------------------------------------------------------------------------
# PREPROCESSING (2/2)
# Theses function ensure coherence between the material robot and the simulation
# [t,HL,KL,HR,KR] (Robot measurements) -> [t,HL,HR,FL,FR] (Simulator)
# H = Hip, K = Knee, R = Right, L = Left, t = Time
# permutation = [(1,1,1.0),(2,2,1.0), (3,4,1.0),(4,3,-1.0),(5,5,-1.0)]
#-----------------------------------------------------------------------------------------------------------------

function apply_permutation(input_file::String, output_file::String, permutation::Vector{Tuple{Int, Int,Float64}}, remove_input_file::Bool)
    """
    Applies permutation to the input and writes it in the output
    permutation = [(a,b,c), ...] -> column a from input takes place b in output and is multiplied by c
    """
    # Open the input and output files
    open(input_file, "r") do infile
        open(output_file, "w") do outfile
            for line in eachline(infile)
                # Parse the input line into columns (assume space-separated values)
                columns = parse.(Float64, split(line, " "))
                # Create a new row with permuted columns
                permuted_row = zeros(Float64, length(columns))
                for (source_col, target_col,sign) in permutation
                    permuted_row[target_col] = columns[source_col]*sign
                end
                # Write the permuted row to the output file
                write(outfile, join(permuted_row, " ") * "\n")
            end
        end
    end
    if(remove_input_file)
        rm(input_file)
    end
end

#-----------------------------------------------------------------------------------------------------------------
# PROCESSING
# These functions implement motor models developped by S. DELIGNE to compute torques
# Models: friction less, basic model and optimised model
#-----------------------------------------------------------------------------------------------------------------

function to_torque_model(U::Vector{Float64}, q̇::Vector{Float64}):: Vector{Float64}
    """
    Takes voltage and velocity as inputs and returns the torque
    Uses the model τ = kt' U - (τc + Kv'q̇)
    """
    # Motor Caracteristics: S. Deligne
    HGR = 353.5           # Hip gear-ratio
    KGR = 212.6           # Knee gear-ratio
    ktp  = 0.395/HGR      # Torque constant with respect to the voltage [Nm/V] 
    Kvp  = 1.589/(HGR*HGR)      # Viscous friction constant [Nm*s/rad] (linked to motor speed)
    τc  = 0.065/HGR           # Dry friction torque [Nm]

    # Optimised motor model : τ = kt'*U - (τc + Kv'q̇) - C(q,q̇)
    ω = q̇[2:end] .* [HGR, HGR, KGR, KGR]
    τ_0 = U[2:end] .* [HGR, HGR, KGR, KGR] .* ktp  .- ω .* [HGR, HGR, KGR, KGR] .* Kvp
    #τ = ifelse.(τ_0 .> 0, max.(τ_0 .- τc, 0.0), min.(τ_0 .+ τc, 0.0)) # Static model
    τ = τ_0 .- sign.(ω) .* [HGR, HGR, KGR, KGR] .* τc
    
    return append!([U[1]],τ)
end

function compute_model(input_file_1::String, input_file_2::String, output_file::String, func::Function)
    """
    Takes two input files and writes the result of func in the output
    func: to_current_basic_model, to_torque_basic_model and to_torque_optimised_model
    """
    # Open all files
    open(input_file_1, "r") do f1
        open(input_file_2, "r") do f2
            open(output_file, "w") do out
                # Read lines from both input files simultaneously
                for (line1, line2) in zip(eachline(f1), eachline(f2))
                    # Parse floats from the lines
                    data1 = parse.(Float64, split(line1, " "))
                    data2 = parse.(Float64, split(line2, " "))
                    
                    # Apply the operation on the two arrays
                    result = func(data1, data2)
                    
                    # Write the result to the output file
                    write(out, join(result, " "), "\n")
                end
            end
        end
    end
end

#-----------------------------------------------------------------------------------------------------------------
# POST-PROCESSING
# These functions allow to extend the files (add padding for simulation) and to plot them
#-----------------------------------------------------------------------------------------------------------------

function extend_data(input_file::String, output_file::String, δt::Float64, ext_fact::Int64; max_lines::Union{Nothing, Int} = nothing)
    open(input_file, "r") do infile
        open(output_file, "w") do outfile
            total_lines_written = 0  # Counter for lines written to the output file

            for line in eachline(infile)
                # Stop if the total lines written exceeds max_lines
                if max_lines !== nothing && total_lines_written >= max_lines
                    break
                end
                # Parse the line into a vector of floats
                data = parse.(Float64, split(line, " "))
                # Extract the time value (first column)
                t = data[1]
                # Duplicate the line ext_fact times with adjusted time values
                for i in 0:(ext_fact - 1)
                    # Stop if the total lines written exceeds max_lines
                    if max_lines !== nothing && total_lines_written >= max_lines
                        break
                    end
                    # Compute the adjusted time
                    new_t = t + i * δt / ext_fact
                    # Write the new line to the output file
                    println(outfile, join(vcat([new_t], data[2:end]), " "))
                    # Increment the counter
                    total_lines_written += 1
                end
            end
        end
    end
end

function plot_data(file_path::String, Folder::String, title::String, time_range::Union{Nothing, Tuple{Float64, Float64}} = nothing)
    # Read the file line by line
    lines = readlines(file_path)

    # Parse the data into a matrix
    data = [parse.(Float64, split(line, " ")) for line in lines]

    # Convert the matrix of parsed data into separate columns
    time = [row[1] for row in data]    # First column is time
    data1 = [row[2] for row in data]  # Second column
    data2 = [row[3] for row in data]  # Third column
    data3 = [row[4] for row in data]  # Fourth column
    data4 = [row[5] for row in data]  # Fifth column

    # If time_range is provided, filter the data
    if time_range !== nothing
        # Extract the start and end of the time range
        start_time, end_time = time_range

        # Filter the data based on the time range
        indices = findall(x -> start_time <= x <= end_time, time)
        time = time[indices]
        data1 = data1[indices]
        data2 = data2[indices]
        data3 = data3[indices]
        data4 = data4[indices]
    end

    # Create the plot
    p1 = plot(time, data1, label="Data 1", xlabel="Time", ylabel="Values", lw=2, color="#1f77b4")
    plot!(p1, time, data2, label="Data 2", lw=2, color="#ff7f0e")
    # Add title
    title!(p1, title*" Hip")
    savefig(joinpath(Folder, title * "_Hip" * ".png"))  # Save as PNG
    # Create the plot
    p2 = plot(time, data3, label="Data 3", xlabel="Time", ylabel="Values", lw=2, color="#2ca02c")
    plot!(p2, time, data4, label="Data 4", lw=2,color="#d62728")
    # Add title
    title!(p2, title*" Knee")
    savefig(joinpath(Folder, title * "_Knee" * ".png"))  # Save as PNG

end

#-----------------------------------------------------------------------------------------------------------------
# OTHER: FOLDER SPECIFIC FUNCTIONS
#-----------------------------------------------------------------------------------------------------------------

function subsample_csv(input_file::String, subsampling_factor::Int, output_file::String)
    if subsampling_factor < 1
        error("Subsampling factor must be a positive integer.")
    end

    # Read the input CSV file into a DataFrame
    df = CSV.read(input_file, DataFrame)

    # Subsample the DataFrame
    subsampled_df = df[1:subsampling_factor:end, :]

    # Write the subsampled data to the output file
    CSV.write(output_file, subsampled_df)

    println("Subsampled data written to: $output_file")
end

function sum_files(file1_path::String, file2_path::String, output_path::String)
    # Open the two files for reading
    file1 = open(file1_path, "r")
    file2 = open(file2_path, "r")
    
    # Create an output file for writing
    output_file = open(output_path, "w")
    
    # Loop through the lines of both files
    while !eof(file1) && !eof(file2)
        # Read a line from each file
        line1 = readline(file1)
        line2 = readline(file2)
        
        # Split the lines into arrays of numbers
        values1 = parse.(Float64, split(line1))
        values2 = parse.(Float64, split(line2))
        
        # Assuming each line has the form: t D1 D2 0 0 and t 0 0 D3 D4
        t, D1, D2, _, _ = values1
        _, _, _, D3, D4 = values2
        
        # Sum the corresponding values
        sum_line = "$t $D1 $D2 $D3 $D4\n"
        
        # Write the summed line to the output file
        write(output_file, sum_line)
    end
    
    # Close the files
    close(file1)
    close(file2)
    close(output_file)
end