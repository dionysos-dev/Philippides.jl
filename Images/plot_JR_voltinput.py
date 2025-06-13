import matplotlib.pyplot as plt
import numpy as np
import math as math

# this is file is only valid when the hybrid has been changed such that it moves in the same direction as the real robot
# -> cfr line 278 in RobotSimulator.jl

verbose = True
#----------------------------------------------------------------------------
#                       Code PARAMETERS
#----------------------------------------------------------------------------
File = "Position"
y_units = "[°]"

#----------------------------------------------------------------------------
#                       Folder DETAILS
#----------------------------------------------------------------------------
columns_names = ["", "Left Hip", "Right Hip", "Left Knee", "Right Knee"]
columns_hybrid = [2, 1, 4, 3]

number_of_files = 3

Simulation_file_p = "../JuliaRobotics/data/" + "voltage-input" + "/" + File + ".txt"
if(File == "Torque"):
    Robot_file = "../Robot_data/Robot_200Hz/Outputs/" + File + ".txt"
else:
    Robot_file = "../Robot_data/Robot_200Hz/Inputs/" + File + ".txt"
WP = "../JuliaRobotics/WalkingPatterns/ZMP.csv"

save_folder = "JuliaRobotics/voltage-input/"

#----------------------------------------------------------------------------
#                       Load DATA
#----------------------------------------------------------------------------

data_simu_p = np.loadtxt(Simulation_file_p)
data_robot = np.loadtxt(Robot_file)
data_WP = np.loadtxt(WP, delimiter=',', skiprows=1)

#----------------------------------------------------------------------------
#                       Make PLOTS
#----------------------------------------------------------------------------

for i in range(len(columns_names)-1):
    plt.figure()

    # Plot parameters
    plt.xlabel('time [s]')
    plt.ylabel(File + " " + y_units)
    #plt.title(columns_names[i+1] + " " + File)
    plt.xlim(0, 5) # 5s plots
    
    plt.axvline(x = 2.0, color = "purple", alpha = 0.33)
    
    if(i < 2):
        plt.ylim(-45,10)
    elif(i == 2):
        plt.ylim(-5, 100)
    else:
        plt.ylim(-90,75)
        
    mult = 180/math.pi
    # Plot data
    plt.plot(data_robot[:,0], data_robot[:,i+1] * mult, label = 'Robot', color='black', linewidth=1.5, linestyle = "--", alpha = 0.65)
    plt.plot(data_simu_p[:,0], data_simu_p[:,i+1] * mult, label = 'Simulation prismatic', color='tab:green', linestyle='-', linewidth=1.5)
    plt.plot(data_WP[:,0], data_WP[:,i+1] * mult, label = 'WalkingPattern (q_ref)', color='tab:red', linestyle=':', linewidth=1.5)
    
    if(i == 1):
        plt.legend(loc = "upper right")
    else:
        plt.legend()
    
    if(verbose):
        plt.show()
    else:
        filename_save = save_folder + File + " " + columns_names[i+1] + '.pdf'
        plt.savefig(filename_save)