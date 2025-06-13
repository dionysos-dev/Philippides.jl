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

number_of_files = 4

Simulation_file_p = "../JuliaRobotics/data/" + "Prismatic" + "/" + File + ".txt"
Simulation_file_h = "../JuliaRobotics/data/" + "Hybrid" + "/" + File + ".txt"
if(File == "Torque"):
    Robot_file = "../Robot_data/Robot_200Hz/Outputs/" + File + ".txt"
else:
    Robot_file = "../Robot_data/Robot_200Hz/Inputs/" + File + ".txt"
WP = "../JuliaRobotics/WalkingPatterns/ZMP.csv"

save_folder = "JuliaRobotics/"

#----------------------------------------------------------------------------
#                       Load DATA
#----------------------------------------------------------------------------

data_simu_p = np.loadtxt(Simulation_file_p)
data_simu_h = np.loadtxt(Simulation_file_h)
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
    
    if(i < 2):
        if(File == "Position"):
            plt.ylim(-45, 5)
        elif(File == "Velocity"):
            plt.ylim(-100, 80)
        else:
            plt.ylim(-9, 8)
    if(i > 1):
        if(File == "Position"):
            plt.ylim(-5, 70)
        elif(File == "Velocity"):
            plt.ylim(-100, 125)
        else:
            plt.ylim(-6, 6)
        
    
    if(i == 2):
        if(File == "Position"):
            plt.axvline(x=2.1, color="purple", alpha = 0.33)
        elif(File == "Velocity"):
            plt.axvline(x=2.2, color="blue", alpha = 0.33)
            plt.axvline(x=3.0, color="purple", alpha = 0.33)
            plt.axvline(x=3.6, color="purple", alpha = 0.33)
        else:
            plt.axvline(x=2.0, color="purple", alpha = 0.33)
    
    if(File == "Position" or File == "Velocity"):
        mult = 180/math.pi
    else:
        mult = 1.0
            
    if(File != "Velocity"):
        plt.plot(data_robot[:,0], data_robot[:,i+1] * mult, label = 'Robot', color='black', linestyle = "--", linewidth=1.5, alpha = 0.65)
    plt.plot(data_simu_p[:,0], data_simu_p[:,i+1]* mult, label = 'Simulation prismatic', color='tab:green', linestyle='-', linewidth=1.5)
    plt.plot(data_simu_h[:,0], data_simu_h[:,columns_hybrid[i]] * (-1.0) * mult, label = 'Simulation hybrid', color='tab:orange', linestyle='-.', linewidth=1.5)
    if(File == "Velocity"):
        plt.plot(data_robot[:,0], data_robot[:,i+1] * mult, label = 'Robot', color='black', linestyle = "--", linewidth=1.5, alpha = 0.65)
    if(File == "Position"):
        plt.plot(data_WP[:,0], data_WP[:,i+1] * mult, label = 'WalkingPattern (q_ref)', color='tab:red', linestyle=':', linewidth=1.5)

    if(i == 3 and File == "Velocity"):
        plt.legend(loc="lower left")
    else:
        plt.legend()
    if(verbose):
        plt.show()
    else:
        filename_save = save_folder + File + " " + columns_names[i+1] + '.pdf'
        plt.savefig(filename_save)