import matplotlib.pyplot as plt
import numpy as np
import math as math

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


number_of_files = 2

Simulation_file = "../JuliaRobotics/data/Dionysos/" + "trajectory" + "/" + File + ".txt"
WP = "../JuliaRobotics/WalkingPatterns/Dionysos_JR_trajectory.csv"

save_folder = "JuliaRobotics_traj/"

#----------------------------------------------------------------------------
#                       Load DATA
#----------------------------------------------------------------------------

data_simu = np.loadtxt(Simulation_file)
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
    plt.xlim(0, 10.0) # 5s plots

    # Plot data
    plt.axvline(x=0.7, color = "purple", alpha = 0.33)
    plt.axvline(x=2.4, color = "purple", alpha = 0.33)
    plt.axvline(x=4.8, color = "purple", alpha = 0.33)
    plt.plot(data_WP[:,0], data_WP[:,i+1]*(180/math.pi), label = 'Reference trajectory', linestyle = "--", color='black', linewidth=1.5, alpha = 0.65)
    plt.plot(data_simu[:,0], data_simu[:,i+1]*(180/math.pi), label = 'Validation on $S_0$', color='tab:green', linewidth=1.5)
    plt.legend()
    if(verbose):
        plt.show()
    else:
        filename_save = save_folder + File + " " + columns_names[i+1] + '.pdf'
        plt.savefig(filename_save)
        
    name = columns_names[i+1]
    mean_error = np.mean(data_WP[:,i+1] - data_simu[:,i+1])*(180/math.pi)
    RMSE = np.sqrt( np.mean( ((data_WP[:,i+1] - data_simu[:,i+1])*(180/math.pi))**2) )
    print("Mean error on ", name, ": ", mean_error)
    print("RMSE on ", name, ": ", RMSE)
    print()