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


number_of_files = 4

if(File == "Position" or File == "Velocity"):
    columns_Robotran = {'Time': 0, 'Tx_boom': 1, 'Tz_boom': 2, 'R_hip': 3, 'T_lh_fixed': 4, 
           'R_lh': 5, 'T_lk_fixed': 6, 'R_lk': 7, 'R_lf': 8, 'T_rh_fixed': 9, 
           'R_rh': 10, 'T_rk_fixed': 11, 'R_rk': 12, 'R_rf': 13}
else:
    columns_Robotran = {'Time': 0, 'R_lh': 1, 'R_rh': 2, 'R_lk': 3, 'R_rk': 4}
columns_index_Robotran = [columns_Robotran['R_lh'], columns_Robotran['R_rh'], columns_Robotran['R_lk'], columns_Robotran['R_rk']]

Simulation_file = "../Robotran_py/data/Robotran/" + "trajectory" + "/" + "dirdyn_q.res"
WP = "../Robotran_py/Walking_Patterns/WP_Dionysos_Robotran_trajectory.csv"

Simulation_file_D = "../JuliaRobotics/data/Dionysos/" + "trajectory" + "/" + File + ".txt"
WP_D = "../JuliaRobotics/WalkingPatterns/Dionysos_JR_trajectory.csv"


save_folder = "Robotran_traj/"

#----------------------------------------------------------------------------
#                       Load DATA
#----------------------------------------------------------------------------

data_simu = np.loadtxt(Simulation_file)
data_WP = np.loadtxt(WP, delimiter=',', skiprows=1)

data_simu_D = np.loadtxt(Simulation_file_D)
data_WP_D = np.loadtxt(WP_D, delimiter=',', skiprows=1)

#----------------------------------------------------------------------------
#                       Make PLOTS
#----------------------------------------------------------------------------

for i in range(len(columns_names)-1):
    plt.figure()

    # Plot parameters
    plt.xlabel('time [s]')
    plt.ylabel(File + " " + y_units)
    #plt.title(columns_names[i+1] + " " + File)
    plt.xlim(0, 11.6) # 5s plots

    if(i < 2):
        plt.ylim(-11.5, 15)
    else:
        plt.ylim(-8, 14)

    # Plot data
    """
    plt.axvline(x=0.7, color = "purple", alpha = 0.33)
    plt.axvline(x=2.4, color = "purple", alpha = 0.33)
    plt.axvline(x=4.8, color = "purple", alpha = 0.33)
    """
    plt.plot(data_WP[:,0], data_WP[:,i+1]*(180/math.pi), label = 'Robotran reference trajectory', linestyle = "--", color='tab:orange', linewidth=1.5, alpha = 0.65)
    plt.plot(data_simu[::100,0], data_simu[::100,columns_index_Robotran[i]]*(180/math.pi), label = 'Robotran validation on $S_0$', color='tab:orange', linewidth=1.5)
    plt.plot(data_WP_D[:,0], data_WP_D[:,i+1]*(180/math.pi), label = 'Julia reference trajectory', linestyle = "--", color='tab:green', linewidth=1.5, alpha = 0.65)
    plt.plot(data_simu_D[:,0], data_simu_D[:,i+1]*(180/math.pi), label = 'Julia validation on $S_0$', color='tab:green', linewidth=1.5)

    if(i < 2):
        plt.legend(loc = "upper left")
    else:
        plt.legend(loc = "lower left")
    if(verbose):
        plt.show()
    else:
        filename_save = save_folder + File + " " + columns_names[i+1] + '.pdf'
        plt.savefig(filename_save)
    
    name = columns_names[i+1]
    mean_error = np.mean(data_WP[:,i+1] - data_simu[::100,columns_index_Robotran[i]])*(180/math.pi)
    RMSE = np.sqrt( np.mean( ((data_WP[:,i+1] - data_simu[::100,columns_index_Robotran[i]])*(180/math.pi))**2) )
    print("Mean error on ", name, ": ", mean_error)
    print("RMSE on ", name, ": ", RMSE)
    print()