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

simu_R_S0 = "../Robotran_py/data/Robotran/controller/Position_S0.txt"
simu_R_S1 = "../Robotran_py/data/Robotran/controller/Position_S1.txt"
Simulation_S0 = "../JuliaRobotics/data/Dionysos/controller/Position_S0.txt"
Simulation_S1 = "../JuliaRobotics/data/Dionysos/controller/Position_S1.txt"

save_folder = "Robotran_controller/"

#----------------------------------------------------------------------------
#                       Load DATA
#----------------------------------------------------------------------------

data_simu_R_S0 = np.loadtxt(simu_R_S0)
data_simu_R_S1 = np.loadtxt(simu_R_S1)
data_simu_J_S0 = np.loadtxt(Simulation_S0)
data_simu_J_S1 = np.loadtxt(Simulation_S1)

#----------------------------------------------------------------------------
#                       Make PLOTS
#----------------------------------------------------------------------------

for i in range(len(columns_names)-1):
    plt.figure()

    # Plot parameters
    plt.xlabel('time [s]')
    plt.ylabel(File + " " + y_units)
    #plt.title(columns_names[i+1] + " " + File)
    plt.xlim(0, 2.5) # 5s plots

    # Plot data
    """
    plt.axvline(x=0.7, color = "purple", alpha = 0.33)
    plt.axvline(x=2.4, color = "purple", alpha = 0.33)
    plt.axvline(x=4.8, color = "purple", alpha = 0.33)
    """
    time = np.arange(0.0, 2.4024, 0.01)
    mult = 180/math.pi
    index = i+2
    plt.plot(time, data_simu_J_S0[::100,index]*mult, label = 'Julia Closed-loop on $S_0$', color='tab:green', linewidth=1.5)
    plt.plot(time, data_simu_J_S1[::100,index]*mult, label = 'Julia Closed-loop on $S_1$', linestyle = "--", color='tab:green', linewidth=1.5, alpha = 0.65)

    time = np.arange(0.0, 2.0, 0.1)
    plt.plot(time, data_simu_R_S0[:,index]*mult, label = 'Robotran Closed-loop on $S_0$', color='tab:blue', linewidth=1.5)
    time = np.arange(0.0, 1.7, 0.1)
    plt.plot(time, data_simu_R_S1[:,index]*mult, label = 'Robotran Closed-loop on $S_1$', linestyle = "--", color='tab:blue', linewidth=1.5, alpha = 0.65)

    if(i == 2):
        plt.legend(loc = "upper right")
        plt.ylim(-2,14)
    else:
        plt.legend()
    if(verbose):
        plt.show()
    else:
        filename_save = save_folder + File + " " + columns_names[i+1] + '.pdf'
        plt.savefig(filename_save)