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
if(File == "Position"):
    columns_names = ["Left Hip", "Right Hip", "Left Knee", "Right Knee"]
else:
    columns_names = ["Left Hip", "Right Hip", "Left Knee"]


number_of_files = 2

Simulation_S0 = "../JuliaRobotics/data/Dionysos/" + "controller" + "/" + File + "_S0" ".txt"
Simulation_S1 = "../JuliaRobotics/data/Dionysos/" + "controller" + "/" + File + "_S1" ".txt"

save_folder = "JuliaRobotics_controller/"

#----------------------------------------------------------------------------
#                       Load DATA
#----------------------------------------------------------------------------

data_S0 = np.loadtxt(Simulation_S0)
data_S1 = np.loadtxt(Simulation_S1)

#----------------------------------------------------------------------------
#                       Make PLOTS
#----------------------------------------------------------------------------

for i in range(len(columns_names)):
    plt.figure()

    # Plot parameters
    plt.xlabel('time [s]')
    plt.ylabel(File + " " + y_units)
    #plt.title(columns_names[i+1] + " " + File)
    plt.xlim(0, 2.5) # 5s plots

    # Plot data
    if(File == "Position"):
        time = np.arange(0.0, 2.4024, 0.0001)
        mult = 180/math.pi
        index = i+2
        plt.plot(time, data_S1[:,index]*mult, label = 'Closed-loop on $S_0$', color='tab:green', linewidth=1.5)
        plt.plot(time, data_S0[:,index]*mult, label = 'Closed-loop on $S_1$', linestyle = "--", color='black', linewidth=1.5, alpha = 0.65)
    else:
        time = np.arange(0.0, 2.5, 0.1)
        mult = 1.0
        index = i
        plt.step(time, np.concatenate(([0], data_S1[:,index])), label = 'Closed-loop on $S_0$', color='tab:green', linewidth=1.5)
        plt.step(time, np.concatenate(([0], data_S0[:,index])), label = 'Closed-loop on $S_1$', linestyle = "--", color='black', linewidth=1.5, alpha = 0.65)
    plt.legend()
            
    
    if(verbose):
        plt.show()
    else:
        filename_save = save_folder + File + " " + columns_names[i] + '.pdf'
        plt.savefig(filename_save)