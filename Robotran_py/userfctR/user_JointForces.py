# -*- coding: utf-8 -*-
"""Module for the definition of joint forces."""
# Author: Robotran Team
# (c) Universite catholique de Louvain, 2020
import numpy as np

# TODO: commenter et clean / reformuler 
class trajectory:
    def __init__(self, tab):
        """
        Iitialize the trajectory structure.

        Parameters:
            tab : Table with the trajectory of the motors [t,qhl,qhr,qkl,qkr]
            i : An index (no specific purpose)
        """
        self.trajectory = tab
        self.i = 0
 
tab = np.loadtxt('../Walking_Patterns/ZMP.csv', skiprows=1, usecols=range(5), delimiter=',')
reference_trajectory = trajectory(tab)

def user_JointForces(mbs_data, tsim):
    """Compute the force and torques in the joint.

    It fills the MBsysPy.MbsData.Qq array.

    Parameters
    ----------
    mbs_data : MBsysPy.MbsData
        The multibody system associated to this computation.
    tsim : float
        The current time of the simulation.

    Notes
    -----
    The numpy.ndarray MBsysPy.MbsData.Qq is 1D array with index starting at 1.
    The first index (array[0]) must not be modified. The first index to be
    filled is array[1].

    Returns
    -------
    None
    """
    # Reset all forces
    mbs_data.Qq[1:] = 0.
    # Recover motor id, position and velocity
    motor_ids = np.array([mbs_data.joint_id["R_lh"],mbs_data.joint_id["R_rh"] ,mbs_data.joint_id["R_lk"],mbs_data.joint_id["R_rk"]])
    q_motors  = np.array([mbs_data.q[motor_ids[0]] , mbs_data.q[motor_ids[1]] ,mbs_data.q[motor_ids[2]] ,mbs_data.q[motor_ids[3]]])
    qd_motors = np.array([mbs_data.qd[motor_ids[0]], mbs_data.qd[motor_ids[1]],mbs_data.qd[motor_ids[2]],mbs_data.qd[motor_ids[3]]])
    # Find the reference postion for current time in the trajectory
    trajectory_index = int(tsim * mbs_data.frequency)
    q_ref = reference_trajectory.trajectory[trajectory_index][1:]
    
    ### Dynamixel Controller ###
    Kp = 900.0/128
    PWM_goal = 885.0
    Nominal_voltage = 12.0
    PWM = (q_ref - q_motors) * (4095.0 / (2 * np.pi) * Kp)
    PWM_sat = np.clip(PWM, -PWM_goal, PWM_goal)
    
    u = PWM_sat * (Nominal_voltage / PWM_goal)
    """   
    if reference_trajectory.i < 1000:
        reference_trajectory.i += 1
        if reference_trajectory.i%100 == 1:
            print(PWM)
            print(u)
    """
    mbs_data.Voltages = u
    ### Motor Equations ###
    HGR = 353.5             # Hip gear-ratio
    KGR = 212.6             # Knee gear-ratio
    ktp  = 0.395/HGR        # Torque constant with respect to the voltage [Nm/V] 
    Kvp  = 1.589/(HGR*HGR)  # Viscous friction constant [Nm*s/rad] (linked to motor speed)
    tauc_u  = 0.065/HGR       # Dry friction torque [Nm]
 
    gear_ratios = np.array([HGR,HGR,KGR,KGR])
    w = qd_motors * gear_ratios
    tau_0 = u * gear_ratios* ktp - w * gear_ratios * Kvp
    tau_m = tau_0 - np.sign(w) * gear_ratios * tauc_u
    for i in range(motor_ids.size):
        mbs_data.Qq[motor_ids[i]] = tau_m[i]
    return
