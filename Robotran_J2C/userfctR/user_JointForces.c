/* ---------------------------
 * Robotran - MBsysC
 * 
 * Template file for the user JointForces function
 * 
 * (c) Universite catholique de Louvain
 *     
 */

#include <stdio.h>
#include <assert.h>
#include "math.h" 

#include "mbs_data.h"
#include "user_model.h"
#include "set_output.h"
#include "useful_functions.h"
#include "user_all_id.h"

#include "structures.h"

// Function to compute joint forces based on motor states and reference trajectory
double* user_JointForces(MbsData *mbs_data, double tsim) {
    // Reset all forces in the joint force array
    zeros_dvec_1(mbs_data->Qq);
    // Retrieve motor IDs for different joints
    int motor_ids[4] = {R_lh_id, R_rh_id, R_lk_id, R_rk_id};
    // Retrieve motor positions and velocities
    double q_motors[4], qd_motors[4];
    for (int i = 0; i < 4; i++) {
        q_motors[i] = mbs_data->q[motor_ids[i]];
        qd_motors[i] = mbs_data->qd[motor_ids[i]];
    }
    
    // Get reference joint positions for the current time step
    double q_ref_RK = U_REF[3];
    // Controller parameters
    double Kp = 900.0 / 128.0;
    double PWM_goal = 885.0;
    double Nominal_voltage = 12.0;
    // Compute PWM signals for motor control
    double PWM, PWM_sat, u[4];
    PWM = (q_ref_RK - q_motors[3]) * (4095.0 / (2 * M_PI) * Kp);
    PWM_sat = fmax(-PWM_goal, fmin(PWM, PWM_goal)); // Clipping function
    u[3] = PWM_sat * (Nominal_voltage / PWM_goal);

    for (int i = 0; i < 3; i++) {
        u[i] = U_REF[i];
        //voltages[i] = u[i];
    }
    
    // Motor parameters
    double HGR = 353.5;   // Hip gear ratio
    double KGR = 212.6;   // Knee gear ratio
    double ktp = 0.395 / HGR;   // Torque constant w.r.t. voltage [Nm/V]
    double Kvp = 1.589 / (HGR * HGR); // Viscous friction constant [Nm*s/rad]
    double tauc_u = 0.065 / HGR;  // Dry friction torque [Nm]
    
    // Compute motor torques
    double gear_ratios[4] = {HGR, HGR, KGR, KGR};
    double w[4], tau_0[4], tau_m[4];
    for (int i = 0; i < 4; i++) {
        w[i] = qd_motors[i] * gear_ratios[i];
        tau_0[i] = u[i] * gear_ratios[i] * ktp - w[i] * gear_ratios[i] * Kvp;
        tau_m[i] = tau_0[i] - ((w[i] > 0) ? tauc_u * gear_ratios[i] : -tauc_u * gear_ratios[i]);
    }
    
    // Assign computed torques to joint force array
    for (int i = 0; i < 4; i++) {
        mbs_data->Qq[motor_ids[i]] = tau_m[i];
    }
    return mbs_data->Qq;
}
