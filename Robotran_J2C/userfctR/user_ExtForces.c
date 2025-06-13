/* ---------------------------
 * Robotran - MBsysC
 * 
 * Template file for the external forces/torques function
 * 
 * (c) Universite catholique de Louvain
 *     
 */

#include "math.h"
#include <stdio.h>
#include "mbs_data.h"
#include "mbs_project_interface.h"

#include "set_output.h"
#include "structures.h"

// -------------------------------------------------------
//      CONTACT MANAGER
//      /!\ compute_penetrations: different from python,
//          *deltas and *deltas_dot are passed as arguments
// -------------------------------------------------------

// Function to update the contact with a new sensor position
void update_contact(Contact_Manager *cm, int ixF, double PxF[4]) {
    if (isnan(cm->Previous_PxF[ixF][0])) {
        // Initial position
        for (int i = 0; i < 4; i++) {
            cm->Previous_PxF[ixF][i] = PxF[i];
        }
        
        if (PxF[3] <= 0) {
            for (int i = 0; i < 4; i++) {
                cm->Contact_PxF[ixF][i] = PxF[i];
            }
            cm->Contact_PxF[ixF][3] = 0.0; // Set z = 0
            cm->InContact[ixF] = 1;
        }
    } else if (PxF[3] <= 0) {
        // Penetration detection
        if (cm->InContact[ixF] == 0) {
            double ratio = cm->Previous_PxF[ixF][3] / (cm->Previous_PxF[ixF][3] - PxF[3]);
            for (int i = 0; i < 4; i++) {
                cm->Contact_PxF[ixF][i] = cm->Previous_PxF[ixF][i] + ratio * (PxF[i] - cm->Previous_PxF[ixF][i]);
            }
            cm->Contact_PxF[ixF][3] = 0.0; // Ensure z = 0
            cm->InContact[ixF] = 1;
        }
    } else {
        cm->InContact[ixF] = 0;
    }
    
    // Update the previous position
    for (int i = 0; i < 4; i++) {
        cm->Previous_PxF[ixF][i] = PxF[i];
    }
}

// Function to update slip contact
void update_slip_contact(Contact_Manager *cm, int ixF, double PxF[4], double delta_slip_x, double delta_slip_y, int slip) {
    if (slip == 1 && cm->InContact[ixF] == 1) {
        cm->Contact_PxF[ixF][1] = PxF[1] + delta_slip_x;
        cm->Contact_PxF[ixF][2] = PxF[2] + delta_slip_y;
    }
}

// Function to compute penetrations
///!!!\\\ different from python, *deltas and *deltas_dot are passed as arguments
void compute_penetrations(Contact_Manager *cm, int ixF, double PxF[4], double VxF[4], double *deltas, double *deltas_dot) {
    for (int i = 0; i < 3; i++) {
        deltas[i] = 0.0;
        deltas_dot[i] = 0.0;
    }
    
    if (cm->InContact[ixF] == 1) {
        for (int i = 0; i < 3; i++) {
            deltas[i] = PxF[i+1] - cm->Contact_PxF[ixF][i+1];
        }
        for (int i = 0; i < 3; i++) {
            deltas_dot[i] = VxF[i+1];
        }
    }
}

// Function to compute the normal force using the Hunt-Crossley Hertz model
double compute_normal_force_hc(HuntCrossleyHertz *model, double delta, double delta_dot) {
    double Fz = 0.0;
    if (delta < 0) {
        Fz = model->k * pow(fabs(delta), model->n) * (1 - model->d * delta_dot);
    }
    return Fz;
}

// Function to compute tangential force using the Viscoelastic Coulomb model
void compute_tangential_force(ViscoelasticCoulombModel *model, double F_n, double delta_x, double delta_x_dot,
     double delta_y, double delta_y_dot, double *Fx, double *Fy, double *delta_no_slip_x, double *delta_no_slip_y, 
     int *slip) {
    // Compute raw viscoelastic forces
    *Fx = -model->k * delta_x - model->d * delta_x_dot;
    *Fy = -model->k * delta_y - model->d * delta_y_dot;

    // Compute total force magnitude
    double F_total = sqrt(*Fx * *Fx + *Fy * *Fy);
    // Maximum Coulomb force
    double F_max = model->mu * F_n;

    // Initialize (assuming no slip at first)
    *delta_no_slip_x = 0;
    *delta_no_slip_y = 0;
    *slip = 0;

    // Apply saturation (if slip occurs)
    if (F_total > F_max) {
        double scaling_factor = F_max / F_total;
        *Fx *= scaling_factor;
        *Fy *= scaling_factor;
        *delta_no_slip_x = -( *Fx + model->d * delta_x_dot ) / model->k;
        *delta_no_slip_y = -( *Fy + model->d * delta_y_dot ) / model->k;
        *slip = 1;
    }
}

// Function to compute normal force using the Viscoelastic Coulomb model
double compute_normal_force_viscoelastic(ViscoelasticCoulombModel *model, double delta, double delta_dot) {
    double Fz = 0.0;
    if (delta <= 0) {
        Fz = -model->k * delta - model->d * delta_dot;
    }
    return Fz;
}

double* user_ExtForces(double PxF[4], double RxF[4][4], 
    double VxF[4], double OMxF[4], 
    double AxF[4], double OMPxF[4], 
    MbsData *mbs_data, double tsim, int ixF)
{

    double *Fx, *Fy; double Fz=0.0;
    double Mx=0.0, My=0.0, Mz=0.0;
    double dxF[4] ={0.0, 0.0, 0.0, 0.0};

    Fx = (double*) malloc(sizeof(double));
    Fy = (double*) malloc(sizeof(double));

    double *SWr = mbs_data->SWr[ixF];

    // default application point of the force: anchor point to which it is attached
    int idpt = 0;
    idpt = mbs_data->xfidpt[ixF];
    dxF[1] = mbs_data->dpt[1][idpt];
    dxF[2] = mbs_data->dpt[2][idpt];
    dxF[3] = mbs_data->dpt[3][idpt];

    // Begin user-defined force calculation
    // Update contact information (function would need to be implemented as part of cm or similar)
    update_contact(cm, ixF, PxF);

    // Compute penetrations and velocities (function would need to be implemented)
    double deltas[4], deltas_dot[4];
    compute_penetrations(cm, ixF, PxF, VxF, deltas, deltas_dot);

    // Compute normal force based on the chosen model (Hunt-Crossley-Hertz or Viscoelastic Coulomb)
    int Model = 0;
    if (Model == 0) {
        Fz = compute_normal_force_hc(hc_model,deltas[2], deltas_dot[2]);
    }
    else if (Model == 1) {
        Fz = compute_normal_force_viscoelastic(vc_model, deltas[2], deltas_dot[2]);
    }

    // Compute tangential forces
    double *delta_no_slip_x, *delta_no_slip_y;
    int *slip;
    delta_no_slip_x = (double*) malloc(sizeof(double));
    delta_no_slip_y = (double*) malloc(sizeof(double));
    slip = (int*) malloc(sizeof(int));

    compute_tangential_force(vc_model,Fz, deltas[0], deltas_dot[0], deltas[1], deltas_dot[1],
        Fx, Fy, delta_no_slip_x, delta_no_slip_y, slip
    );

    // Update slip contact data
    update_slip_contact(cm, ixF, PxF, *delta_no_slip_x, *delta_no_slip_y, *slip);

    // Concatenate force, torque, and force application point to the returned array.
    SWr[1] = *Fx;
    SWr[2] = *Fy;
    SWr[3] = Fz;
    SWr[4] = Mx;
    SWr[5] = My;
    SWr[6] = Mz;
    SWr[7] = dxF[1];
    SWr[8] = dxF[2];
    SWr[9] = dxF[3];

    // Store results in mbs_data.
    cm->results[4 * ixF] = *Fx;
    cm->results[4 * ixF + 1] = *Fy;
    cm->results[4 * ixF + 2] = Fz;
    cm->results[4 * ixF + 3] = ixF;

    // Free temporary variables
    free(Fx); free(Fy); free(delta_no_slip_x); free(delta_no_slip_y); free(slip);

    return SWr;
}