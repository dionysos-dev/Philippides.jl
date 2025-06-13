#include <stdio.h>
#include <stdlib.h>
#include "mbs_data.h"
#include "mbs_part.h"
#include "mbs_equil.h"
#include "mbs_modal.h"
#include "mbs_dirdyn.h"
#include "integrator.h"
#include "mbs_invdyn.h"
#include "mbs_solvekin.h"
#include "mbs_message.h"
#include "mbs_loader.h"
#include "mbs_set.h"
#include "structures.h"
#include "user_all_id.h"

// CSV parameters (not used)
const char* filename_CSV = "../../Walking_Patterns/WP.csv";
double FREQUENCY = 50.0;
int COLUMNS = 5; // time,q1_l,q1_r,q2_l,q2_r -> 5 columns

// Global variable to hold the MBS data structure
MbsData *mbs_data;

// Initialize function (only needs to be called once)
void init() {
    //mbs_msg("Starting philippides MBS project!\n");

    // Instantiate useful structures
    init_viscoelastic_coulomb(0.8, 2e4, 100.0);
    init_hunt_crossley_hertz(5e4, 1.5, 0.3);

    // Load the MBS model from the .mbs file
    //mbs_msg("Loading the philippides data file !\n");
    mbs_data = mbs_load("philippides_c.mbs");
    //mbs_msg("*.mbs file loaded!\n");

    // Setup coordinate partitioning
    MbsPart *mbs_part;
    mbs_data->process = 1;
    mbs_part = mbs_new_part(mbs_data);
    mbs_part->options->rowperm = 1;
    mbs_part->options->verbose = 0; // 1
    mbs_run_part(mbs_part, mbs_data);
    mbs_delete_part(mbs_part);
    init_voltages();
    init_trajectory();
    init_U_REF();
}

// Function to compute dynamics with given initial conditions
double philippides_results[16];
void philippides(double x[20]) {
    // Instantiate useful structures
    init_contact_manager(11);

    // Set the initial conditions
    mbs_data->q[Tx_boom_id] = x[0];
    mbs_data->q[Tz_boom_id] = x[1];
    mbs_data->q[R_lh_id] = x[2];
    mbs_data->q[R_rh_id] = x[3];
    mbs_data->q[R_lk_id] = x[4];
    mbs_data->q[R_rk_id] = x[5];
    mbs_data->q[R_lf_id] = x[6];
    mbs_data->q[R_rf_id] = x[7];

    // Set the velocities
    mbs_data->qd[Tx_boom_id] = x[8];
    mbs_data->qd[Tz_boom_id] = x[9];
    mbs_data->qd[R_lh_id] = x[10];
    mbs_data->qd[R_rh_id] = x[11];
    mbs_data->qd[R_lk_id] = x[12];
    mbs_data->qd[R_rk_id] = x[13];
    mbs_data->qd[R_lf_id] = x[14];
    mbs_data->qd[R_rf_id] = x[15];

    // Set Inputs
    U_REF[0] = x[16]; // LH
    U_REF[1] = x[17]; // RH
    U_REF[2] = x[18]; // LK 
    U_REF[3] = x[19]; // POSITION /!\ OF RK

    // Run direct dynamics (time integration)
    MbsDirdyn *mbs_dirdyn;
    mbs_data->process = 3;
    mbs_dirdyn = mbs_new_dirdyn(mbs_data);

    mbs_dirdyn->options->dt0 = 1e-3;
    mbs_dirdyn->options->tf = 1e-1;
    mbs_dirdyn->options->save2file = 0; // 1
    mbs_dirdyn->options->verbose = 0;
    
    mbs_run_dirdyn(mbs_dirdyn, mbs_data);
    mbs_delete_dirdyn(mbs_dirdyn, mbs_data);

    // Free structures
    free_contact_manager();

    // Store results (positions and velocities)
    philippides_results[0] = mbs_data->q[Tx_boom_id];
    philippides_results[1] = mbs_data->q[Tz_boom_id]; 
    philippides_results[2] = mbs_data->q[R_lh_id];     
    philippides_results[3] = mbs_data->q[R_rh_id];  
    philippides_results[4] = mbs_data->q[R_lk_id];  
    philippides_results[5] = mbs_data->q[R_rk_id];
    philippides_results[6] = mbs_data->q[R_lf_id];
    philippides_results[7] = mbs_data->q[R_rf_id];

    philippides_results[8] = mbs_data->qd[Tx_boom_id];
    philippides_results[9] = mbs_data->qd[Tz_boom_id]; 
    philippides_results[10] = mbs_data->qd[R_lh_id];     
    philippides_results[11] = mbs_data->qd[R_rh_id];  
    philippides_results[12] = mbs_data->qd[R_lk_id];  
    philippides_results[13] = mbs_data->qd[R_rk_id];
    philippides_results[14] = mbs_data->qd[R_lf_id];
    philippides_results[15] = mbs_data->qd[R_rf_id]; 
}

// Function to get the results from the philippides function
void get_philippides_results(double *results) {
    for (int i = 0; i < 16; i++) {
        results[i] = philippides_results[i];
    }
}

// Function to free resources when done
void free_resources() {
    mbs_delete_data(mbs_data);
    free_viscoelastic_coulomb();
    free_hunt_crossley_hertz();
    free_voltages();
    free_trajectory();
    free_U_REF();
}

// Main function (not used)
int main(int argc, char const *argv[]) {
    // Not used
    return 0;
}
