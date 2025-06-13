   /**
    *
    *   Universite catholique de Louvain
    *   Mechatronic, Electrical Energy, and Dynamic systems (iMMC/MEED) 
    *   https://www.robotran.be
    *   Contact : info@robotran.be
    *
    *
    *   MBsysC main script template for complete model:
    *   -----------------------------------------------
    *    This template loads the data file *.mbs and executes:
    *      - the coordinate partitioning module
    *      - the equilibrium module
    *      - the modal module
    *      - the direct dynamic module (time integration of
    *        equations of motion).
    *      - the inverse kinematics module
    *      - the inverse dynamics module
    *    It may be adapted and completed by the user.
    *
    *    (c) Universite catholique de Louvain
    */

#include <stdio.h>
#include <time.h>

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

// CSV parameters
const char* filename_CSV = "../../Walking_Patterns/ZMP.csv";
double FREQUENCY = 50.0;
int COLUMNS = 5; // time,q1_l,q1_r,q2_l,q2_r -> 5 columns

int main(int argc, char const *argv[])
{
    //mbs_msg("Starting philippides MBS project!\n");
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     *                     LOADING                               *
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    MbsData *mbs_data;

    //mbs_msg("Loading the philippides data file !\n");
    mbs_data = mbs_load("philippides_c.mbs");
    //mbs_msg("*.mbs file loaded!\n");

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     *            Structures Initialisations                     *
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    init_voltages();
    init_trajectory();
    init_contact_manager(11);
    init_viscoelastic_coulomb(0.8, 2e4, 100.0);
    init_hunt_crossley_hertz(5e4, 1.5, 0.3);
 
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     *              COORDINATE PARTITIONING                      *
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    MbsPart *mbs_part;
    mbs_data->process = 1;

    mbs_part = mbs_new_part(mbs_data);

    mbs_part->options->rowperm=1;
    mbs_part->options->verbose = 0;

    mbs_run_part(mbs_part, mbs_data);

    mbs_delete_part(mbs_part);
    
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     *                   DIRECT DYNAMICS                         *
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    MbsDirdyn *mbs_dirdyn;
    mbs_data->process = 3;

    mbs_dirdyn = mbs_new_dirdyn(mbs_data);

    // dirdyn options (see documentations for additional options)
    mbs_dirdyn->options->dt0 = 1e-3;
    mbs_dirdyn->options->tf  = 20.0;
    mbs_dirdyn->options->verbose = 0;
    mbs_dirdyn->options->save2file = 1;
    //mbs_dirdyn->options->realtime = 1;
    clock_t start = clock();
    mbs_run_dirdyn(mbs_dirdyn, mbs_data);
    clock_t end = clock();
    double time_taken = (double) (end-start) / CLOCKS_PER_SEC;
    printf("Function took %.3f seconds to execute \n", time_taken);

    mbs_delete_dirdyn(mbs_dirdyn, mbs_data);
    
    
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     *                   CLOSING OPERATIONS                      *
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    mbs_delete_data(mbs_data);

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     *              Release Structure Memory                     *
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    free_voltages();
    free_trajectory();
    free_contact_manager();
    free_viscoelastic_coulomb();
    free_hunt_crossley_hertz();
}

