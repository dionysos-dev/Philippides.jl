/* ---------------------------
 * Robotran - MBsysC
 * 
 * Template file for direct dynamics module
 * 
 * This file enables the user to implement custom user constraints
 * while using accelred in direct dynamics. It is a template
 * file that can be edited by the user.
 * 
 * (c) Robotran Team
 *     
 */

#include "mbs_data.h"

int user_cons_J_accelred(MbsData* s, double tsim)
{
    double** Jac = s->jac_user;

    /*-- Begin of user code --*/

    // double NRh2 = 1.0;
    // double NR_ERR = 1.0e-20;

    //// Using Newton-Raphson procedure (example)
    // while ((NRh2 > NR_ERR) && (NR_iter++ < 30))
    // {

       // // closing the loops

        // NRh2 = ...
       
    // }
    // // if more than 30 steps have been taken to satisfy the constraints
    // if (NR_iter > 30)
    // {
        // printf("err in user_cons_J_accelred \n");
        // return -1;
    // }
    
    // Jac[1][1] = ...
    // Jac[1][2] = 0;
    // Jac[1][3] = 0;
    // Jac[1][4] = 0;
    // Jac[1][5] = 0;
    // Jac[1][6] = ;
    // Jac[1][7] = ... ;
    // Jac[2][1] = ... ;
    // ...

    return 0;
    /*-- End of user code --*/

}
