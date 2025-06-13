/* ---------------------------
 * Robotran - MBsysC
 * 
 * Template file for the user Link3D Forces function
 * 
 * (c) Universite catholique de Louvain
 *     
 */

#include "math.h"

#include "mbs_data.h"


double* user_Link3DForces(double PxF[4], double RxF[4][4], 
                          double VxF[4], double OMxF[4], 
                          double AxF[4], double OMPxF[4], 
                          MbsData *mbs_data, double tsim,int ixF)
{
    double Fx=0.0, Fy=0.0, Fz=0.0;
    double Mx=0.0, My=0.0, Mz=0.0;

    double *SWr = mbs_data->l3DWr[ixF];

/* Begin of user declaration */
    
    

/* End of user declaration */


    switch(ixF){

/* Begin of user code */
        case 1: 

                    
        break;

    /*  case 2: 
    
     
        break;
     */
        
/* End of user code */

    }
    SWr[1]=Fx;
    SWr[2]=Fy;
    SWr[3]=Fz;
    SWr[4]=Mx;
    SWr[5]=My;
    SWr[6]=Mz;

    return SWr;
}

