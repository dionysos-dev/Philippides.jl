/** ---------------------------
  * Robotran - MBsysC
  * 
  * Template file for equilibrium module
  * 
  * This files enable the user to call custom at
  * specific places in the time simulation. It is a template
  * file that can be edited by the user.
  * 
  * (c) Universite catholique de Louvain
  *     
  */

#include "math.h"

#include "mbs_data.h"
#include "mbs_equil_struct.h"

/*! \brief user own initialization functions
 *
 * \param[in,out] mbs_data data structure of the model
 * \param[in,out] mbs_equil general structure of the equilibrium module (for advance users)
 *
 * For beginners, it is advised to only use the MbsData structure.
 * The field MbsEquil is provided for more advance users.
 */
void user_equil_init(MbsData *mbs_data, MbsEquil *mbs_equil)
{

}

/*! \brief user own loop functions
 *
 * \param[in,out] mbs_data data structure of the model
 * \param[in,out] mbs_equil general structure of the equilibrium module (for advance users)
 *
 * For beginners, it is advised to only use the MbsData structure.
 * The field MbsEquil is provided for more advance users.
 */
void user_equil_loop(MbsData *mbs_data, MbsEquil *mbs_equil)
{

}

/*! \brief user own finishing functions
 *
 * \param[in,out] mbs_data data structure of the model
 * \param[in,out] mbs_equil general structure of the equilibrium module (for advance users)
 *
 * For beginners, it is advised to only use the MbsData structure.
 * The field MbsEquil is provided for more advance users.
 */
void user_equil_finish(MbsData *mbs_data, MbsEquil *mbs_equil)
{

}

/*! \brief user own implementation of added equilibrium equations Fxe
* Necessary to express equilibrium f(x)=0
* 
* \param[in]  mbs_data data structure of the model
* \param[out] f vectors which contains the added equibrium functions :  =f(xe, ... )
*
*/
void user_equil_fxe(MbsData *mbs_data, double* f)  
{
    //f[x] must be zero at the equilibrium, (x = 1 2 ... nxe)

    //f[1]= mbs_data->q[1] - mbs_data->q[2]   // write down your equation here
    //f[2]= ...  ;
    //...
    //f[nxe]= ;
}
