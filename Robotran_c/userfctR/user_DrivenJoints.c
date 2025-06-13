/* ---------------------------
 * Robotran - MBsysC
 * 
 * Template file for the driven joint function
 * 
 * (c) Universite catholique de Louvain
 *     
 */

 #include "math.h"
 #include "user_all_id.h"
 #include "mbs_data.h"
 
 
 void user_DrivenJoints(MbsData *mbs_data,double tsim)
 {
 
     int fixed_translations_ids[] = {
         T_lh_fixed_id,
         T_rh_fixed_id,
         T_lk_fixed_id,
         T_rk_fixed_id, 
         R_hip_id
     };
 
     int nb_ids = 5;
     for (int i = 0; i < nb_ids; i++)
     {
         int elem = fixed_translations_ids[i];
         mbs_data->q[elem] = 0.0;
         mbs_data->qd[elem] = 0.0;
         mbs_data->qdd[elem] = 0.0;
     }
 }