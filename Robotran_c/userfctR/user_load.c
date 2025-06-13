/**
 * @file user_load.c
 *
 * This files enable the user to call custom code to initialize the project at
 * loading and properly closing it at freeing.
 *
 * @author Robotran team
 *
 * (c) Universite catholique de Louvain
 */

#include "mbs_data.h"

/*! \brief User own initialization operations.
 *
 * This function will automatically be called at the project loading by mbs_load()
 * function. The user can define in this function various initialization, allocation
 * or configuration steps required by the project (ie. allocating and configuring
 * user model of type being structure).
 *
 * \param[in,out] mbs_data Data structure of the model.
 * \param[in] mbs_loader Pointer to the mbs_loader structure as filled in mbs_load_with_loader function
 *
 */
void user_load_post(MbsData *mbs_data, MbsLoader *mbs_loader)
{
    
    /*
        This function is called at the end of mbs_load() and is useful when compiling the project 
        separately in order for example to use Simulink (when the executable does not call the main.c)
    */
}

/*!
 * \brief User own freeing operations.
 *
 * This function will automatically be called at the project freeing by mbs_delete_data()
 * function. All memory allocated by the user (ie. in a user
 * model) should be properly freed. Such memory may have been allocated in the
 * user_load_post() function.
 *
 * \param[in,out] mbs_data data structure of the model
 */
void user_free(MbsData *mbs_data)
{
    // This function is called at the beginning of mbs_delete_data().
}
