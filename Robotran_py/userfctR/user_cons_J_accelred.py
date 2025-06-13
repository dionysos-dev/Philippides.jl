# -*- coding: utf-8 -*-
"""
Module for the definition of user constraints.
For accelred module (dirdyn) only

Summary
-------
The user constraints enable to impose constraints that can not be resolved using
classical Robotran cuts.
"""
# Author: Robotran Team
# (c) Universite catholique de Louvain, 2022


def user_cons_J_accelred(mbs_data, tsim):
    """Compute the Jacobian for the user constraints.
    The user constraints must be solved inside this function, by the user. 
    The Jacobian is stored in mbs_data.

    Parameters
    ----------
    mbs_data : MBsysPy.MbsData
        The instance containing the multibody project.
    tsim : float
        The current time of the simulation.

    Returns
    -------
    None
    """

    # Example: Compute the value of Jac.
    # mbs_data.jac_user[1,1] =  1.
    # mbs_data.jac_user[1,2] = -2*mbs_data.q[2].
    
    return
