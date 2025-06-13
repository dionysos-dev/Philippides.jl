# -*- coding: utf-8 -*-
"""Module for the definition of driven joints."""
# Author: Robotran Team
# (c) Universite catholique de Louvain, 2020

def user_DrivenJoints(mbs_data, tsim):
    """Set the values of the driven joints directly in the MbsData structure.

    The position, velocity and acceleration of the driven joints must be set in
    the attributes mbs_data.q, mbs_data.qd and mbs_data.qdd .

    Parameters
    ----------
    mbs_data : MBsysPy.MbsData
        The multibody system associated to this computation.
    tsim : float
        The current time of the simulation.

    Returns
    -------
    None
    """
    # The motors are rigidly linked to the hip/thighs
    # The rotation of the hip is kept locked for the moment

    fixed_translations_ids = [mbs_data.joint_id["T_lh_fixed"], mbs_data.joint_id["T_rh_fixed"],
                              mbs_data.joint_id["T_lk_fixed"],mbs_data.joint_id["T_lk_fixed"],mbs_data.joint_id["R_hip"] ]
    for elem in fixed_translations_ids:
        mbs_data.q[elem]    = 0
        mbs_data.qd[elem]   = 0
        mbs_data.qdd[elem]  = 0  

    return
