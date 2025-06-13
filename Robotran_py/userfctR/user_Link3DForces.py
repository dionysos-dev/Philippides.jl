# -*- coding: utf-8 -*-
"""Module for the definition of user 3D links forces."""
# Author: Robotran Team
# (c) Universite catholique de Louvain, 2020

import numpy as np


def user_Link3DForces(PxF, RxF, VxF, OMxF, AxF, OMPxF, mbs_data, tsim, ixF):
    """Compute the force in the given 3Dlink.
    
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
           Link 3D are not yet supported in MBsysPy
    This function (and its description) may be erroneous
                        Do not use 
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    
    A 3D link is defined by it's starting point A (on body_A) and the ending
    point B (on body_B).

    The force and torque applied on both body is located on point A.

    Parameters
    ----------
    PxF : numpy.ndarray
        Position vector (index starting at 1) from point A to point B expressed
        the inertial frame: PxF[1:4] = [P_x, P_y, P_z]
    RxF : numpy.ndarray
        Rotation matrix (index starting at 1) from body_A frame to the body_B
        frame: Frame_sbody_B = RxF[1:4,1:4] * Frame_body_A
    VxF : numpy.ndarray
        Relative velocity vector (index starting at 1) between point A and B
        expressed in the inertial frame: VxF[1:4] = [V_x, V_y, V_z]
    OMxF : numpy.ndarray
        Relative angular velocity vector (index starting at 1) between point A
        and B expressed in the inertial frame: OMxF[1:4] = [OM_x, OM_y, OM_z]
    AxF : numpy.ndarray
        Relative acceleration vector (index starting at 1) between point A and B
        expressed in the inertial frame: AxF[1:4] = [A_x, A_y, A_z]
    OMPxF : numpy.ndarray
        Relative angular acceleration vector (index starting at 1) between point A
        and B expressed in the inertial frame: OMPxF[1:4] = [OMP_x, OMP_y, OMP_z]
    mbs_data : MBsysPy.MbsData
        The multibody system associated to this computation.
    tsim : float
        The current time of the simulation.
    ixF : int
        The ID identifying the computed  3Dlink.

    Returns
    -------
    Swr : numpy.ndarray
        An array of length 7 equal to [0., Fx, Fy, Fz, Mx, My, Mz].
        F_# are the forces components expressed in inertial frame.
        M_# are the torques components expressed in inertial frame.
    """
    
    Fx = 0.0
    Fy = 0.0
    Fz = 0.0
    Mx = 0.0
    My = 0.0
    Mz = 0.0

    # No simple example to give

    # Concatening force and torque to returned array.
    # This should not be modified.
    Swr = np.zeros(7)
    Swr[1:7] = np.r_[Fx, Fy, Fz, Mx, My, Mz]

    return Swr
