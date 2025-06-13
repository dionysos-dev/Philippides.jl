# -*- coding: utf-8 -*-
"""
Module for the definition of the derivatives of the user states.

Summary
-------
The user derivatives allows to add some equations to be time-integrated with
the multibody system.
"""
# Author: Robotran Team
# (c) Universite catholique de Louvain, 2020


def user_derivatives(mbs_data):
    """Compute the derivatives of the user states.

    Parameters
    ----------
    mbs_data : MBsysPy.MbsData
        The instance containing the multibody project. The derivatives values
        must be filled in the 'uxd' attribute which is a numpy.ndarray. The first
        index (mbs_data.uxd[0]) must not be modified. The first index to be
        filled is mbs_data.uxd[1].

    Returns
    -------
    None
    """

    # Example: integration of sin(t) in first user state variable
    # mbs_data.uxd[1] = np.cos(mbs_data.tsim)

    return
