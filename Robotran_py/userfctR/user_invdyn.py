# -*- coding: utf-8 -*-
"""Module for the definition of user functions related to inverse dynamic analysis."""
# Author: Robotran Team
# (c) Universite catholique de Louvain, 2021


def user_invdyn_init(mbs_data, mbs_invdyn):
    """Run specific operation required by the user before running inverse dynamic.

    Parameters
    ----------
    mbs_data : MBsysPy.MbsData
        The instance containing the multibody project.
    mbs_invdyn : MBsysPy.MbsInvdyn
        The instance of the current inverse dynamic process.

    Returns
    -------
    None.

    """

    return


def user_invdyn_loop(mbs_data, mbs_invdyn):
    """Run specific operation required by the user at the end of step.

    Parameters
    ----------
    mbs_data : MBsysPy.MbsData
        The instance containing the multibody project.
    mbs_invdyn : MBsysPy.MbsDirdyn
        The instance of the current inverse dynamic process.

    Returns
    -------
    None.
    """

    return


def user_invdyn_finish(mbs_data, mbs_invdyn):
    """Run specific operations required by the user when inverse dynamic analysis ends.

    Parameters
    ----------
    mbs_data : MBsysPy.MbsData
        The instance containing the multibody project.
    mbs_invdyn : MBsysPy.MbsDirdyn
        The instance of the current inverse dynamic process.

    Returns
    -------
    None.

    """

    return
