# -*- coding: utf-8 -*-
"""Module for the definition of functions related to Equilibrium analysis."""
# Author: Robotran Team
# (c) Universite catholique de Louvain, 2020


def user_equil_init(mbs_data, mbs_equil):
    """Run specific operation required by the user before running equilibrium.

    Parameters
    ----------
    mbs_data : MBsysPy.MbsData
        The instance containing the multibody project.
    mbs_equil : MBsysPy.MbsEquil
        The instance of the current equilibrium process.

    Returns
    -------
    None.

    """

    return


def user_equil_loop(mbs_data, mbs_equil):
    """Run specific operation required by the user at each equilibrium step.

    Parameters
    ----------
    mbs_data : MBsysPy.MbsData
        The instance containing the multibody project.
    mbs_equil : MBsysPy.MbsEquil
        The instance of the current equilibrium process.

    Returns
    -------
    None.

    """

    return


def user_equil_finish(mbs_data, mbs_equil):
    """Run specific operation required by the user at the end of equilibrium process.

    Parameters
    ----------
    mbs_data : MBsysPy.MbsData
        The instance containing the multibody project.
    mbs_equil : MBsysPy.MbsEquil
        The instance of the current equilibrium process.

    Returns
    -------
    None.

    """

    return


def user_equil_fxe(mbs_data, f):
    """Add user-defined equilibrium equation to the system.

    The equilibrium process, if successful, will reach fun[1:]=0.

    The values assigned to f have to be floats (not integers).

    Parameters
    ----------
    mbs_data: MBsysPy.MbsData
        The instance containing the multibody project.
    f: numpy.ndarray
        Numpy array containing the residue of the user equilibrium equations.

        The first index (f[0]) must not be modified. The residues are filled in
        f[1: nxe+1] (the array is oversized to the number of equilibrium variables).

    Returns
    -------
    None

    Examples
    --------
    # In this example at the equilibrium the value of the fifth generalized
    coordinate is 0:
    f[1] = mbs_data.q[5]
    # Here we use an user function from an external module
    import my_external_module  # Should be done outside the function.
    f[2] = my_external_module.my_function(mbs_data)

    """

    return
