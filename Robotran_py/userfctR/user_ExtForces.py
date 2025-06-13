# -*- coding: utf-8 -*-
"""Module for defining user function required to compute external forces."""
# Author: Robotran Team
# (c) Universite catholique de Louvain, 2019
#TODO: Ajuster les valeur des modèle de contact (avec Brieuc)
import numpy as np

class Contact_Manager:
    def __init__(self, nSensors):
        """
        Initialize the memory of the contact points for the tangential contact forces.

        Parameters:
            nSensors (int) : Number of force sensors + 1
            Contact_PxF : Positions of the contact points for each sensor
            Previous_PxF : Previous position of the sensor
            InContact : Status of the sensor { 0: no contact, 1: contact }
            results : Force outputs on each sensor [ixF,Fx,Fy,Fz] for ixF
        """
        self.nSensors = nSensors
        self.Contact_PxF  = np.full((nSensors, 4), np.nan)
        self.Previous_PxF = np.full((nSensors, 4), np.nan)
        self.InContact    = np.zeros(nSensors, dtype=int)
        self.results      = np.zeros(4*nSensors)
        return 
    
    def update_contact(self, ixF, PxF):
        """
        Update the contact memory with the new sensor position.
        If a contact is detected (Zsensor < 0) the contact point is computed and the status is updated.

        Parameters:
            ixF : Sensor ID
            PxF : Current position of the sensor
        """
        PxF = PxF[:4]  # [&, x, y, z]
        # Initialization
        if np.isnan(self.Previous_PxF[ixF, 0]):     # Nan -> PxF is the initial position
            self.Previous_PxF[ixF] = PxF            #
            if PxF[3] <= 0:                         # Initial position in conact -> contact point set at interface z =0
                self.Contact_PxF[ixF] = PxF         #
                self.Contact_PxF[ixF, 3] = 0.0      #
                self.InContact[ixF] = 1             #
        
        # Detection of penetration
        if PxF[3] <= 0:                                                                                                 # Penetration
            if self.InContact[ixF] == 0:                                                                                # Contact initiation
                ratio = self.Previous_PxF[ixF, 3] / (self.Previous_PxF[ixF, 3] - PxF[3])                                # Linear interpolation to find 
                self.Contact_PxF[ixF, :4] = self.Previous_PxF[ixF, :4] + ratio * (PxF - self.Previous_PxF[ixF, :4])     #   exact contact point
                self.Contact_PxF[ixF, 3] = 0.0                                                                          # Ensure z = 0 (redundant)
                self.InContact[ixF] = 1                                                                                 # Update status
        else:
            self.InContact[ixF] = 0     # Update the status if contact is interrupted
        self.Previous_PxF[ixF] = PxF    # 

        return
    
    def update_slip_contact(self, ixF, PxF, delta_slip_x, delta_slip_y, slip):
        """
        Update the position of the contact point when slip is detected

        Parameters:
            ixF : Sensor ID
            PxF : Current position of the sensor
            delta_slip_x : Distance between the contact point and the position of the sensor along x during slip
            delta_slip_y :  "" along y
            slip (int) : binary {0: stick , 1: slip}
        """
        if slip == 1 and self.InContact[ixF] == 1:                                       
            self.Contact_PxF[ixF, 1:3] = PxF[1:3] + np.array([delta_slip_x, delta_slip_y]) 
        return
    
    def compute_penetrations(self, ixF, PxF, VxF):
        """
        Compute the penetrations and penetrations speeds along all directions with respect to the contact point

        Parameters:
            ixF : Sensor ID
            PxF : Current position of the sensor
            VxF : Current speed of the sensor
        
        Returns:
            deltas : Penetrations in the inertial frame
            deltas_dot : Penetration speeds in the inertial frame
        """
        deltas = np.zeros(3)                                    # Penetrations set to 0 
        deltas_dot = np.zeros(3)                                #   if no contact
        if self.InContact[ixF] == 1:                            # 
            deltas = PxF[1:4] - self.Contact_PxF[ixF, 1:4]      # Current position - Contact position
            deltas_dot = VxF[1:4]                               # Penetration speeds are the sensor speeds
        return deltas, deltas_dot


class HuntCrossleyHertz:
    def __init__(self, k, n, d):
        """
        Initialize the Hunt-Crossley Hertz contact model.

        Parameters:
        k (float): Stiffness coefficient 
        n (float): Nonlinearity exponent (usually 3/2 for Hertzian contact)
        d (float): Damping coefficient
        """
        self.k = k
        self.n = n
        self.d = d

    def compute_normal_force(self, delta, delta_dot):
        """
        Compute the normal contact force.

        Parameters:
        delta (float): Penetration depth (m)
        delta_dot (float): Penetration rate (m/s)

        Returns:
        float: Normal force (N)
        """
        if delta < 0:                                                          # Detect penetration
            Fz = self.k * (abs(delta) ** self.n) * (1 - self.d * delta_dot)    # H-C-H model
        else:                                                                  #
            Fz = 0                                                             # No force if there's no contact
        return Fz


class ViscoelasticCoulombModel:
    def __init__(self, mu, k, d):
        """
        Initialize the Viscoelastic Coulomb contact model.

        Parameters:
        mu (float): Coefficient of friction
        k (float): Stiffness coefficient
        d (float): Damping coefficient
        """
        self.mu = mu
        self.k = k
        self.d = d

    def compute_tangential_force(self, F_n, delta_x, delta_x_dot, delta_y, delta_y_dot):
        """
        Compute the tangential friction force.

        Parameters:
        F_n (float): Normal force (N)
        delta_x, delta_y (float): Tangential penetrations (m)
        delta_x_dot, delta_y_dot (float): Tangential penetration velocities (m/s)

        Returns:
            Fx, Fy : Forces along x and y with respect to the sensor position in the inertial frame
            delta_no_slip_x, delta_no_slip_y : Distances between the contact point and the current position along x and y
            slip : {0: no lateral slip, 1: lateral slip}
        """
        # Compute raw viscoelastic forces
        # F = -k x - d x_dot
        Fx = -self.k * delta_x - self.d * delta_x_dot   
        Fy = -self.k * delta_y - self.d * delta_y_dot  

        # Compute total force magnitude
        F_total = np.sqrt(Fx**2 + Fy**2)
        # Maximum Coulomb force
        F_max = max(self.mu * F_n,0)                          

        # Initialisation /!\ Not accurate if no slippage
        delta_no_slip_x, delta_no_slip_y = 0, 0        
        slip = 0

        # Apply saturation
        if F_total > F_max:
            scaling_factor = F_max / F_total                        # Force normalisation respecting proportions
            Fx *= scaling_factor                                    #
            Fy *= scaling_factor                                    #
            delta_no_slip_x = -(Fx + self.d * delta_x_dot) / self.k # Inverse ViscoElsatic model for Coulomb saturated forces
            delta_no_slip_y = -(Fy + self.d * delta_y_dot) / self.k # 
            slip = 1                                                # Update slip status
        return Fx, Fy, delta_no_slip_x, delta_no_slip_y, slip
    
    def compute_normal_force(self, delta, delta_dot):
        """ 
        Compute normal force using a viscoelastic model.

        Parameters:
            delta : Normal penetration
            delta_dot : Normal penetration speed
        """
        # ViscoElastic Model
        if(delta <= 0):
            Fz = -self.k * delta - self.d * delta_dot   
        else:
            Fz = 0
        return Fz

# Normal Model Choice 0: Hunt-Crossley-Hertz, 1: Viscoelastic Coulomb
Model = 0
# Instantiate a Hunt-Crossley-Hertz friction model
# lambda = 3/2 * k * alpha = k * d => d = 3/2 *alpha
# Xing [5e4 , 1.5 , 0.3]
# Brieuc [1e4, 1.5, 3.3]
k = 5e4       # Stiffness coefficient
n = 1.5       # Nonlinearity exponent (3/2 for Hertzian contact)
d = 0.3       # Damping coefficient
hch_model = HuntCrossleyHertz(k, n, d)

# Instantiate a ViscoElastic Coulomb model
# Xing [0.8,2e4,100.0]
mu = 0.8      # Coefficient of friction
k = 2e4       # Stiffness coefficient
d = 50.0     # Damping coefficient
vc_model = ViscoelasticCoulombModel(mu, k, d)

# Instantiate a ContactManager
cm = Contact_Manager(11) # For 10 sensors to be accessed through their ID

def user_ExtForces(PxF, RxF, VxF, OMxF, AxF, OMPxF, mbs_data, tsim, ixF):
    """Compute an user-specified external force.

    Parameters
    ----------
    PxF : numpy.ndarray
        Position vector (index starting at 1) of the force sensor expressed in
        the inertial frame: PxF[1:4] = [P_x, P_y, P_z]
    RxF : numpy.ndarray
        Rotation matrix (index starting at 1) from the inertial frame to the
        force sensor frame: Frame_sensor = RxF[1:4,1:4] * Frame_inertial
    VxF : numpy.ndarray
        Velocity vector (index starting at 1) of the force sensor expressed in
        the inertial frame: VxF[1:4] = [V_x, V_y, V_z]
    OMxF : numpy.ndarray
        Angular velocity vector (index starting at 1) of the force sensor
        expressed in the inertial frame: OMxF[1:4] = [OM_x, OM_y, OM_z]
    AxF : numpy.ndarray
        Acceleration vector (index starting at 1) of the force sensor expressed
        in the inertial frame: AxF[1:4] = [A_x, A_y, A_z]
    OMPxF : numpy.ndarray
        Angular acceleration vector (index starting at 1) of the force sensor
        expressed in the inertial frame: OMPxF[1:4] = [OMP_x, OMP_y, OMP_z]
    mbs_data : MBsysPy.MbsData
        The multibody system associated to this computation.
    tsim : float
        The current time of the simulation.
    ixF : int
        The ID identifying the computed force sensor.

    Notes
    -----
    For 1D numpy.ndarray with index starting at 1, the first index (array[0])
    must not be modified. The first index to be filled is array[1].

    For 2D numpy.ndarray with index starting at 1, the first row (mat[0, :]) and
    line (mat[:,0]) must not be modified. The subarray to be filled is mat[1:, 1:].

    Returns
    -------
    Swr : numpy.ndarray
        An array of length 10 equal to [0., Fx, Fy, Fz, Mx, My, Mz, dxF].
        F_# are the forces components expressed in inertial frame.
        M_# are the torques components expressed in inertial frame.
        dxF is an array of length 3 containing the component of the forces/torque
        application point expressed in the BODY-FIXED frame.
        This array is a specific line of MbsData.SWr.
    """

    Fx = 0.0
    Fy = 0.0
    Fz = 0.0
    Mx = 0.0
    My = 0.0
    Mz = 0.0
    idpt = mbs_data.xfidpt[ixF]
    dxF = mbs_data.dpt[1:, idpt]
    """Compute user-defined external forces."""
    # Update contact information
    cm.update_contact(ixF, PxF)

    # Compute penetrations
    deltas, deltas_dot = cm.compute_penetrations(ixF, PxF, VxF)
    # Normal force computation
    # 0: Hunt-Crossley-Hertz, 1: Viscoelastic Coulomb
    if Model == 0:
        Fz = hch_model.compute_normal_force(deltas[2], deltas_dot[2])
    elif Model == 1:
        Fz = vc_model.compute_normal_force(deltas[2], deltas_dot[2])

    # Compute tangential forces
    Fx, Fy, delta_slip_x, delta_slip_y, slip = vc_model.compute_tangential_force(
        Fz, deltas[0], deltas_dot[0], deltas[1], deltas_dot[1]
    )

    # Update slip contact
    cm.update_slip_contact(ixF, PxF, delta_slip_x, delta_slip_y, slip)


    """
     Example : Contact force with a wall when X coordinate is higher than 1m.
               The force is perfectly horizontal (inertial frame)
     xlim = 1.0 # m
     kwall= 1e5 # N/m
     if PxF[1]>xlim:
         Fx = (PxF[1]-xlim)*kwall
    """

    # Concatenating force, torque and force application point to returned array.
    # This must not be modified.
    Swr = mbs_data.SWr[ixF]
    Swr[1:] = [Fx, Fy, Fz, Mx, My, Mz, dxF[0], dxF[1], dxF[2]]
    cm.results[4*ixF : 4*(ixF +1)] = np.array([ixF,Fx,Fy,Fz])
    if(ixF == 10): mbs_data.Force_Sensors = cm.results[4:]

    return Swr
