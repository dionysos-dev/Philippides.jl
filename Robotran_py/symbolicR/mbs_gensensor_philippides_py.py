#
#	MBsysTran - Release 8.1
#
#	Copyright 
#	Universite catholique de Louvain (UCLouvain) 
#	Mechatronic, Electrical Energy, and Dynamic systems (MEED Division) 
#	2, Place du Levant
#	1348 Louvain-la-Neuve 
#	Belgium 
#
#	http://www.robotran.be 
#
#	==> Generation Date: Fri Apr  4 12:37:42 2025
#	==> using automatic loading with extension .mbs 
#
#	==> Project name: philippides_py
#
#	==> Number of joints: 13
#
#	==> Function: F6 - Sensors Kinematics
#
#	==> Git hash: a9259031f25de7c9f86dbac08464ef31e84ae3b1
#
#	==> Input XML
#

from math import sin, cos, sqrt

def sensor(sens, s, isens):
  q = s.q
  qd = s.qd
  qdd = s.qdd

  dpt = s.dpt
 
# Trigonometric functions

  S3 = sin(q[3])
  C3 = cos(q[3])
  S5 = sin(q[5])
  C5 = cos(q[5])
  S7 = sin(q[7])
  C7 = cos(q[7])
  S8 = sin(q[8])
  C8 = cos(q[8])
  S10 = sin(q[10])
  C10 = cos(q[10])
  S12 = sin(q[12])
  C12 = cos(q[12])
  S13 = sin(q[13])
  C13 = cos(q[13])
 
# Augmented Joint Position Vectors

  Dz43 = q[4]+s.dpt[3,4]
  Dz63 = q[6]+s.dpt[3,6]
  Dz93 = q[9]+s.dpt[3,5]
  Dz113 = q[11]+s.dpt[3,13]
 
# Augmented Joint Position Vectors

 
# Sensor Kinematics


  if (isens == 1): 

    sens.P[1] = q[1]
    sens.P[2] = 0
    sens.P[3] = s.dpt[3,1]
    sens.R[1,1] = (1.0)
    sens.R[2,2] = (1.0)
    sens.R[3,3] = (1.0)
    sens.V[1] = qd[1]
    sens.V[2] = 0
    sens.V[3] = 0
    sens.OM[1] = 0
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.J[1,1] = (1.0)
    sens.A[1] = qdd[1]
    sens.A[2] = 0
    sens.A[3] = 0
    sens.OMP[1] = 0
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 2): 

    POcp2_32 = q[2]+s.dpt[3,1]
    sens.P[1] = q[1]
    sens.P[2] = s.dpt[2,2]
    sens.P[3] = POcp2_32
    sens.R[1,1] = (1.0)
    sens.R[2,2] = (1.0)
    sens.R[3,3] = (1.0)
    sens.V[1] = qd[1]
    sens.V[2] = 0
    sens.V[3] = qd[2]
    sens.OM[1] = 0
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.J[1,1] = (1.0)
    sens.J[3,2] = (1.0)
    sens.A[1] = qdd[1]
    sens.A[2] = 0
    sens.A[3] = qdd[2]
    sens.OMP[1] = 0
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 3): 

    POcp3_32 = q[2]+s.dpt[3,1]
    POcp3_23 = s.dpt[2,2]+s.dpt[2,3]
    sens.P[1] = q[1]
    sens.P[2] = POcp3_23
    sens.P[3] = POcp3_32
    sens.R[1,1] = C3
    sens.R[1,3] = -S3
    sens.R[2,2] = (1.0)
    sens.R[3,1] = S3
    sens.R[3,3] = C3
    sens.V[1] = qd[1]
    sens.V[2] = 0
    sens.V[3] = qd[2]
    sens.OM[1] = 0
    sens.OM[2] = qd[3]
    sens.OM[3] = 0
    sens.J[1,1] = (1.0)
    sens.J[3,2] = (1.0)
    sens.J[5,3] = (1.0)
    sens.A[1] = qdd[1]
    sens.A[2] = 0
    sens.A[3] = qdd[2]
    sens.OMP[1] = 0
    sens.OMP[2] = qdd[3]
    sens.OMP[3] = 0

  if (isens == 4): 

    POcp4_32 = q[2]+s.dpt[3,1]
    POcp4_23 = s.dpt[2,2]+s.dpt[2,3]
    RLcp4_14 = Dz43*S3
    RLcp4_34 = Dz43*C3
    POcp4_14 = RLcp4_14+q[1]
    POcp4_24 = POcp4_23+s.dpt[2,4]
    POcp4_34 = POcp4_32+RLcp4_34
    ORcp4_14 = RLcp4_34*qd[3]
    ORcp4_34 = -RLcp4_14*qd[3]
    VIcp4_14 = ORcp4_14+qd[1]+qd[4]*S3
    VIcp4_34 = ORcp4_34+qd[2]+qd[4]*C3
    ACcp4_14 = qdd[1]+ORcp4_34*qd[3]+RLcp4_34*qdd[3]+qdd[4]*S3+(2.0)*qd[3]*qd[4]*C3
    ACcp4_34 = qdd[2]-ORcp4_14*qd[3]-RLcp4_14*qdd[3]+qdd[4]*C3-(2.0)*qd[3]*qd[4]*S3
    sens.P[1] = POcp4_14
    sens.P[2] = POcp4_24
    sens.P[3] = POcp4_34
    sens.R[1,1] = C3
    sens.R[1,3] = -S3
    sens.R[2,2] = (1.0)
    sens.R[3,1] = S3
    sens.R[3,3] = C3
    sens.V[1] = VIcp4_14
    sens.V[2] = 0
    sens.V[3] = VIcp4_34
    sens.OM[1] = 0
    sens.OM[2] = qd[3]
    sens.OM[3] = 0
    sens.J[1,1] = (1.0)
    sens.J[1,3] = RLcp4_34
    sens.J[1,4] = S3
    sens.J[3,2] = (1.0)
    sens.J[3,3] = -RLcp4_14
    sens.J[3,4] = C3
    sens.J[5,3] = (1.0)
    sens.A[1] = ACcp4_14
    sens.A[2] = 0
    sens.A[3] = ACcp4_34
    sens.OMP[1] = 0
    sens.OMP[2] = qdd[3]
    sens.OMP[3] = 0

  if (isens == 5): 

    ROcp5_15 = C3*C5-S3*S5
    ROcp5_35 = -C3*S5-S3*C5
    ROcp5_75 = C3*S5+S3*C5
    ROcp5_95 = C3*C5-S3*S5
    POcp5_32 = q[2]+s.dpt[3,1]
    POcp5_23 = s.dpt[2,2]+s.dpt[2,3]
    RLcp5_14 = Dz43*S3
    RLcp5_34 = Dz43*C3
    POcp5_14 = RLcp5_14+q[1]
    POcp5_24 = POcp5_23+s.dpt[2,4]
    POcp5_34 = POcp5_32+RLcp5_34
    ORcp5_14 = RLcp5_34*qd[3]
    ORcp5_34 = -RLcp5_14*qd[3]
    VIcp5_14 = ORcp5_14+qd[1]+qd[4]*S3
    VIcp5_34 = ORcp5_34+qd[2]+qd[4]*C3
    ACcp5_14 = qdd[1]+ORcp5_34*qd[3]+RLcp5_34*qdd[3]+qdd[4]*S3+(2.0)*qd[3]*qd[4]*C3
    ACcp5_34 = qdd[2]-ORcp5_14*qd[3]-RLcp5_14*qdd[3]+qdd[4]*C3-(2.0)*qd[3]*qd[4]*S3
    OMcp5_25 = qd[3]+qd[5]
    OPcp5_25 = qdd[3]+qdd[5]
    sens.P[1] = POcp5_14
    sens.P[2] = POcp5_24
    sens.P[3] = POcp5_34
    sens.R[1,1] = ROcp5_15
    sens.R[1,3] = ROcp5_35
    sens.R[2,2] = (1.0)
    sens.R[3,1] = ROcp5_75
    sens.R[3,3] = ROcp5_95
    sens.V[1] = VIcp5_14
    sens.V[2] = 0
    sens.V[3] = VIcp5_34
    sens.OM[1] = 0
    sens.OM[2] = OMcp5_25
    sens.OM[3] = 0
    sens.J[1,1] = (1.0)
    sens.J[1,3] = RLcp5_34
    sens.J[1,4] = S3
    sens.J[3,2] = (1.0)
    sens.J[3,3] = -RLcp5_14
    sens.J[3,4] = C3
    sens.J[5,3] = (1.0)
    sens.J[5,5] = (1.0)
    sens.A[1] = ACcp5_14
    sens.A[2] = 0
    sens.A[3] = ACcp5_34
    sens.OMP[1] = 0
    sens.OMP[2] = OPcp5_25
    sens.OMP[3] = 0

  if (isens == 6): 

    ROcp6_15 = C3*C5-S3*S5
    ROcp6_35 = -C3*S5-S3*C5
    ROcp6_75 = C3*S5+S3*C5
    ROcp6_95 = C3*C5-S3*S5
    POcp6_32 = q[2]+s.dpt[3,1]
    POcp6_23 = s.dpt[2,2]+s.dpt[2,3]
    RLcp6_14 = Dz43*S3
    RLcp6_34 = Dz43*C3
    POcp6_14 = RLcp6_14+q[1]
    POcp6_24 = POcp6_23+s.dpt[2,4]
    POcp6_34 = POcp6_32+RLcp6_34
    ORcp6_14 = RLcp6_34*qd[3]
    ORcp6_34 = -RLcp6_14*qd[3]
    VIcp6_14 = ORcp6_14+qd[1]+qd[4]*S3
    VIcp6_34 = ORcp6_34+qd[2]+qd[4]*C3
    ACcp6_14 = qdd[1]+ORcp6_34*qd[3]+RLcp6_34*qdd[3]+qdd[4]*S3+(2.0)*qd[3]*qd[4]*C3
    ACcp6_34 = qdd[2]-ORcp6_14*qd[3]-RLcp6_14*qdd[3]+qdd[4]*C3-(2.0)*qd[3]*qd[4]*S3
    OMcp6_25 = qd[3]+qd[5]
    OPcp6_25 = qdd[3]+qdd[5]
    RLcp6_16 = ROcp6_75*Dz63
    RLcp6_36 = ROcp6_95*Dz63
    POcp6_16 = POcp6_14+RLcp6_16
    POcp6_36 = POcp6_34+RLcp6_36
    JTcp6_16_3 = RLcp6_34+RLcp6_36
    JTcp6_36_3 = -RLcp6_14-RLcp6_16
    ORcp6_16 = OMcp6_25*RLcp6_36
    ORcp6_36 = -OMcp6_25*RLcp6_16
    VIcp6_16 = ORcp6_16+VIcp6_14+ROcp6_75*qd[6]
    VIcp6_36 = ORcp6_36+VIcp6_34+ROcp6_95*qd[6]
    ACcp6_16 = ACcp6_14+OMcp6_25*ORcp6_36+(2.0)*OMcp6_25*ROcp6_95*qd[6]+OPcp6_25*RLcp6_36+ROcp6_75*qdd[6]
    ACcp6_36 = ACcp6_34-OMcp6_25*ORcp6_16-(2.0)*OMcp6_25*ROcp6_75*qd[6]-OPcp6_25*RLcp6_16+ROcp6_95*qdd[6]
    sens.P[1] = POcp6_16
    sens.P[2] = POcp6_24
    sens.P[3] = POcp6_36
    sens.R[1,1] = ROcp6_15
    sens.R[1,3] = ROcp6_35
    sens.R[2,2] = (1.0)
    sens.R[3,1] = ROcp6_75
    sens.R[3,3] = ROcp6_95
    sens.V[1] = VIcp6_16
    sens.V[2] = 0
    sens.V[3] = VIcp6_36
    sens.OM[1] = 0
    sens.OM[2] = OMcp6_25
    sens.OM[3] = 0
    sens.J[1,1] = (1.0)
    sens.J[1,3] = JTcp6_16_3
    sens.J[1,4] = S3
    sens.J[1,5] = RLcp6_36
    sens.J[1,6] = ROcp6_75
    sens.J[3,2] = (1.0)
    sens.J[3,3] = JTcp6_36_3
    sens.J[3,4] = C3
    sens.J[3,5] = -RLcp6_16
    sens.J[3,6] = ROcp6_95
    sens.J[5,3] = (1.0)
    sens.J[5,5] = (1.0)
    sens.A[1] = ACcp6_16
    sens.A[2] = 0
    sens.A[3] = ACcp6_36
    sens.OMP[1] = 0
    sens.OMP[2] = OPcp6_25
    sens.OMP[3] = 0

  if (isens == 7): 

    ROcp7_15 = C3*C5-S3*S5
    ROcp7_35 = -C3*S5-S3*C5
    ROcp7_75 = C3*S5+S3*C5
    ROcp7_95 = C3*C5-S3*S5
    ROcp7_17 = ROcp7_15*C7-ROcp7_75*S7
    ROcp7_37 = ROcp7_35*C7-ROcp7_95*S7
    ROcp7_77 = ROcp7_15*S7+ROcp7_75*C7
    ROcp7_97 = ROcp7_35*S7+ROcp7_95*C7
    POcp7_32 = q[2]+s.dpt[3,1]
    POcp7_23 = s.dpt[2,2]+s.dpt[2,3]
    RLcp7_14 = Dz43*S3
    RLcp7_34 = Dz43*C3
    POcp7_14 = RLcp7_14+q[1]
    POcp7_24 = POcp7_23+s.dpt[2,4]
    POcp7_34 = POcp7_32+RLcp7_34
    ORcp7_14 = RLcp7_34*qd[3]
    ORcp7_34 = -RLcp7_14*qd[3]
    VIcp7_14 = ORcp7_14+qd[1]+qd[4]*S3
    VIcp7_34 = ORcp7_34+qd[2]+qd[4]*C3
    ACcp7_14 = qdd[1]+ORcp7_34*qd[3]+RLcp7_34*qdd[3]+qdd[4]*S3+(2.0)*qd[3]*qd[4]*C3
    ACcp7_34 = qdd[2]-ORcp7_14*qd[3]-RLcp7_14*qdd[3]+qdd[4]*C3-(2.0)*qd[3]*qd[4]*S3
    OMcp7_25 = qd[3]+qd[5]
    OPcp7_25 = qdd[3]+qdd[5]
    RLcp7_16 = ROcp7_75*Dz63
    RLcp7_36 = ROcp7_95*Dz63
    POcp7_16 = POcp7_14+RLcp7_16
    POcp7_36 = POcp7_34+RLcp7_36
    JTcp7_16_3 = RLcp7_34+RLcp7_36
    JTcp7_36_3 = -RLcp7_14-RLcp7_16
    ORcp7_16 = OMcp7_25*RLcp7_36
    ORcp7_36 = -OMcp7_25*RLcp7_16
    VIcp7_16 = ORcp7_16+VIcp7_14+ROcp7_75*qd[6]
    VIcp7_36 = ORcp7_36+VIcp7_34+ROcp7_95*qd[6]
    ACcp7_16 = ACcp7_14+OMcp7_25*ORcp7_36+(2.0)*OMcp7_25*ROcp7_95*qd[6]+OPcp7_25*RLcp7_36+ROcp7_75*qdd[6]
    ACcp7_36 = ACcp7_34-OMcp7_25*ORcp7_16-(2.0)*OMcp7_25*ROcp7_75*qd[6]-OPcp7_25*RLcp7_16+ROcp7_95*qdd[6]
    OMcp7_27 = OMcp7_25+qd[7]
    OPcp7_27 = OPcp7_25+qdd[7]
    sens.P[1] = POcp7_16
    sens.P[2] = POcp7_24
    sens.P[3] = POcp7_36
    sens.R[1,1] = ROcp7_17
    sens.R[1,3] = ROcp7_37
    sens.R[2,2] = (1.0)
    sens.R[3,1] = ROcp7_77
    sens.R[3,3] = ROcp7_97
    sens.V[1] = VIcp7_16
    sens.V[2] = 0
    sens.V[3] = VIcp7_36
    sens.OM[1] = 0
    sens.OM[2] = OMcp7_27
    sens.OM[3] = 0
    sens.J[1,1] = (1.0)
    sens.J[1,3] = JTcp7_16_3
    sens.J[1,4] = S3
    sens.J[1,5] = RLcp7_36
    sens.J[1,6] = ROcp7_75
    sens.J[3,2] = (1.0)
    sens.J[3,3] = JTcp7_36_3
    sens.J[3,4] = C3
    sens.J[3,5] = -RLcp7_16
    sens.J[3,6] = ROcp7_95
    sens.J[5,3] = (1.0)
    sens.J[5,5] = (1.0)
    sens.J[5,7] = (1.0)
    sens.A[1] = ACcp7_16
    sens.A[2] = 0
    sens.A[3] = ACcp7_36
    sens.OMP[1] = 0
    sens.OMP[2] = OPcp7_27
    sens.OMP[3] = 0

  if (isens == 8): 

    ROcp8_15 = C3*C5-S3*S5
    ROcp8_35 = -C3*S5-S3*C5
    ROcp8_75 = C3*S5+S3*C5
    ROcp8_95 = C3*C5-S3*S5
    ROcp8_17 = ROcp8_15*C7-ROcp8_75*S7
    ROcp8_37 = ROcp8_35*C7-ROcp8_95*S7
    ROcp8_77 = ROcp8_15*S7+ROcp8_75*C7
    ROcp8_97 = ROcp8_35*S7+ROcp8_95*C7
    ROcp8_18 = ROcp8_17*C8-ROcp8_77*S8
    ROcp8_38 = ROcp8_37*C8-ROcp8_97*S8
    ROcp8_78 = ROcp8_17*S8+ROcp8_77*C8
    ROcp8_98 = ROcp8_37*S8+ROcp8_97*C8
    POcp8_32 = q[2]+s.dpt[3,1]
    POcp8_23 = s.dpt[2,2]+s.dpt[2,3]
    RLcp8_14 = Dz43*S3
    RLcp8_34 = Dz43*C3
    POcp8_14 = RLcp8_14+q[1]
    POcp8_24 = POcp8_23+s.dpt[2,4]
    POcp8_34 = POcp8_32+RLcp8_34
    ORcp8_14 = RLcp8_34*qd[3]
    ORcp8_34 = -RLcp8_14*qd[3]
    VIcp8_14 = ORcp8_14+qd[1]+qd[4]*S3
    VIcp8_34 = ORcp8_34+qd[2]+qd[4]*C3
    ACcp8_14 = qdd[1]+ORcp8_34*qd[3]+RLcp8_34*qdd[3]+qdd[4]*S3+(2.0)*qd[3]*qd[4]*C3
    ACcp8_34 = qdd[2]-ORcp8_14*qd[3]-RLcp8_14*qdd[3]+qdd[4]*C3-(2.0)*qd[3]*qd[4]*S3
    OMcp8_25 = qd[3]+qd[5]
    OPcp8_25 = qdd[3]+qdd[5]
    RLcp8_16 = ROcp8_75*Dz63
    RLcp8_36 = ROcp8_95*Dz63
    POcp8_16 = POcp8_14+RLcp8_16
    POcp8_36 = POcp8_34+RLcp8_36
    JTcp8_16_3 = RLcp8_34+RLcp8_36
    JTcp8_36_3 = -RLcp8_14-RLcp8_16
    ORcp8_16 = OMcp8_25*RLcp8_36
    ORcp8_36 = -OMcp8_25*RLcp8_16
    VIcp8_16 = ORcp8_16+VIcp8_14+ROcp8_75*qd[6]
    VIcp8_36 = ORcp8_36+VIcp8_34+ROcp8_95*qd[6]
    ACcp8_16 = ACcp8_14+OMcp8_25*ORcp8_36+(2.0)*OMcp8_25*ROcp8_95*qd[6]+OPcp8_25*RLcp8_36+ROcp8_75*qdd[6]
    ACcp8_36 = ACcp8_34-OMcp8_25*ORcp8_16-(2.0)*OMcp8_25*ROcp8_75*qd[6]-OPcp8_25*RLcp8_16+ROcp8_95*qdd[6]
    OMcp8_27 = OMcp8_25+qd[7]
    OPcp8_27 = OPcp8_25+qdd[7]
    RLcp8_18 = ROcp8_77*s.dpt[3,7]
    RLcp8_38 = ROcp8_97*s.dpt[3,7]
    POcp8_18 = POcp8_16+RLcp8_18
    POcp8_38 = POcp8_36+RLcp8_38
    JTcp8_18_3 = JTcp8_16_3+RLcp8_38
    JTcp8_38_3 = JTcp8_36_3-RLcp8_18
    JTcp8_18_5 = RLcp8_36+RLcp8_38
    JTcp8_38_5 = -RLcp8_16-RLcp8_18
    OMcp8_28 = OMcp8_27+qd[8]
    ORcp8_18 = OMcp8_27*RLcp8_38
    ORcp8_38 = -OMcp8_27*RLcp8_18
    VIcp8_18 = ORcp8_18+VIcp8_16
    VIcp8_38 = ORcp8_38+VIcp8_36
    OPcp8_28 = OPcp8_27+qdd[8]
    ACcp8_18 = ACcp8_16+OMcp8_27*ORcp8_38+OPcp8_27*RLcp8_38
    ACcp8_38 = ACcp8_36-OMcp8_27*ORcp8_18-OPcp8_27*RLcp8_18
    sens.P[1] = POcp8_18
    sens.P[2] = POcp8_24
    sens.P[3] = POcp8_38
    sens.R[1,1] = ROcp8_18
    sens.R[1,3] = ROcp8_38
    sens.R[2,2] = (1.0)
    sens.R[3,1] = ROcp8_78
    sens.R[3,3] = ROcp8_98
    sens.V[1] = VIcp8_18
    sens.V[2] = 0
    sens.V[3] = VIcp8_38
    sens.OM[1] = 0
    sens.OM[2] = OMcp8_28
    sens.OM[3] = 0
    sens.J[1,1] = (1.0)
    sens.J[1,3] = JTcp8_18_3
    sens.J[1,4] = S3
    sens.J[1,5] = JTcp8_18_5
    sens.J[1,6] = ROcp8_75
    sens.J[1,7] = RLcp8_38
    sens.J[3,2] = (1.0)
    sens.J[3,3] = JTcp8_38_3
    sens.J[3,4] = C3
    sens.J[3,5] = JTcp8_38_5
    sens.J[3,6] = ROcp8_95
    sens.J[3,7] = -RLcp8_18
    sens.J[5,3] = (1.0)
    sens.J[5,5] = (1.0)
    sens.J[5,7] = (1.0)
    sens.J[5,8] = (1.0)
    sens.A[1] = ACcp8_18
    sens.A[2] = 0
    sens.A[3] = ACcp8_38
    sens.OMP[1] = 0
    sens.OMP[2] = OPcp8_28
    sens.OMP[3] = 0

  if (isens == 9): 

    POcp9_32 = q[2]+s.dpt[3,1]
    POcp9_23 = s.dpt[2,2]+s.dpt[2,3]
    RLcp9_14 = Dz93*S3
    RLcp9_34 = Dz93*C3
    POcp9_14 = RLcp9_14+q[1]
    POcp9_24 = POcp9_23+s.dpt[2,5]
    POcp9_34 = POcp9_32+RLcp9_34
    ORcp9_14 = RLcp9_34*qd[3]
    ORcp9_34 = -RLcp9_14*qd[3]
    VIcp9_14 = ORcp9_14+qd[1]+qd[9]*S3
    VIcp9_34 = ORcp9_34+qd[2]+qd[9]*C3
    ACcp9_14 = qdd[1]+ORcp9_34*qd[3]+RLcp9_34*qdd[3]+qdd[9]*S3+(2.0)*qd[3]*qd[9]*C3
    ACcp9_34 = qdd[2]-ORcp9_14*qd[3]-RLcp9_14*qdd[3]+qdd[9]*C3-(2.0)*qd[3]*qd[9]*S3
    sens.P[1] = POcp9_14
    sens.P[2] = POcp9_24
    sens.P[3] = POcp9_34
    sens.R[1,1] = C3
    sens.R[1,3] = -S3
    sens.R[2,2] = (1.0)
    sens.R[3,1] = S3
    sens.R[3,3] = C3
    sens.V[1] = VIcp9_14
    sens.V[2] = 0
    sens.V[3] = VIcp9_34
    sens.OM[1] = 0
    sens.OM[2] = qd[3]
    sens.OM[3] = 0
    sens.J[1,1] = (1.0)
    sens.J[1,3] = RLcp9_34
    sens.J[1,9] = S3
    sens.J[3,2] = (1.0)
    sens.J[3,3] = -RLcp9_14
    sens.J[3,9] = C3
    sens.J[5,3] = (1.0)
    sens.A[1] = ACcp9_14
    sens.A[2] = 0
    sens.A[3] = ACcp9_34
    sens.OMP[1] = 0
    sens.OMP[2] = qdd[3]
    sens.OMP[3] = 0

  if (isens == 10): 

    ROcp10_110 = C10*C3-S10*S3
    ROcp10_310 = -C10*S3-S10*C3
    ROcp10_710 = C10*S3+S10*C3
    ROcp10_910 = C10*C3-S10*S3
    POcp10_32 = q[2]+s.dpt[3,1]
    POcp10_23 = s.dpt[2,2]+s.dpt[2,3]
    RLcp10_14 = Dz93*S3
    RLcp10_34 = Dz93*C3
    POcp10_14 = RLcp10_14+q[1]
    POcp10_24 = POcp10_23+s.dpt[2,5]
    POcp10_34 = POcp10_32+RLcp10_34
    ORcp10_14 = RLcp10_34*qd[3]
    ORcp10_34 = -RLcp10_14*qd[3]
    VIcp10_14 = ORcp10_14+qd[1]+qd[9]*S3
    VIcp10_34 = ORcp10_34+qd[2]+qd[9]*C3
    ACcp10_14 = qdd[1]+ORcp10_34*qd[3]+RLcp10_34*qdd[3]+qdd[9]*S3+(2.0)*qd[3]*qd[9]*C3
    ACcp10_34 = qdd[2]-ORcp10_14*qd[3]-RLcp10_14*qdd[3]+qdd[9]*C3-(2.0)*qd[3]*qd[9]*S3
    OMcp10_25 = qd[10]+qd[3]
    OPcp10_25 = qdd[10]+qdd[3]
    sens.P[1] = POcp10_14
    sens.P[2] = POcp10_24
    sens.P[3] = POcp10_34
    sens.R[1,1] = ROcp10_110
    sens.R[1,3] = ROcp10_310
    sens.R[2,2] = (1.0)
    sens.R[3,1] = ROcp10_710
    sens.R[3,3] = ROcp10_910
    sens.V[1] = VIcp10_14
    sens.V[2] = 0
    sens.V[3] = VIcp10_34
    sens.OM[1] = 0
    sens.OM[2] = OMcp10_25
    sens.OM[3] = 0
    sens.J[1,1] = (1.0)
    sens.J[1,3] = RLcp10_34
    sens.J[1,9] = S3
    sens.J[3,2] = (1.0)
    sens.J[3,3] = -RLcp10_14
    sens.J[3,9] = C3
    sens.J[5,3] = (1.0)
    sens.J[5,10] = (1.0)
    sens.A[1] = ACcp10_14
    sens.A[2] = 0
    sens.A[3] = ACcp10_34
    sens.OMP[1] = 0
    sens.OMP[2] = OPcp10_25
    sens.OMP[3] = 0

  if (isens == 11): 

    ROcp11_110 = C10*C3-S10*S3
    ROcp11_310 = -C10*S3-S10*C3
    ROcp11_710 = C10*S3+S10*C3
    ROcp11_910 = C10*C3-S10*S3
    POcp11_32 = q[2]+s.dpt[3,1]
    POcp11_23 = s.dpt[2,2]+s.dpt[2,3]
    RLcp11_14 = Dz93*S3
    RLcp11_34 = Dz93*C3
    POcp11_14 = RLcp11_14+q[1]
    POcp11_24 = POcp11_23+s.dpt[2,5]
    POcp11_34 = POcp11_32+RLcp11_34
    ORcp11_14 = RLcp11_34*qd[3]
    ORcp11_34 = -RLcp11_14*qd[3]
    VIcp11_14 = ORcp11_14+qd[1]+qd[9]*S3
    VIcp11_34 = ORcp11_34+qd[2]+qd[9]*C3
    ACcp11_14 = qdd[1]+ORcp11_34*qd[3]+RLcp11_34*qdd[3]+qdd[9]*S3+(2.0)*qd[3]*qd[9]*C3
    ACcp11_34 = qdd[2]-ORcp11_14*qd[3]-RLcp11_14*qdd[3]+qdd[9]*C3-(2.0)*qd[3]*qd[9]*S3
    OMcp11_25 = qd[10]+qd[3]
    OPcp11_25 = qdd[10]+qdd[3]
    RLcp11_16 = ROcp11_710*Dz113
    RLcp11_36 = ROcp11_910*Dz113
    POcp11_16 = POcp11_14+RLcp11_16
    POcp11_36 = POcp11_34+RLcp11_36
    JTcp11_16_3 = RLcp11_34+RLcp11_36
    JTcp11_36_3 = -RLcp11_14-RLcp11_16
    ORcp11_16 = OMcp11_25*RLcp11_36
    ORcp11_36 = -OMcp11_25*RLcp11_16
    VIcp11_16 = ORcp11_16+VIcp11_14+ROcp11_710*qd[11]
    VIcp11_36 = ORcp11_36+VIcp11_34+ROcp11_910*qd[11]
    ACcp11_16 = ACcp11_14+OMcp11_25*ORcp11_36+(2.0)*OMcp11_25*ROcp11_910*qd[11]+OPcp11_25*RLcp11_36+ROcp11_710*qdd[11]
    ACcp11_36 = ACcp11_34-OMcp11_25*ORcp11_16-(2.0)*OMcp11_25*ROcp11_710*qd[11]-OPcp11_25*RLcp11_16+ROcp11_910*qdd[11]
    sens.P[1] = POcp11_16
    sens.P[2] = POcp11_24
    sens.P[3] = POcp11_36
    sens.R[1,1] = ROcp11_110
    sens.R[1,3] = ROcp11_310
    sens.R[2,2] = (1.0)
    sens.R[3,1] = ROcp11_710
    sens.R[3,3] = ROcp11_910
    sens.V[1] = VIcp11_16
    sens.V[2] = 0
    sens.V[3] = VIcp11_36
    sens.OM[1] = 0
    sens.OM[2] = OMcp11_25
    sens.OM[3] = 0
    sens.J[1,1] = (1.0)
    sens.J[1,3] = JTcp11_16_3
    sens.J[1,9] = S3
    sens.J[1,10] = RLcp11_36
    sens.J[1,11] = ROcp11_710
    sens.J[3,2] = (1.0)
    sens.J[3,3] = JTcp11_36_3
    sens.J[3,9] = C3
    sens.J[3,10] = -RLcp11_16
    sens.J[3,11] = ROcp11_910
    sens.J[5,3] = (1.0)
    sens.J[5,10] = (1.0)
    sens.A[1] = ACcp11_16
    sens.A[2] = 0
    sens.A[3] = ACcp11_36
    sens.OMP[1] = 0
    sens.OMP[2] = OPcp11_25
    sens.OMP[3] = 0

  if (isens == 12): 

    ROcp12_110 = C10*C3-S10*S3
    ROcp12_310 = -C10*S3-S10*C3
    ROcp12_710 = C10*S3+S10*C3
    ROcp12_910 = C10*C3-S10*S3
    ROcp12_112 = ROcp12_110*C12-ROcp12_710*S12
    ROcp12_312 = ROcp12_310*C12-ROcp12_910*S12
    ROcp12_712 = ROcp12_110*S12+ROcp12_710*C12
    ROcp12_912 = ROcp12_310*S12+ROcp12_910*C12
    POcp12_32 = q[2]+s.dpt[3,1]
    POcp12_23 = s.dpt[2,2]+s.dpt[2,3]
    RLcp12_14 = Dz93*S3
    RLcp12_34 = Dz93*C3
    POcp12_14 = RLcp12_14+q[1]
    POcp12_24 = POcp12_23+s.dpt[2,5]
    POcp12_34 = POcp12_32+RLcp12_34
    ORcp12_14 = RLcp12_34*qd[3]
    ORcp12_34 = -RLcp12_14*qd[3]
    VIcp12_14 = ORcp12_14+qd[1]+qd[9]*S3
    VIcp12_34 = ORcp12_34+qd[2]+qd[9]*C3
    ACcp12_14 = qdd[1]+ORcp12_34*qd[3]+RLcp12_34*qdd[3]+qdd[9]*S3+(2.0)*qd[3]*qd[9]*C3
    ACcp12_34 = qdd[2]-ORcp12_14*qd[3]-RLcp12_14*qdd[3]+qdd[9]*C3-(2.0)*qd[3]*qd[9]*S3
    OMcp12_25 = qd[10]+qd[3]
    OPcp12_25 = qdd[10]+qdd[3]
    RLcp12_16 = ROcp12_710*Dz113
    RLcp12_36 = ROcp12_910*Dz113
    POcp12_16 = POcp12_14+RLcp12_16
    POcp12_36 = POcp12_34+RLcp12_36
    JTcp12_16_3 = RLcp12_34+RLcp12_36
    JTcp12_36_3 = -RLcp12_14-RLcp12_16
    ORcp12_16 = OMcp12_25*RLcp12_36
    ORcp12_36 = -OMcp12_25*RLcp12_16
    VIcp12_16 = ORcp12_16+VIcp12_14+ROcp12_710*qd[11]
    VIcp12_36 = ORcp12_36+VIcp12_34+ROcp12_910*qd[11]
    ACcp12_16 = ACcp12_14+OMcp12_25*ORcp12_36+(2.0)*OMcp12_25*ROcp12_910*qd[11]+OPcp12_25*RLcp12_36+ROcp12_710*qdd[11]
    ACcp12_36 = ACcp12_34-OMcp12_25*ORcp12_16-(2.0)*OMcp12_25*ROcp12_710*qd[11]-OPcp12_25*RLcp12_16+ROcp12_910*qdd[11]
    OMcp12_27 = OMcp12_25+qd[12]
    OPcp12_27 = OPcp12_25+qdd[12]
    sens.P[1] = POcp12_16
    sens.P[2] = POcp12_24
    sens.P[3] = POcp12_36
    sens.R[1,1] = ROcp12_112
    sens.R[1,3] = ROcp12_312
    sens.R[2,2] = (1.0)
    sens.R[3,1] = ROcp12_712
    sens.R[3,3] = ROcp12_912
    sens.V[1] = VIcp12_16
    sens.V[2] = 0
    sens.V[3] = VIcp12_36
    sens.OM[1] = 0
    sens.OM[2] = OMcp12_27
    sens.OM[3] = 0
    sens.J[1,1] = (1.0)
    sens.J[1,3] = JTcp12_16_3
    sens.J[1,9] = S3
    sens.J[1,10] = RLcp12_36
    sens.J[1,11] = ROcp12_710
    sens.J[3,2] = (1.0)
    sens.J[3,3] = JTcp12_36_3
    sens.J[3,9] = C3
    sens.J[3,10] = -RLcp12_16
    sens.J[3,11] = ROcp12_910
    sens.J[5,3] = (1.0)
    sens.J[5,10] = (1.0)
    sens.J[5,12] = (1.0)
    sens.A[1] = ACcp12_16
    sens.A[2] = 0
    sens.A[3] = ACcp12_36
    sens.OMP[1] = 0
    sens.OMP[2] = OPcp12_27
    sens.OMP[3] = 0

  if (isens == 13): 

    ROcp13_110 = C10*C3-S10*S3
    ROcp13_310 = -C10*S3-S10*C3
    ROcp13_710 = C10*S3+S10*C3
    ROcp13_910 = C10*C3-S10*S3
    ROcp13_112 = ROcp13_110*C12-ROcp13_710*S12
    ROcp13_312 = ROcp13_310*C12-ROcp13_910*S12
    ROcp13_712 = ROcp13_110*S12+ROcp13_710*C12
    ROcp13_912 = ROcp13_310*S12+ROcp13_910*C12
    ROcp13_113 = ROcp13_112*C13-ROcp13_712*S13
    ROcp13_313 = ROcp13_312*C13-ROcp13_912*S13
    ROcp13_713 = ROcp13_112*S13+ROcp13_712*C13
    ROcp13_913 = ROcp13_312*S13+ROcp13_912*C13
    POcp13_32 = q[2]+s.dpt[3,1]
    POcp13_23 = s.dpt[2,2]+s.dpt[2,3]
    RLcp13_14 = Dz93*S3
    RLcp13_34 = Dz93*C3
    POcp13_14 = RLcp13_14+q[1]
    POcp13_24 = POcp13_23+s.dpt[2,5]
    POcp13_34 = POcp13_32+RLcp13_34
    ORcp13_14 = RLcp13_34*qd[3]
    ORcp13_34 = -RLcp13_14*qd[3]
    VIcp13_14 = ORcp13_14+qd[1]+qd[9]*S3
    VIcp13_34 = ORcp13_34+qd[2]+qd[9]*C3
    ACcp13_14 = qdd[1]+ORcp13_34*qd[3]+RLcp13_34*qdd[3]+qdd[9]*S3+(2.0)*qd[3]*qd[9]*C3
    ACcp13_34 = qdd[2]-ORcp13_14*qd[3]-RLcp13_14*qdd[3]+qdd[9]*C3-(2.0)*qd[3]*qd[9]*S3
    OMcp13_25 = qd[10]+qd[3]
    OPcp13_25 = qdd[10]+qdd[3]
    RLcp13_16 = ROcp13_710*Dz113
    RLcp13_36 = ROcp13_910*Dz113
    POcp13_16 = POcp13_14+RLcp13_16
    POcp13_36 = POcp13_34+RLcp13_36
    JTcp13_16_3 = RLcp13_34+RLcp13_36
    JTcp13_36_3 = -RLcp13_14-RLcp13_16
    ORcp13_16 = OMcp13_25*RLcp13_36
    ORcp13_36 = -OMcp13_25*RLcp13_16
    VIcp13_16 = ORcp13_16+VIcp13_14+ROcp13_710*qd[11]
    VIcp13_36 = ORcp13_36+VIcp13_34+ROcp13_910*qd[11]
    ACcp13_16 = ACcp13_14+OMcp13_25*ORcp13_36+(2.0)*OMcp13_25*ROcp13_910*qd[11]+OPcp13_25*RLcp13_36+ROcp13_710*qdd[11]
    ACcp13_36 = ACcp13_34-OMcp13_25*ORcp13_16-(2.0)*OMcp13_25*ROcp13_710*qd[11]-OPcp13_25*RLcp13_16+ROcp13_910*qdd[11]
    OMcp13_27 = OMcp13_25+qd[12]
    OPcp13_27 = OPcp13_25+qdd[12]
    RLcp13_18 = ROcp13_712*s.dpt[3,14]
    RLcp13_38 = ROcp13_912*s.dpt[3,14]
    POcp13_18 = POcp13_16+RLcp13_18
    POcp13_38 = POcp13_36+RLcp13_38
    JTcp13_18_3 = JTcp13_16_3+RLcp13_38
    JTcp13_38_3 = JTcp13_36_3-RLcp13_18
    JTcp13_18_5 = RLcp13_36+RLcp13_38
    JTcp13_38_5 = -RLcp13_16-RLcp13_18
    OMcp13_28 = OMcp13_27+qd[13]
    ORcp13_18 = OMcp13_27*RLcp13_38
    ORcp13_38 = -OMcp13_27*RLcp13_18
    VIcp13_18 = ORcp13_18+VIcp13_16
    VIcp13_38 = ORcp13_38+VIcp13_36
    OPcp13_28 = OPcp13_27+qdd[13]
    ACcp13_18 = ACcp13_16+OMcp13_27*ORcp13_38+OPcp13_27*RLcp13_38
    ACcp13_38 = ACcp13_36-OMcp13_27*ORcp13_18-OPcp13_27*RLcp13_18
    sens.P[1] = POcp13_18
    sens.P[2] = POcp13_24
    sens.P[3] = POcp13_38
    sens.R[1,1] = ROcp13_113
    sens.R[1,3] = ROcp13_313
    sens.R[2,2] = (1.0)
    sens.R[3,1] = ROcp13_713
    sens.R[3,3] = ROcp13_913
    sens.V[1] = VIcp13_18
    sens.V[2] = 0
    sens.V[3] = VIcp13_38
    sens.OM[1] = 0
    sens.OM[2] = OMcp13_28
    sens.OM[3] = 0
    sens.J[1,1] = (1.0)
    sens.J[1,3] = JTcp13_18_3
    sens.J[1,9] = S3
    sens.J[1,10] = JTcp13_18_5
    sens.J[1,11] = ROcp13_710
    sens.J[1,12] = RLcp13_38
    sens.J[3,2] = (1.0)
    sens.J[3,3] = JTcp13_38_3
    sens.J[3,9] = C3
    sens.J[3,10] = JTcp13_38_5
    sens.J[3,11] = ROcp13_910
    sens.J[3,12] = -RLcp13_18
    sens.J[5,3] = (1.0)
    sens.J[5,10] = (1.0)
    sens.J[5,12] = (1.0)
    sens.J[5,13] = (1.0)
    sens.A[1] = ACcp13_18
    sens.A[2] = 0
    sens.A[3] = ACcp13_38
    sens.OMP[1] = 0
    sens.OMP[2] = OPcp13_28
    sens.OMP[3] = 0

 


# Number of continuation lines = 0


