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
#	==> Generation Date: Fri Apr  4 12:04:43 2025
#	==> using automatic loading with extension .mbs 
#
#	==> Project name: Project_mbs
#
#	==> Number of joints: 13
#
#	==> Function: F2 - Recursive Inverse Dynamics of tree-like MBS
#
#	==> Git hash: a9259031f25de7c9f86dbac08464ef31e84ae3b1
#
#	==> Input XML
#

from math import sin, cos

def invdyna(phi,s,tsim):
    Qq = phi  # compatibility with symbolic generation
    q = s.q
    qd = s.qd
    qdd = s.qdd
 
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

 
# Forward Kinematics

    ALPHA32 = qdd[2]-s.g[3]
    BS93 = -qd[3]*qd[3]
    ALPHA13 = qdd[1]*C3-ALPHA32*S3
    ALPHA33 = qdd[1]*S3+ALPHA32*C3
    BS94 = -qd[3]*qd[3]
    ALPHA14 = ALPHA13+(2.0)*qd[3]*qd[4]+qdd[3]*Dz43
    ALPHA34 = qdd[4]+ALPHA33+BS93*Dz43
    OM25 = qd[3]+qd[5]
    OMp25 = qdd[3]+qdd[5]
    BS95 = -OM25*OM25
    ALPHA15 = ALPHA14*C5-ALPHA34*S5
    ALPHA35 = ALPHA14*S5+ALPHA34*C5
    BS96 = -OM25*OM25
    ALPHA16 = ALPHA15+(2.0)*qd[6]*OM25+OMp25*Dz63
    ALPHA36 = qdd[6]+ALPHA35+BS95*Dz63
    OM27 = qd[7]+OM25
    OMp27 = qdd[7]+OMp25
    BS97 = -OM27*OM27
    ALPHA17 = ALPHA16*C7-ALPHA36*S7
    ALPHA37 = ALPHA16*S7+ALPHA36*C7
    OM28 = qd[8]+OM27
    OMp28 = qdd[8]+OMp27
    BS98 = -OM28*OM28
    ALPHA18 = C8*(ALPHA17+OMp27*s.dpt[3,7])-S8*(ALPHA37+BS97*s.dpt[3,7])
    ALPHA38 = C8*(ALPHA37+BS97*s.dpt[3,7])+S8*(ALPHA17+OMp27*s.dpt[3,7])
    BS99 = -qd[3]*qd[3]
    ALPHA19 = ALPHA13+(2.0)*qd[3]*qd[9]+qdd[3]*Dz93
    ALPHA39 = qdd[9]+ALPHA33+BS93*Dz93
    OM210 = qd[10]+qd[3]
    OMp210 = qdd[10]+qdd[3]
    BS910 = -OM210*OM210
    ALPHA110 = ALPHA19*C10-ALPHA39*S10
    ALPHA310 = ALPHA19*S10+ALPHA39*C10
    BS911 = -OM210*OM210
    ALPHA111 = ALPHA110+(2.0)*qd[11]*OM210+OMp210*Dz113
    ALPHA311 = qdd[11]+ALPHA310+BS910*Dz113
    OM212 = qd[12]+OM210
    OMp212 = qdd[12]+OMp210
    BS912 = -OM212*OM212
    ALPHA112 = ALPHA111*C12-ALPHA311*S12
    ALPHA312 = ALPHA111*S12+ALPHA311*C12
    OM213 = qd[13]+OM212
    OMp213 = qdd[13]+OMp212
    BS913 = -OM213*OM213
    ALPHA113 = C13*(ALPHA112+OMp212*s.dpt[3,14])-S13*(ALPHA312+BS912*s.dpt[3,14])
    ALPHA313 = C13*(ALPHA312+BS912*s.dpt[3,14])+S13*(ALPHA112+OMp212*s.dpt[3,14])
 
# Backward Dynamics

    Fs113 = -s.frc[1,13]+s.m[13]*(ALPHA113+OMp213*s.l[3,13])
    Fs313 = -s.frc[3,13]+s.m[13]*(ALPHA313+BS913*s.l[3,13])
    Cq213 = -s.trq[2,13]+s.In[5,13]*OMp213+Fs113*s.l[3,13]
    Fs112 = -s.frc[1,12]+s.m[12]*(ALPHA112+OMp212*s.l[3,12])
    Fs312 = -s.frc[3,12]+s.m[12]*(ALPHA312+BS912*s.l[3,12])
    Fq112 = Fs112+Fs113*C13+Fs313*S13
    Fq312 = Fs312-Fs113*S13+Fs313*C13
    Cq212 = -s.trq[2,12]+Cq213+s.In[5,12]*OMp212+Fs112*s.l[3,12]+s.dpt[3,14]*(Fs113*C13+Fs313*S13)
    Fs111 = -s.frc[1,11]+s.m[11]*(ALPHA111+OMp210*s.l[3,11])
    Fs311 = -s.frc[3,11]+s.m[11]*(ALPHA311+BS911*s.l[3,11])
    Fq111 = Fs111+Fq112*C12+Fq312*S12
    Fq311 = Fs311-Fq112*S12+Fq312*C12
    Cq211 = -s.trq[2,11]+Cq212+s.In[5,11]*OMp210+Fs111*s.l[3,11]
    Fs110 = -s.frc[1,10]+s.m[10]*(ALPHA110+OMp210*s.l[3,10])
    Fs310 = -s.frc[3,10]+s.m[10]*(ALPHA310+BS910*s.l[3,10])
    Fq110 = Fq111+Fs110
    Fq310 = Fq311+Fs310
    Cq210 = -s.trq[2,10]+Cq211+s.In[5,10]*OMp210+Fq111*Dz113+Fs110*s.l[3,10]
    Fs19 = -s.frc[1,9]+s.m[9]*(ALPHA19+qdd[3]*s.l[3,9])
    Fs39 = -s.frc[3,9]+s.m[9]*(ALPHA39+BS99*s.l[3,9])
    Fq19 = Fs19+Fq110*C10+Fq310*S10
    Fq39 = Fs39-Fq110*S10+Fq310*C10
    Cq29 = -s.trq[2,9]+Cq210+qdd[3]*s.In[5,9]+Fs19*s.l[3,9]
    Fs18 = -s.frc[1,8]+s.m[8]*(ALPHA18+OMp28*s.l[3,8])
    Fs38 = -s.frc[3,8]+s.m[8]*(ALPHA38+BS98*s.l[3,8])
    Cq28 = -s.trq[2,8]+s.In[5,8]*OMp28+Fs18*s.l[3,8]
    Fs17 = -s.frc[1,7]+s.m[7]*(ALPHA17+OMp27*s.l[3,7])
    Fs37 = -s.frc[3,7]+s.m[7]*(ALPHA37+BS97*s.l[3,7])
    Fq17 = Fs17+Fs18*C8+Fs38*S8
    Fq37 = Fs37-Fs18*S8+Fs38*C8
    Cq27 = -s.trq[2,7]+Cq28+s.In[5,7]*OMp27+Fs17*s.l[3,7]+s.dpt[3,7]*(Fs18*C8+Fs38*S8)
    Fs16 = -s.frc[1,6]+s.m[6]*(ALPHA16+OMp25*s.l[3,6])
    Fs36 = -s.frc[3,6]+s.m[6]*(ALPHA36+BS96*s.l[3,6])
    Fq16 = Fs16+Fq17*C7+Fq37*S7
    Fq36 = Fs36-Fq17*S7+Fq37*C7
    Cq26 = -s.trq[2,6]+Cq27+s.In[5,6]*OMp25+Fs16*s.l[3,6]
    Fs15 = -s.frc[1,5]+s.m[5]*(ALPHA15+OMp25*s.l[3,5])
    Fs35 = -s.frc[3,5]+s.m[5]*(ALPHA35+BS95*s.l[3,5])
    Fq15 = Fq16+Fs15
    Fq35 = Fq36+Fs35
    Cq25 = -s.trq[2,5]+Cq26+s.In[5,5]*OMp25+Fq16*Dz63+Fs15*s.l[3,5]
    Fs14 = -s.frc[1,4]+s.m[4]*(ALPHA14+qdd[3]*s.l[3,4])
    Fs34 = -s.frc[3,4]+s.m[4]*(ALPHA34+BS94*s.l[3,4])
    Fq14 = Fs14+Fq15*C5+Fq35*S5
    Fq34 = Fs34-Fq15*S5+Fq35*C5
    Cq24 = -s.trq[2,4]+Cq25+qdd[3]*s.In[5,4]+Fs14*s.l[3,4]
    Fs13 = -s.frc[1,3]+s.m[3]*ALPHA13
    Fs33 = -s.frc[3,3]+s.m[3]*ALPHA33
    Fq13 = Fq14+Fq19+Fs13
    Fq33 = Fq34+Fq39+Fs33
    Cq23 = -s.trq[2,3]+Cq24+Cq29+qdd[3]*s.In[5,3]+Fq14*Dz43+Fq19*Dz93
    Fq12 = -s.frc[1,2]+Fq13*C3+Fq33*S3
    Fq32 = -s.frc[3,2]-Fq13*S3+Fq33*C3
    Fs11 = -s.frc[1,1]+qdd[1]*s.m[1]
    Fq11 = Fq12+Fs11
 
# Symbolic model output

    Qq[1] = Fq11
    Qq[2] = Fq32
    Qq[3] = Cq23
    Qq[4] = Fq34
    Qq[5] = Cq25
    Qq[6] = Fq36
    Qq[7] = Cq27
    Qq[8] = Cq28
    Qq[9] = Fq39
    Qq[10] = Cq210
    Qq[11] = Fq311
    Qq[12] = Cq212
    Qq[13] = Cq213

# Number of continuation lines = 0


