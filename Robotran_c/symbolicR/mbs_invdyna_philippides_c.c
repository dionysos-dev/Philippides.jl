//
//	MBsysTran - Release 8.1
//
//	Copyright 
//	Universite catholique de Louvain (UCLouvain) 
//	Mechatronic, Electrical Energy, and Dynamic systems (MEED Division) 
//	2, Place du Levant
//	1348 Louvain-la-Neuve 
//	Belgium 
//
//	http://www.robotran.be 
//
//	==> Generation Date: Fri Apr  4 13:00:28 2025
//	==> using automatic loading with extension .mbs 
//
//	==> Project name: philippides_c
//
//	==> Number of joints: 13
//
//	==> Function: F2 - Recursive Inverse Dynamics of tree-like MBS
//
//	==> Git hash: a9259031f25de7c9f86dbac08464ef31e84ae3b1
//
//	==> Input XML
//

#include <math.h> 

#include "mbs_data.h"

 
void mbs_invdyna(double *Qq,
MbsData *s, double tsim)
{
#include "mbs_invdyna_philippides_c.h"

double *q, *qd, *qdd;
double *g, *m;
double **l, **In, **dpt, **frc, **trq;

q = s->q;
qd = s->qd;
qdd = s->qdd;

dpt = s->dpt;
l   = s->l;

m = s->m;
In  = s->In;

frc = s->frc;
trq = s->trq;
g = s->g;
 
// Trigonometric functions

S3 = sin(q[3]);
C3 = cos(q[3]);
S5 = sin(q[5]);
C5 = cos(q[5]);
S7 = sin(q[7]);
C7 = cos(q[7]);
S8 = sin(q[8]);
C8 = cos(q[8]);
S10 = sin(q[10]);
C10 = cos(q[10]);
S12 = sin(q[12]);
C12 = cos(q[12]);
S13 = sin(q[13]);
C13 = cos(q[13]);
 
// Augmented Joint Position Vectors

Dz43 = q[4]+dpt[3][4];
Dz63 = q[6]+dpt[3][6];
Dz93 = q[9]+dpt[3][5];
Dz113 = q[11]+dpt[3][13];
 
// Augmented Joint Position Vectors

 
// Forward Kinematics

ALPHA32 = qdd[2]-g[3];
BS93 = -qd[3]*qd[3];
ALPHA13 = qdd[1]*C3-ALPHA32*S3;
ALPHA33 = qdd[1]*S3+ALPHA32*C3;
BS94 = -qd[3]*qd[3];
ALPHA14 = ALPHA13+(2.0)*qd[3]*qd[4]+qdd[3]*Dz43;
ALPHA34 = qdd[4]+ALPHA33+BS93*Dz43;
OM25 = qd[3]+qd[5];
OMp25 = qdd[3]+qdd[5];
BS95 = -OM25*OM25;
ALPHA15 = ALPHA14*C5-ALPHA34*S5;
ALPHA35 = ALPHA14*S5+ALPHA34*C5;
BS96 = -OM25*OM25;
ALPHA16 = ALPHA15+(2.0)*qd[6]*OM25+OMp25*Dz63;
ALPHA36 = qdd[6]+ALPHA35+BS95*Dz63;
OM27 = qd[7]+OM25;
OMp27 = qdd[7]+OMp25;
BS97 = -OM27*OM27;
ALPHA17 = ALPHA16*C7-ALPHA36*S7;
ALPHA37 = ALPHA16*S7+ALPHA36*C7;
OM28 = qd[8]+OM27;
OMp28 = qdd[8]+OMp27;
BS98 = -OM28*OM28;
ALPHA18 = C8*(ALPHA17+OMp27*dpt[3][7])-S8*(ALPHA37+BS97*dpt[3][7]);
ALPHA38 = C8*(ALPHA37+BS97*dpt[3][7])+S8*(ALPHA17+OMp27*dpt[3][7]);
BS99 = -qd[3]*qd[3];
ALPHA19 = ALPHA13+(2.0)*qd[3]*qd[9]+qdd[3]*Dz93;
ALPHA39 = qdd[9]+ALPHA33+BS93*Dz93;
OM210 = qd[10]+qd[3];
OMp210 = qdd[10]+qdd[3];
BS910 = -OM210*OM210;
ALPHA110 = ALPHA19*C10-ALPHA39*S10;
ALPHA310 = ALPHA19*S10+ALPHA39*C10;
BS911 = -OM210*OM210;
ALPHA111 = ALPHA110+(2.0)*qd[11]*OM210+OMp210*Dz113;
ALPHA311 = qdd[11]+ALPHA310+BS910*Dz113;
OM212 = qd[12]+OM210;
OMp212 = qdd[12]+OMp210;
BS912 = -OM212*OM212;
ALPHA112 = ALPHA111*C12-ALPHA311*S12;
ALPHA312 = ALPHA111*S12+ALPHA311*C12;
OM213 = qd[13]+OM212;
OMp213 = qdd[13]+OMp212;
BS913 = -OM213*OM213;
ALPHA113 = C13*(ALPHA112+OMp212*dpt[3][14])-S13*(ALPHA312+BS912*dpt[3][14]);
ALPHA313 = C13*(ALPHA312+BS912*dpt[3][14])+S13*(ALPHA112+OMp212*dpt[3][14]);
 
// Backward Dynamics

Fs113 = -frc[1][13]+m[13]*(ALPHA113+OMp213*l[3][13]);
Fs313 = -frc[3][13]+m[13]*(ALPHA313+BS913*l[3][13]);
Cq213 = -trq[2][13]+In[5][13]*OMp213+Fs113*l[3][13];
Fs112 = -frc[1][12]+m[12]*(ALPHA112+OMp212*l[3][12]);
Fs312 = -frc[3][12]+m[12]*(ALPHA312+BS912*l[3][12]);
Fq112 = Fs112+Fs113*C13+Fs313*S13;
Fq312 = Fs312-Fs113*S13+Fs313*C13;
Cq212 = -trq[2][12]+Cq213+In[5][12]*OMp212+Fs112*l[3][12]+dpt[3][14]*(Fs113*C13+Fs313*S13);
Fs111 = -frc[1][11]+m[11]*(ALPHA111+OMp210*l[3][11]);
Fs311 = -frc[3][11]+m[11]*(ALPHA311+BS911*l[3][11]);
Fq111 = Fs111+Fq112*C12+Fq312*S12;
Fq311 = Fs311-Fq112*S12+Fq312*C12;
Cq211 = -trq[2][11]+Cq212+In[5][11]*OMp210+Fs111*l[3][11];
Fs110 = -frc[1][10]+m[10]*(ALPHA110+OMp210*l[3][10]);
Fs310 = -frc[3][10]+m[10]*(ALPHA310+BS910*l[3][10]);
Fq110 = Fq111+Fs110;
Fq310 = Fq311+Fs310;
Cq210 = -trq[2][10]+Cq211+In[5][10]*OMp210+Fq111*Dz113+Fs110*l[3][10];
Fs19 = -frc[1][9]+m[9]*(ALPHA19+qdd[3]*l[3][9]);
Fs39 = -frc[3][9]+m[9]*(ALPHA39+BS99*l[3][9]);
Fq19 = Fs19+Fq110*C10+Fq310*S10;
Fq39 = Fs39-Fq110*S10+Fq310*C10;
Cq29 = -trq[2][9]+Cq210+qdd[3]*In[5][9]+Fs19*l[3][9];
Fs18 = -frc[1][8]+m[8]*(ALPHA18+OMp28*l[3][8]);
Fs38 = -frc[3][8]+m[8]*(ALPHA38+BS98*l[3][8]);
Cq28 = -trq[2][8]+In[5][8]*OMp28+Fs18*l[3][8];
Fs17 = -frc[1][7]+m[7]*(ALPHA17+OMp27*l[3][7]);
Fs37 = -frc[3][7]+m[7]*(ALPHA37+BS97*l[3][7]);
Fq17 = Fs17+Fs18*C8+Fs38*S8;
Fq37 = Fs37-Fs18*S8+Fs38*C8;
Cq27 = -trq[2][7]+Cq28+In[5][7]*OMp27+Fs17*l[3][7]+dpt[3][7]*(Fs18*C8+Fs38*S8);
Fs16 = -frc[1][6]+m[6]*(ALPHA16+OMp25*l[3][6]);
Fs36 = -frc[3][6]+m[6]*(ALPHA36+BS96*l[3][6]);
Fq16 = Fs16+Fq17*C7+Fq37*S7;
Fq36 = Fs36-Fq17*S7+Fq37*C7;
Cq26 = -trq[2][6]+Cq27+In[5][6]*OMp25+Fs16*l[3][6];
Fs15 = -frc[1][5]+m[5]*(ALPHA15+OMp25*l[3][5]);
Fs35 = -frc[3][5]+m[5]*(ALPHA35+BS95*l[3][5]);
Fq15 = Fq16+Fs15;
Fq35 = Fq36+Fs35;
Cq25 = -trq[2][5]+Cq26+In[5][5]*OMp25+Fq16*Dz63+Fs15*l[3][5];
Fs14 = -frc[1][4]+m[4]*(ALPHA14+qdd[3]*l[3][4]);
Fs34 = -frc[3][4]+m[4]*(ALPHA34+BS94*l[3][4]);
Fq14 = Fs14+Fq15*C5+Fq35*S5;
Fq34 = Fs34-Fq15*S5+Fq35*C5;
Cq24 = -trq[2][4]+Cq25+qdd[3]*In[5][4]+Fs14*l[3][4];
Fs13 = -frc[1][3]+m[3]*ALPHA13;
Fs33 = -frc[3][3]+m[3]*ALPHA33;
Fq13 = Fq14+Fq19+Fs13;
Fq33 = Fq34+Fq39+Fs33;
Cq23 = -trq[2][3]+Cq24+Cq29+qdd[3]*In[5][3]+Fq14*Dz43+Fq19*Dz93;
Fq12 = -frc[1][2]+Fq13*C3+Fq33*S3;
Fq32 = -frc[3][2]-Fq13*S3+Fq33*C3;
Fs11 = -frc[1][1]+qdd[1]*m[1];
Fq11 = Fq12+Fs11;
 
// Symbolic model output

Qq[1] = Fq11;
Qq[2] = Fq32;
Qq[3] = Cq23;
Qq[4] = Fq34;
Qq[5] = Cq25;
Qq[6] = Fq36;
Qq[7] = Cq27;
Qq[8] = Cq28;
Qq[9] = Fq39;
Qq[10] = Cq210;
Qq[11] = Fq311;
Qq[12] = Cq212;
Qq[13] = Cq213;

// Number of continuation lines = 0

}
