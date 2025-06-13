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
//	==> Function: F7 - Link Forces (1D)
//
//	==> Git hash: a9259031f25de7c9f86dbac08464ef31e84ae3b1
//
//	==> Input XML
//

#include <math.h> 

#include "mbs_data.h"

#include "mbs_project_interface.h"

void mbs_link(double **frc, double **trq, double *Flink, double *Z, double *Zd,
MbsData *s, double tsim)
{
#include "mbs_link_philippides_c.h"

double *q, *qd;
double **l, **dpt;

frc = s->frc;
trq = s->trq;
Z = s->Z;
Zd = s->Zd;

q = s->q;
qd = s->qd;

dpt = s->dpt;
l  = s->l;

// Number of continuation lines = 0

#include "mbs_message.h"

mbs_msg("Your symbolic files seem obsolete, i.e. not up-to-date with your MBsysPad model. Please regenerate your symbolic files (MBsysPad->Tools->Generate Symbolic Files). Exiting.\n");
mbs_msg("Error raised in mbs_link.\n");
s->flag_stop = 1;
return;
}
