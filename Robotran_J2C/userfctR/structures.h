#ifndef STRUCTURES_H
#define STRUCTURES_H

//------------------------------------------------------------------------------
// File: structures.h
// Description: Contains all structure definitions of the user functions
//------------------------------------------------------------------------------

#ifdef _WIN32
    // On Windows, we need to export symbols with DLL export attributes
    #define EXPORT_SYMBOL __declspec(dllexport)
#else
    // On Unix-like systems (Linux/macOS), we use the visibility attribute
    #define EXPORT_SYMBOL __attribute__((visibility("default")))
#endif

typedef struct {
    int nSensors;
    double **Contact_PxF;   // Positions of the contact points for each sensor (nSensors x 4)
    double **Previous_PxF;  // Previous positions of the sensor (nSensors x 4)
    int *InContact;         // Status of the sensor (0 or 1) (nSensors)
    double *results;        // Force outputs on each sensor (4 * nSensors)
} Contact_Manager;

typedef struct {
    double k; // Stiffness coefficient
    double n; // Nonlinearity exponent
    double d; // Damping coefficient
} HuntCrossleyHertz;

typedef struct {
    double mu; // Coefficient of friction
    double k;  // Stiffness coefficient
    double d;  // Damping coefficient
} ViscoelasticCoulombModel;

typedef struct {
    double **trajectory;  // 2D array storing the trajectory data
    int index;           // Index for tracking progress in the trajectory
    int rows;
} Trajectory;


EXPORT_SYMBOL extern double FREQUENCY;
EXPORT_SYMBOL extern const char *filename_CSV;
EXPORT_SYMBOL extern Contact_Manager *cm;
EXPORT_SYMBOL extern HuntCrossleyHertz *hc_model;
EXPORT_SYMBOL extern ViscoelasticCoulombModel *vc_model;
EXPORT_SYMBOL extern Trajectory *traj;
EXPORT_SYMBOL extern double *voltages;
EXPORT_SYMBOL extern double *U_REF;

EXPORT_SYMBOL void init_voltages(void);
EXPORT_SYMBOL void init_U_REF(void);
EXPORT_SYMBOL void init_trajectory(void);
EXPORT_SYMBOL void init_contact_manager(int nSensors);
EXPORT_SYMBOL void init_viscoelastic_coulomb(double mu, double k, double d);
EXPORT_SYMBOL void init_hunt_crossley_hertz(double k, double n, double d);

EXPORT_SYMBOL void free_voltages(void);
EXPORT_SYMBOL void free_U_REF(void);
EXPORT_SYMBOL void free_trajectory(void);
EXPORT_SYMBOL void free_contact_manager(void);
EXPORT_SYMBOL void free_viscoelastic_coulomb(void);
EXPORT_SYMBOL void free_hunt_crossley_hertz(void);

#endif // STRUCTURES_H
