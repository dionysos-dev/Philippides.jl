#include "structures.h"
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include "math.h" 

// Contact structures
Contact_Manager *cm = NULL;
HuntCrossleyHertz *hc_model = NULL;
ViscoelasticCoulombModel *vc_model = NULL;

// Reference trajectory
Trajectory *traj = NULL;

// Voltages applied on the motors
double *voltages = NULL;

void init_voltages(void) {
    voltages = malloc(sizeof(double)*4);
    if (!voltages) {
        fprintf(stderr, "Failed to allocate voltages\n");
        exit(EXIT_FAILURE);
    }
}

void free_voltages(void) {
    free(voltages);
}

int count_lines(const char *filename) {

    FILE *file = fopen(filename, "r");
    if (file == NULL) {
        perror("Error opening file");
        return -1;  
    }

    int count = 0;
    char ch;

    while ((ch = fgetc(file)) != EOF) {
        if (ch == '\n') {
            count++;
        }
    }

    fclose(file);

    // If file isn't empty, add 1 (last line may not have '\n')
    return count > 0 ? count + 1 : 0;
}

// Function to initialize trajectory data by reading from a CSV file
void fill_trajectory(Trajectory *traj, const char *filename, int rows, int cols) {
    traj->rows = rows;
    traj->index = 0;
    FILE *file = fopen(filename, "r"); // Open the file for reading
    if (!file) {
        perror("Error opening file");
    }
    char buffer[1024];
    fgets(buffer, sizeof(buffer), file); // Skip the header line
    
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            fscanf(file, "%lf,", &traj->trajectory[i][j]); // Read values from file
        }
    }
    
    fclose(file); // Close the file
}

void init_trajectory(void) {
    int rows = count_lines(filename_CSV);
    traj = (Trajectory*)malloc(sizeof(Trajectory)); // Allocate memory for trajectory structure
    traj->trajectory = (double**)malloc(rows * sizeof(double*)); // Allocate memory for rows
    for (int i = 0; i < rows; i++) {
        traj->trajectory[i] = (double*)malloc(5 * sizeof(double)); // Allocate memory for columns
    }
    fill_trajectory(traj, filename_CSV, rows, 5); //There are 5 rows (time, 4 motors)
}

void free_trajectory(void) {
    int rows = count_lines(filename_CSV);
    for(int i = 0; i < rows; i++){
        free(traj->trajectory[i]);
    }
    free(traj->trajectory);
    free(traj);
}

// Function to create and initialize the Contact_Manager structure
void init_contact_manager(int nSensors) {
    cm = (Contact_Manager *) malloc(sizeof(Contact_Manager));
    cm->nSensors = nSensors;
    
    // Allocate memory for Contact_PxF and Previous_PxF
    cm->Contact_PxF = (double **)malloc(nSensors * sizeof(double *));
    cm->Previous_PxF = (double **)malloc(nSensors * sizeof(double *));
    
    for (int i = 0; i < nSensors; i++) {
        cm->Contact_PxF[i] = (double *)malloc(4 * sizeof(double)); // 4 values per sensor
        cm->Previous_PxF[i] = (double *)malloc(4 * sizeof(double)); // 4 values per sensor
    }
    
    // Allocate memory for InContact and results
    cm->InContact = (int *)malloc(nSensors * sizeof(int));
    cm->results = (double *)malloc(4 * nSensors * sizeof(double)); // 4 * nSensors results
    
    // Initialize values to NaN or zero
    for (int i = 0; i < nSensors; i++) {
        for (int j = 0; j < 4; j++) {
            cm->Contact_PxF[i][j] = NAN;
            cm->Previous_PxF[i][j] = NAN;
        }
        cm->InContact[i] = 0;
    }
    
    for (int i = 0; i < 4 * nSensors; i++) {
        cm->results[i] = 0.0;
    }

}

void free_contact_manager(void) {
    for (int i = 0; i < cm->nSensors; i++) {
        free(cm->Contact_PxF[i]);
        free(cm->Previous_PxF[i]);
    }

    free(cm->Contact_PxF);
    free(cm->Previous_PxF);
    free(cm->InContact);
    free(cm->results);
    free(cm);
}


// Function to initialize the Viscoelastic Coulomb model
void init_viscoelastic_coulomb(double mu, double k, double d) {
    vc_model = (ViscoelasticCoulombModel*) malloc(sizeof(ViscoelasticCoulombModel));
    vc_model->mu = mu;
    vc_model->k = k;
    vc_model->d = d;
}

void free_viscoelastic_coulomb(void) {
    free(vc_model);
}

// Function to initialize the Hunt-Crossley Hertz model
void init_hunt_crossley_hertz(double k, double n, double d) {
    hc_model = (HuntCrossleyHertz*) malloc(sizeof(HuntCrossleyHertz));
    hc_model->k = k;
    hc_model->n = n;
    hc_model->d = d;
}

void free_hunt_crossley_hertz(void) {
    free(hc_model);
}