

#include <iostream>
#include "CustomLibrary/StateSaver.h"
#include <Eigen/Dense>

const int ncampioni = 1; 

double** StateSaver::buffer_7 = nullptr;
double** StateSaver::buffer_6 = nullptr;

Eigen::Matrix<double,7,1> StateSaver::tau_measured;
Eigen::Matrix<double,7,1> StateSaver::q;
Eigen::Matrix<double, 7, 1> StateSaver::qdot;
Eigen::Matrix<double, 7, 1> StateSaver::pose;
Eigen::Matrix<double, 6, 1> StateSaver::error;

int main(){




    StateSaver::buffer_7 = new double*[ncampioni * 7];
    StateSaver::buffer_6 = new double*[ncampioni * 6];

for(int i = 0; i< ncampioni*7; i++)
    StateSaver::buffer_7[i] = new double[4]; // tau_m, q, dq, pose

for(int i = 0; i< ncampioni*6; i++)
    StateSaver::buffer_6[i] = new double[1]; // error



StateSaver sv;

Eigen::Matrix<double,6,1> error;
error << 1,1,1,1,1,1;
sv.error = error;

  

Eigen::Matrix<double,7,1> tau_m;
tau_m << 3,1,3,3,2,2,2;
sv.tau_measured = tau_m;

Eigen::Matrix<double,7,1> q;
q << 1,4,4,4,33,3,3;
sv.q = q;
    
Eigen::Matrix<double,7,1> qdot;
qdot << 1,2,3,4,5,5,5;
sv.qdot = qdot;

Eigen::Matrix<double,7,1> pose;
pose << 2,2,2,2,3,4,6;
sv.pose = pose; 
    
   
 

    // Si lanciano due thread che bufferizzano i dati raccolti
        sv.fill_buffer();
    // Si attende che i thread terminino la scrittura nei buffer
        sv.wait_filler();

    // Scrittura su file
    sv.scrivi_su_file(ncampioni);
}