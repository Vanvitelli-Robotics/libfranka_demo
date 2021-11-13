

#include <iostream>
#include <Eigen/Dense>
#include <array>

#include "CustomLibrary/StateSaver.h"


double** StateSaver::buffer_6;
double** StateSaver::buffer_7; 
std::array<double,7> StateSaver::tau;
Eigen::Matrix<double, 6, 1> StateSaver::error;
    
int main()
{
  /*Eigen::MatrixXd matrix(2,2);
  matrix(0,0) = 3;
  matrix(1,0) = 2.5;
  matrix(0,1) = -1;
  matrix(1,1) = matrix(1,0) + matrix(0,1);
  //std::cout << matrix << std::endl;

  Eigen::Quaterniond quaternion(1,2,3,4);
  quaternion.normalize(); 
  Eigen::Vector4d quaternioncoeff = quaternion.coeffs();
  std::array<double,7> pose;

  Eigen::Vector3d position(1,2,3);*/


  /* prova classe StateSaver */ 
  
  extern std::thread s1;
  extern std::thread s2;

  StateSaver sv;

  sv.buffer_7 = new double*[7*1000];
  for(int i = 0; i< 1000*7; i++)
      sv.buffer_7[i] = new double[1];
  
  

  sv.buffer_6 = new double*[6*1000];
  for(int i = 0; i< 1000*6; i++)
      sv.buffer_6[i] = new double[1];

  
  
  int x = 0;
  std::array<double,7> tau{1,1,1,1,1,1,1};
  Eigen::Matrix<double, 6, 1> error;
  error << 1,1,1,1,1,1;

  sv.tau = tau;
  sv.error = error;

  while(x<1000)
  {

  for(int i = 0;i < 7; i++)
        sv.tau[i] = sv.tau[i]+1;

  for(int i = 0;i < 6; i++)
        sv.error[i] = sv.error[i]+1;
  
  x++;
    
    StateSaver::fill_buffer();
    s1.join();
    s2.join();

    //sv.wait_filler();
    


  }

  sv.scrivi_su_file(1000);
  


}