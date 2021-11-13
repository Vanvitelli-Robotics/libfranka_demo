

#include <iostream>
#include <Eigen/Dense>
#include <array>


int main()
{
  Eigen::MatrixXd matrix(3,3);
  matrix(0,0) = 3;
  matrix(1,0) = 2.5;
  matrix(0,1) = -1;
  matrix(1,1) = matrix(1,0) + matrix(0,1);
  std::cout << matrix << std::endl;

  
  


  

    



}