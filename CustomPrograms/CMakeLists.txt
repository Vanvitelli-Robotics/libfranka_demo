cmake_minimum_required(VERSION 3.4)


# Nome del progetto
project(CustomPrograms)

# Cpp standard 14 
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Lista dei programmi da compilare: 
set(CUSTOMPROGRAMS

EigenExamples
TorqueControlGeneric

# aggiungi qui programmi che vuoi compilare

)



foreach(custom ${CUSTOMPROGRAMS})
  add_executable(${custom} ${custom}.cpp)
  # target_link_libraries cerca le librerie a partire dalla directory build 
  target_link_libraries(${custom} Franka::Franka examples_common Eigen3::Eigen3)
  
endforeach()

