cmake_minimum_required(VERSION 3.4)


# Nome del progetto
project(CustomPrograms)


#list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR}/../cmake)
#find_package(Franka REQUIRED)
#find_package(Eigen3 REQUIRED)


# Cpp standard 14 
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# Lista dei programmi da compilare: 
set(CUSTOMPROGRAMS

EigenExamples
TorqueControlGeneric

# aggiungi qui programmi che vuoi compilare

)


#find_library(EXAMPLES_COMM examples_comm)
foreach(custom ${CUSTOMPROGRAMS})
  add_executable(${custom} ${custom}.cpp)
  
  target_link_libraries(${custom} Franka::Franka examples_common Eigen3::Eigen3)
  
endforeach()

