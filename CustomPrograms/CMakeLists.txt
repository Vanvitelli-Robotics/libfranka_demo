cmake_minimum_required(VERSION 3.4)


# Nome del progetto
project(CustomPrograms)

# Cpp standard 14 
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)

# Comando per far compilare anche le librerie contenute nella cartella CustomLibrary
add_subdirectory(CustomLibrary)

# Lista dei programmi da compilare: 
set(CUSTOMPROGRAMS

TorqueControlGeneric
Cartesian_Impedance_Control
Cartesian_Force_Impedance_Control
UtilizzoStateSaver

# Aggiungi qui il programma che vuoi compilare

)



# Link delle librerie: franka, Eigen e examples_comm. Il link viene richiesto per tutti 
# i programmi del set CUSTOMPROGRAMS perchè con elevata probabiltà serviranno a tutti. 

foreach(custom ${CUSTOMPROGRAMS})
  add_executable(${custom} ${custom}.cpp)
  # target_link_libraries cerca le librerie in tutte le sottodirectory di build 
  target_link_libraries(${custom} Franka::Franka examples_common StateSaver Eigen3::Eigen3)
  
endforeach()




