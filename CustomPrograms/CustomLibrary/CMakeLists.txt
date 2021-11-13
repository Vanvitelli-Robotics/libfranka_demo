
cmake_minimum_required(VERSION 3.4)


# Nome del progetto
project(CustomLibrary)

# Cpp standard 14 
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

## Per creare una libreria statica:

# add_library(NomeLibreriaStatica STATIC 
## Inserire i nomi dei file Cpp che costruiscono la liberia  
# File1.cpp
# File2.cpp
  
# )


add_library(StateSaver STATIC 
StateSaver.cpp
# la libreria libFillerBuffer.a viene creata nel path Panda_lab/build/CustomPorgrams/CustomLibrary
)

target_link_libraries(StateSaver PUBLIC Eigen3::Eigen3 Threads::Threads)



