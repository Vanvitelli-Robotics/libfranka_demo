#pragma once
#include <iostream>
#include <thread>
#include <Eigen/Dense>



class StateSaver{

public: 

    static double** buffer_7;
    static double** buffer_6;

    // Dati di prova
        static std::array<double,7> tau;
    // Dati di dimensione 7
        static Eigen::Matrix<double,7,1> tau_measured;
        static Eigen::Matrix<double,7,1> q;
        static Eigen::Matrix<double, 7, 1> dq;
    // Dati di dimensione 6
        static Eigen::Matrix<double, 6, 1> error;
    

    static void fill_buffer();
    static void wait_filler();
    static void scrivi_su_file(int ncampioni);
    static void fill_buffer_7();
    static void fill_buffer_6();
    static void ciao();

};



