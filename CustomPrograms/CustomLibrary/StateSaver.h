#pragma once
#include <iostream>
#include <thread>
#include <Eigen/Dense>


#define  lenght_buffer7 4
#define  lenght_buffer6 1

class StateSaver{
    /* Questa classe è stata pensata per bufferizzare lo stato del robot ad ogni ciclo e per salvare su file i dati alla fine dell'esecuzione del moto. 
    * Le variabili a 7 componenti e quelle a 6 componenti sono memorizzate su due file differenti. 
    * L'implementazione prevede che vengano memorizzati i seguenti vettori:
    * Lunghezza 7 : tau, q , qdot , pose (transl+quat)
    * Lunghezza 6: pose_error
    * Se si desidera aggiungere o eliminare variabili che vengono memorizzate è necessario modificare i metodi
    * della classe.
    */ 
public: 

    static double** buffer_7;
    static double** buffer_6;

    
    // Dati di dimensione 7
        static Eigen::Matrix<double,7,1> tau_measured;
        static Eigen::Matrix<double,7,1> q;
        static Eigen::Matrix<double, 7, 1> qdot;
        static Eigen::Matrix<double, 7, 1> pose; // posizione + quaternione

    // Dati di dimensione 6
        static Eigen::Matrix<double, 6, 1> error;
        

    static void fill_buffer();
    static void wait_filler();
    static void scrivi_su_file(int ncampioni);
    static void fill_buffer_7();
    static void fill_buffer_6();


};



