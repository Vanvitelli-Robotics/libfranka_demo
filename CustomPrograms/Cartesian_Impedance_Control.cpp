// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

// Librerie standard
#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <vector>
#include <thread>

// Libreria Eigen Dense
#include <Eigen/Dense>

// Librerie Libfranka
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

// Contiene definizioni di funzioni utili 
#include "../examples/examples_common.h"

// Header file con le dichiarazioni delle funzioni custom
#include "CustomLibrary/StateSaver.h"

const int t_fin = 20; // 20 s
const int fs = 1000; // 1 KHz
const int ncampioni = t_fin*fs; 



/**
 * Esempio programma C++ per il controllo in coppia del robot Panda. 
 * Tale esempio è una modifica del file examples/cartesian_impedance_control.cpp.
 * E' stato realizzato un controllo di tipo PD con compensazione di gravità in cui la posa 
 * desiderata è pari alla posa iniziale del robot.
 * 
 * Nota: l'orientamento dell'EndEffector rispetto alla terna base è espresso attraverso quaternione unitario.
 * Nota: il Panda Robot compensa da se le coppie gravitazionali e gli attriti quindi non è 
 * necessario compensare le coppie gravitazionali. Inoltre, l'esempio prevede la compensazione 
 * delle coppie di Coriolis e Centrifughe. 
 * 
 *
 * 
 * Struttura del codice:
 * 1. Setup del robot
 * 2. Definizione dei parametri.
 * 3. Definizione del loop di controllo.
 * 4. Esecuzione del loop di controllo.
 * 
 * Osservazione: il loop di controllo DEVE essere eseguito ad una frequenza di 1 Khz.
 * Per tale motivo la scrittura su file viene rimandata alla fine del loop di controllo.
 * 
 */

// Funzione per compattare posizione e quaternione sotto un unico vettore posa 7x1
Eigen::Matrix<double,7,1> posquat2mat(const Eigen::Vector3d& position, const Eigen::Quaterniond quaternion){
    
    Eigen::Matrix<double,7,1> pose;
    pose(0,0) = position(0);
    pose(1,0) = position(1);
    pose(2,0) = position(2);
    pose(3,0) = quaternion.w();
    pose(4,0) = quaternion.x();
    pose(5,0) = quaternion.y();
    pose(6,0) = quaternion.z();
    
    return pose;
}


// Per utilizzare le variabili static della classe StateSaver serve istanziarle fuori dal main
  double** StateSaver::buffer_6 = nullptr;
  double** StateSaver::buffer_7 = nullptr; 

  Eigen::Matrix<double, 6, 1> StateSaver::error;
  Eigen::Matrix<double,7,1> StateSaver::tau_measured;
  Eigen::Matrix<double,7,1> StateSaver::q;
  Eigen::Matrix<double, 7, 1> StateSaver::qdot;
  Eigen::Matrix<double, 7, 1> StateSaver::pose;


int main(int argc, char** argv) {

  
    if (argc != 2) {
        std::cerr << "Specificare l'indirizzo IP del robot." << std::endl;
        return -1;
    }

    

  // Controllo di cedevolezza attiva (PD + compensazione di gravità)
    const double rigidezza_translazionale{150.0};
    const double rigidezza_torsionale{10.0};
    Eigen::MatrixXd rigidezza(6, 6), smorzamento(6, 6);

  // Costruzione di Kp = [K_t O ; O K_o] = rigidezza
    rigidezza.setZero();
    rigidezza.topLeftCorner(3, 3) << rigidezza_translazionale * Eigen::MatrixXd::Identity(3, 3);
    rigidezza.bottomRightCorner(3, 3) << rigidezza_torsionale * Eigen::MatrixXd::Identity(3, 3);
  
  // Costruzione di Kd = [K_d_t O ; O K_d_o] = smorzamento
    smorzamento.setZero();
    smorzamento.topLeftCorner(3, 3) << 2.0 * sqrt(rigidezza_translazionale) *
                                        Eigen::MatrixXd::Identity(3, 3);
    smorzamento.bottomRightCorner(3, 3) << 2.0 * sqrt(rigidezza_torsionale) *
                                            Eigen::MatrixXd::Identity(3, 3);

    try {

      // Connessione al robot (inizializzazione variabile robot)
        franka::Robot robot(argv[1]);
      

      // Inizializzazione oggetto StateSaver per salvare su file
        StateSaver::buffer_7 = new double*[ncampioni* 7];
        StateSaver::buffer_6 = new double*[ncampioni * 6];

        for(int i = 0; i< ncampioni * 7; i++)
            StateSaver::buffer_7[i] = new double[lenght_buffer7]; // tau_m, q, dq, pose, pose_dot
        
        for(int i = 0; i< ncampioni * 6; i++)
            StateSaver::buffer_6[i] = new double[lenght_buffer6]; // error

        StateSaver sv;

      // Set collision behavior (https://frankaemika.github.io/libfranka/classfranka_1_1Robot.html#a168e1214ac36d74ac64f894332b84534)
        robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});    
        robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
        robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});

      // Modello cinematico e dinamico del robot
        franka::Model model = robot.loadModel();

      // Lettura dello stato attuale del robot
        franka::RobotState initial_state = robot.readOnce();
      
        Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

      // Estrazione della posizione dalla matrice di trasformazione
        Eigen::Vector3d position_d(initial_transform.translation());

      // Estrazione dell'orientamento in forma di quaternione a partire dalla matrice di rotazione
        Eigen::Quaterniond orientation_d(initial_transform.linear());


      // Variabile per tenere traccia del tempo trascorso

        double time = 0.0;

      // Definizione della callback per il loop di controllo in coppia:
        
        std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        impedance_control_callback = [&](const franka::RobotState& robot_state,
                                          franka::Duration duration) -> franka::Torques {

      
      // Update time.
        time += duration.toSec();
        
      // Calcolo delle coppie di Coriolis a partire dal modello dinamico e dallo stato del robot
        std::array<double, 7> coriolis_array = model.coriolis(robot_state);

      // Jacobiano geometrico del manipolatore
        std::array<double, 42> jacobian_array = 
        model.zeroJacobian(franka::Frame::kEndEffector, robot_state);

      // Conversione a Eigen Map
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
        Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());

      // Estrazione di posizione e orientamento attuali  
        Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
        Eigen::Vector3d position(transform.translation());
        Eigen::Quaterniond orientation(transform.linear());
        
      // Calcolo errore in posizione:
        Eigen::Matrix<double, 6, 1> error;
        error.head(3) << position - position_d;

      
      // Calcolo errore in orientamento:
        if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
            orientation.coeffs() << -orientation.coeffs();
        }

        Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
        error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();

        error.tail(3) << -transform.linear() * error.tail(3);

      // Costruzione delle coppie di controllo
        Eigen::VectorXd tau_task(7), tau_d(7);

      // L'errore è stato definito con il segno opposto quindi serve invertire i segni.
        tau_task << jacobian.transpose() * (-rigidezza * error - smorzamento * (jacobian * dq));
        
        
      // Salvataggio dello stato del robot attuale
      
        sv.tau_measured = (Eigen::Matrix<double, 7, 1>) robot_state.tau_J.data();
        sv.q = q;
        sv.qdot = dq;
        sv.pose = posquat2mat(position,orientation);
        sv.error = error;
        
        
        
        

      // Si lanciano due thread che bufferizzano i dati raccolti
        sv.fill_buffer();
        
      // Compensazione delle coppie di Coriolis
        tau_d << tau_task + coriolis;

        std::array<double, 7> tau_d_array{};
        Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

      // Si attende che i thread terminino la scrittura nei buffer
        sv.wait_filler();
        
        if (time >= t_fin) {
          franka::Torques tau_m = tau_d_array;
          std::cout << std::endl << "Fine controllo di impedenza" << std::endl;
          return franka::MotionFinished(tau_m);
        }      
        return tau_d_array;

        };

      
      // Start del loop di controllo real-time
        robot.control(impedance_control_callback);


      // Scrittura su file
         sv.scrivi_su_file(ncampioni);

      // Free memory
        for(int i = 0; i< ncampioni*7; i++)
            delete[] StateSaver::buffer_7[i];
        delete[] StateSaver::buffer_7;

        for(int i = 0; i< ncampioni*6; i++)
            delete[] StateSaver::buffer_6[i];
        delete[] StateSaver::buffer_6;

      } catch (const franka::Exception& ex) {
          // print exception
          std::cout << ex.what() << std::endl;
      }


      


      return 0;
}
