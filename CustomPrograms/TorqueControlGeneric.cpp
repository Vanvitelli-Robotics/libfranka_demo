// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

// Librerie standard
#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <vector>

// Libreria Eigen Dense
#include <Eigen/Dense>

// Librerie Libfranka
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

// Contiene definizioni di funzioni utili 
#include "../examples/examples_common.h"


/**
 * Programma di base C++ per il controllo in coppia del robot Panda. 
 *
 * Struttura del codice:
 * 1. Setup del robot
 * 2. Definizione dei parametri.
 * 3. Definizione del loop di controllo.
 * 4. Esecuzione del loop di controllo.
 * 
 * Nota: il programma non realizza nessun tipo di controllo ma può essere utilizzato come punto di partenza
 * per lo sviluppo di un algoritmo di controllo. Per avere un riscontro pratico si consiglia di
 * leggere il programma "Cartesian_Impedance_Control" nella cartella CustomPrograms così come
 * tutti gli esempi della cartella examples. 
 */


int main(int argc, char** argv) {

    if (argc != 2) {
        std::cerr << "Specificare l'indirizzo IP del robot." << std::endl;
        return -1;
    }

    try {

      // Connessione al robot 
        franka::Robot robot(argv[1]);
        
      // Set collision behavior (https://frankaemika.github.io/libfranka/classfranka_1_1Robot.html#a168e1214ac36d74ac64f894332b84534)
        setDefaultBehavior(robot);

      // Documentazione classe franka::Robot https://frankaemika.github.io/libfranka/classfranka_1_1Robot.html
      
      // Modello cinematico e dinamico del robot: 
        franka::Model model = robot.loadModel();

      // Lettura dello stato attuale del robot:
        franka::RobotState initial_state = robot.readOnce();

      // Documentazione franka::RobotState https://frankaemika.github.io/libfranka/structfranka_1_1RobotState.html
      
      // A partire da una variabile robot_state si possono ottenere diverse informazioni, tra cui:
      // initial_state.O_T_EE è la matrice di trasformazione omogenea che descrive la posa 
      // dell'organo terminale rispetto alla terna zero del robot. 
      
        Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

      // Estrazione della posizione dalla matrice di trasformazione
        Eigen::Vector3d position_d(initial_transform.translation());

      // Estrazione dell'orientamento in forma di quaternione a partire dalla matrice di rotazione
        Eigen::Quaterniond orientation_d(initial_transform.linear());

      // Inizializzazione variabile time
        double time = 0.0;

      // Definizione della callback per il loop di controllo in coppia:
      // L'esempio definisce una lambda function ma nulla vieta di creare una funzione esterna. 
      
        auto generic_callback = [&](const franka::RobotState& robot_state,
                                          franka::Duration duration) -> franka::Torques {

      /** Inizio callback: durante il loop di controllo la callback viene eseguita con una frequenza
       * di 1Khz. La variabile robot_state viene aggiornata ogni loop con la funzione robot.readOnce() **/
      
      // Update time.
        time += duration.toSec();
      
      // Calcolo coppie di controllo di controllo
        Eigen::VectorXd tau_task(7), tau_d(7);
        std::array<double, 7> tau_d_array{};
        Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;


        return tau_d_array;
        };

      // Avvio del loop di controllo 
        robot.control(generic_callback);

      } catch (const franka::Exception& ex) {
          // print exception
          std::cout << ex.what() << std::endl;
      }

      return 0;
}

