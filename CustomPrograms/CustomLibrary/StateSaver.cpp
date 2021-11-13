
#include <iostream>
#include <thread>
#include <Eigen/Dense>
#include <Eigen/Core>
#include "StateSaver.h"
#include <fstream>

std::thread s1;
std::thread s2;

void StateSaver::fill_buffer(){
        s1 = std::thread(StateSaver::fill_buffer_6);
        s2 = std::thread(StateSaver::fill_buffer_7);
}


void StateSaver::wait_filler(){
        s1.join();
        s2.join();
}

void StateSaver::scrivi_su_file(int ncampioni){
        char *path1 = "/home/giulio/Desktop/model_7.txt";
        char *path2 = "/home/giulio/Desktop/model_6.txt";
        std::ofstream File7(path1);
        std::ofstream File6(path2);

        for(int i = 0; i < (ncampioni*7); i++) {
                for(int j = 0; j < 1; j++){
                        File7 << buffer_7[i][j] << " ";
                }
                File7 << "\n";
        }
        File7.close();
                                              
                                              
        for(int i = 0; i < (ncampioni*6); i++) {
                for(int j = 0; j < 1; j++){
                        File6 << buffer_6[i][j] << " ";
                }
                File6 << "\n";
        }
        File6.close();
                                              
        
}

void StateSaver::fill_buffer_7(){

        static int x = 0;

         for(int i = 0; i < 7; i++) {
             buffer_7[x+i][0] = StateSaver::tau[i]; // 1st column occupied by tau
        } 
        x = x + 7;

}

void StateSaver::fill_buffer_6(){

        static int y = 0;

         for(int i = 0; i < 6; i++) {
             buffer_6[y+i][0] = error[i]; // 1st column occupied by error
             
        } 
        y = y + 6;

}
