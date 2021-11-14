// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <iostream>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/gripper.h>
#include <franka/model.h>
#include <thread>
#include <fstream>
#include "../examples/examples_common.h"

/**
 *
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 */


//Franka Gripper parameters
franka::Gripper gripper("10.224.20.198");
franka::GripperState gripper_state = gripper.readOnce();
double max_width = gripper_state.max_width;
double grasping_width = 0.011; //[m]
double force_grasping = 60.0; //[N]

//Lissajous figure parameters
const double Ax = 0.10; //[m]
const double Ay = 0.05; //[m]
const double px = M_PI_2;
const double py = 0.0;
const double wx = 1.0;
const double wy = 2.0;
const double n = 4.0; //Lissajous figure lap number
const double t_traj = 20.0; //[s] trajectory duration for a singol lap
const double t_fin = t_traj*n; //[s] total trajectory duration

const double tau = 0.5; //[s] second-order system time constant used beyond

//Parameters useful for recording informations during control-loop
const int fs = 1000; //[Hz] sampling rate
const int tf = t_fin + 0.5; // equivalente a tf = ceil(t_fin) (0.5 assicura l'approssimazione per eccesso)
const int n_rows_1 = 7; //vectors rows number whic will be saved during motion
const int n_element_1 = 2; //number of vectors to save 
const int n_rows_2 = 6;
const int n_element_2 = 5; 
double buffer_1 [(fs*tf*n_rows_1)+1][n_element_1]; //fs*tf*n_rows_1+1 samples to save in 2 columns (tau_m=1, q=1)
double buffer_2 [(fs*tf*n_rows_2)+1][n_element_2]; //fs*tf*n_rows_2+1 samples to save in 5 columns (pose=1, pose_dot=1, external_wrench, desired_pose, desired_pose_dot)
int x = 0.0;
int y = 0.0;

//Then acquired informations are written in .txt file OUTSIDE the control-loop
const char *path1 = "Force_Impedance_RobotState/model_7.txt";
const char *path2 = "Force_Impedance_RobotState/model_6.txt";
std::ofstream MyFile_1(path1);
std::ofstream MyFile_2(path2);


void fill_buffer1(const std::array<double, 7>& tau_m, const std::array<double, 7>& q){ //arrays passed as const because RobotState is passed as const in robot.control

   for(int i = 0; i < n_rows_1; i++) {


             buffer_1[x+i][0] = tau_m[i]; //1st column occupied by tau_m
             buffer_1[x+i][1] = q[i];     //2nd column occupied by q

                                   } 

  }



void fill_buffer2(const std::array<double, 6>& pose, const Eigen::Matrix<double, 6, 1>& posedot, const Eigen::Matrix<double, 6, 1>& extwrench, const Eigen::Matrix<double, 6, 1>& desired_pose, const Eigen::Matrix<double, 6, 1>& desired_pose_dot){ 

   for(int k = 0; k < n_rows_2; k++) {

             buffer_2[y+k][0] = pose[k]; //1st column occupied by pose
             buffer_2[y+k][1] = posedot(k); //2nd column occupied by posedot
             buffer_2[y+k][2] = extwrench(k); //3rd column occupied by extwrench
             buffer_2[y+k][3] = desired_pose[k]; //4th column occupied by desired_pose
             buffer_2[y+k][4] = desired_pose_dot[k]; //5th column occupied by desired_pose_dot
              
                                     } 


                   }



void write_to_file_1(std::ofstream& File, int fs, int tf, int n_rows, int n_element){

     for(int i = 0; i < (fs*tf*n_rows)+1; i++) {
 
       for(int j = 0; j < n_element; j++){
        
         File << buffer_1[i][j] << " ";
       
                                           }

                  File << "\n";
         
                                         }

        File.close();
                                              }



void write_to_file_2(std::ofstream& File, int fs, int tf, int n_rows, int n_element){


     for(int i = 0; i < (fs*tf*n_rows)+1; i++) {
 
       for(int j = 0; j < n_element; j++){
        
         File << buffer_2[i][j] << " ";
       
                                            }

                  File << "\n";
         
                                             }

        File.close();
  
                                                                                       }                                                                                     


double compute_lissajous_figure_length(const double n, const double Ax, const double Ay, const double wx, const double wy){


   int N = n*100; //sampling points number
   double th[N]; //vector of sampled integration interval (from 0 to N-1 with a 2*pi/100 step size)
   th[0] = 0.0;

   for(int i=1; i<N; i++){

        th[i] = th[i-1] + (2*M_PI*n)/(N); 

                         }

   double len = 0.0; //total length Lissajous figure

   for(int k=0; k<N-1; k++){
 
        len = len + sqrt( pow(Ax*wx*std::cos(wx*th[k]+M_PI/2),2)+pow(Ay*wy*std::cos(wy*th[k]),2) )*2*M_PI/100;

        }

    return len;

                                                                                                                         }


Eigen::Matrix<double, 6, 1> trajectory_planner(double n, double s0, double sf, double t, double t_fin, double Ax, double Ay, double wx, double wy, Eigen::Vector3d c, Eigen::Matrix<double, 3, 3> R){

   double s = 0.0;
   double sd = 0.0;

   if( t <= t_fin){

   s = s0 + (sf-s0)*(6*pow((t/t_fin),5) -15*pow((t/t_fin),4) + 10*pow((t/t_fin),3));

   sd = (sf-s0)*1/t_fin*(30*pow((t/t_fin),4) - 60*pow((t/t_fin),3) + 30*pow((t/t_fin),2));
   
                 }
   else{

   s = sf;

   sd = 0.0;

       }


   Eigen::VectorXd p_s(3); //point of the Lissajous figure in the end-effector frame
   p_s << Ax*std::sin(wx*2*M_PI*s/(sf/n)+M_PI/2), Ay*std::sin(wy*2*M_PI*s/(sf/n)), 0;
   p_s << (-t*exp(-t/tau)*1/tau - exp(-t/tau) +1)*p_s(0), (-t*exp(-t/tau)*1/tau - exp(-t/tau) +1)*p_s(1), 0;   //x and y components multiplied by 2� order step-response system in orderto avoid
                                                                                                               //discontinuities in t=0 (pd(0)=p0)
                                                                                                                                                                     

   Eigen::VectorXd p(3); //point of the Lissajous figure in base frame
   p = c + R*p_s;

   Eigen::VectorXd p_s_dot(3); //derivative of p_s w.r.t. variable s (without considering 2� order step-response system)
   p_s_dot << (Ax*wx*2*M_PI/(sf/n))*std::cos(wx*2*M_PI*s/(sf/n)+M_PI/2), (Ay*wy*2*M_PI/(sf/n))*std::cos(wy*2*M_PI*s/(sf/n)), 0;

   Eigen::VectorXd tan(3); //tangent versor
   tan = R * p_s_dot;
  
   Eigen::VectorXd pd(3); //derivative of p w.r.t. time
   pd = sd * tan;

   //Since p_s is multiplied byt 2� order step-response system, to have that pd is really d/dt(p) we have to modify pd as follow:
   pd << (t*exp(-t/tau)*1/pow((tau),2))*Ax*std::sin(wx*2*M_PI*s/(sf/n)+M_PI/2) + (-t*exp(-t/tau)*1/tau - exp(-t/tau) +1)*pd(0), (t*exp(-t/tau)*1/pow((tau),2))*Ay*std::sin(wy*2*M_PI*s/(sf/n)) + (-t*exp(-t/tau)*1/tau - exp(-t/tau) +1)*pd(1), 0;

   Eigen::VectorXd p_pd(6);
   p_pd << p, pd;

    return p_pd;
		}



int main(int argc, char** argv) {

  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
                 }

   std::array<double, 6> h_offset_array = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //offset wrench_vector to be subtracted from O_F_ext_hat_K afterward
   Eigen::Map<Eigen::Matrix<double, 6, 1>> h_offset(h_offset_array.data());

  try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

   /*
    gripper.homing();

    if (!gripper.grasp(grasping_width, 0.05, force_grasping)) {
      std::cout << "Failed to grasp object." << std::endl;
      return -1;
    }  */

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.


    robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
    

    //First grasp the holder
    std::array<double, 7> q_goal_grasp = {{0.180728,0.742286,-0.00850259,-1.79291,-0.01306,2.59829,-2.13941}};
    MotionGenerator motion_generator_grasp(0.2, q_goal_grasp);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator_grasp);
    std::cout << "Finished moving to initial joint configuration." << std::endl;

     if (!gripper.grasp(grasping_width, 0.05, force_grasping)) {
      std::cout << "Failed to grasp object." << std::endl;
      return -1;
    }







    // Move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{0.4786,0.6018,0.0162,-1.9798,-0.0173,2.5815,-1.8495}}; //0.4786,0.6018,0.0162,-1.9798,-0.0173,2.5815,-1.8495 --- 0.0588, -0.1200, -0.0415, -2.2067, -0.0057, 2.0869, -2.3357 --- 0.2485, 0.2768, 0.0529, -2.4831, -0.0388, 2.7593, -2.0208 from inverse kinematics to have alignment with rotated end-effector frame and base frame 
    MotionGenerator motion_generator(0.2, q_goal);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;


    double time_2 = 0.0;

    franka::Torques zero_torques{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    robot.control(
        [&time_2, &h_offset, zero_torques](
            const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques {
         
          //Update time
          time_2 += period.toSec();

          if (time_2 == 0.0){
            return zero_torques;
                            }

          if (time_2 >= 4){

          Eigen::Map<const Eigen::Matrix<double, 6, 1>> h_ext(robot_state.O_F_ext_hat_K.data());
          h_offset = h_offset + h_ext;

                            }

          if (time_2 >= 4.5) {
            std::cout << std::endl << "Finished computing h_ext_offset, job done!" << std::endl;
            return franka::MotionFinished(zero_torques);
                              }

          // Sending zero torques - if EE is configured correctly, robot should not move
          return zero_torques;
        },
        false, 1000);
  
        h_offset = h_offset/500;


    Eigen::Matrix<double, 4, 4> Td0_init; //Initial homogeneous transformation matrix which describes DESIRED end-effector frame position and orientation w.r.t. zero frame
    Eigen::Matrix<double, 4, 4> Td0; //Homogeneous transformation matrix which describes DESIRED end-effector frame position and orientation w.r.t. zero frame
    

    std::array<double, 6> external_wrench_array;
    std::array<double, 7> dq_array;
    Eigen::VectorXd wrench_error_integral(6);
    std::array<double, 36> Kp_array = {1000.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                       0.0, 1000.0, 0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 500.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 1000.0/20.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0, 1000.0/20.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0, 0.0, 1000.0/20.0};

    Eigen::Map<const Eigen::Matrix<double, 6, 6>> Kp(Kp_array.data());  //Eigen matrix saved in column-major format


    std::array<double, 36> Kpf_array = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 2500.0/5000.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

   Eigen::Map<const Eigen::Matrix<double, 6, 6>> Kpf(Kpf_array.data());

   std::array<double, 36> Ki_array = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                      0.0, 0.0, 5000.0/5000.0, 0.0, 0.0, 0.0,
                                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

   Eigen::Map<const Eigen::Matrix<double, 6, 6>> Ki(Ki_array.data());


   std::array<double, 36> Dx_array = {sqrt(1000.0), 0.0, 0.0, 0.0, 0.0, 0.0,
                                      0.0, sqrt(1000.0), 0.0, 0.0, 0.0, 0.0,
                                      0.0, 0.0, sqrt(500.0), 0.0, 0.0, 0.0,
                                      0.0, 0.0, 0.0, sqrt(1000.0/20.0), 0.0, 0.0,
                                      0.0, 0.0, 0.0, 0.0, sqrt(1000.0/20.0), 0.0,
                                      0.0, 0.0, 0.0, 0.0, 0.0, sqrt(1000.0/20.0)};

   Eigen::Map<const Eigen::Matrix<double, 6, 6>> Dx(Dx_array.data());


   std::array<double, 49> Kd_array = {sqrt(1000.0), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                      0.0, sqrt(1000.0), 0.0, 0.0, 0.0, 0.0, 0.0,
                                      0.0, 0.0, sqrt(1000.0), 0.0, 0.0, 0.0, 0.0,
                                      0.0, 0.0, 0.0, sqrt(1000.0), 0.0, 0.0, 0.0,
                                      0.0, 0.0, 0.0, 0.0, sqrt(1000.0), 0.0, 0.0,
                                      0.0, 0.0, 0.0, 0.0, 0.0, sqrt(1000.0), 0.0,
                                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, sqrt(1000.0)};

   Eigen::Map<const Eigen::Matrix<double, 7, 7>> Kd(Kd_array.data());


   std::array<double, 16> Tde_array = {-1.0, 0.0, 0.0, 0.0,  //Homogeneous transformation matrix to turn O_T_EE of 180� respect to y-axis of current frame
                                       0.0, 1.0, 0.0, 0.0,
                                       0.0, 0.0, -1.0, 0.0,
                                       0.0, 0.0, 0.0, 1.0};

    Eigen::Map<const Eigen::Matrix<double, 4, 4>> Tde(Tde_array.data());

    double length_figure = compute_lissajous_figure_length(n, Ax, Ay, wx, wy); //total length Lissajous figure

    double time = 0.0;
    double delta_t = 0.0;


    // Load the kinematics and dynamics model.
    franka::Model model = robot.loadModel();

    // Define callback for the joint torque control loop.
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        impedance_force_control_callback =
            [&time, &delta_t, &Tde, &Td0_init, &Td0, &model, &wrench_error_integral, &Kp, &Kpf, &Ki, &Dx, &Kd, &external_wrench_array, &dq_array, &length_figure, &h_offset](
                const franka::RobotState& state, franka::Duration period) -> franka::Torques{

      // Update time.
      time += period.toSec();
      delta_t = period.toSec();
      
      //Initial homogeneous transformation matrix
      if (time == 0.0) {
        Eigen::Map<const Eigen::Matrix<double, 4, 4>> init_pose(state.O_T_EE_c.data());
        Td0_init = init_pose*Tde;       
      }

      //Inital position and rotational matrix
      std::array<double, 3> p0_array = {Td0_init(12), Td0_init(13), Td0_init(14)};
      Eigen::Map<const Eigen::Vector3d> p0(p0_array.data());
      std::array<double, 9> R0_array = {Td0_init(0), Td0_init(1), Td0_init(2), Td0_init(4), Td0_init(5), Td0_init(6), Td0_init(8), Td0_init(9) , Td0_init(10)};
      Eigen::Map<const Eigen::Matrix<double, 3, 3>> R0(R0_array.data());
      
      //Initial minimal representation of end effector orientation (RPY angles - XYZ current frame)
      double phi0 = atan2(-Td0_init(9), Td0_init(10));
      double theta0 = atan2(Td0_init(8), sqrt(pow(Td0_init(9),2)+pow(Td0_init(10),2)));
      double psi0 = atan2(-Td0_init(4), Td0_init(0));
      Eigen::VectorXd orient0(3);
      orient0 << phi0, theta0, psi0;


      //Current minimal representation of end effector orientation (RPY angles - XYZ current frame)
      Eigen::Map<const Eigen::Matrix<double, 4, 4>> curr_pose(state.O_T_EE.data());
      Td0 = curr_pose*Tde;
      double phi = atan2(-Td0(9), Td0(10));
      double theta = atan2(Td0(8), sqrt(pow(Td0(9),2)+pow(Td0(10),2)));
      double psi = atan2(-Td0(4), Td0(0));
      Eigen::VectorXd orient(3);
      orient << phi, theta, psi;
      

      //Calculation 6x6 matrix which maps geometric jacobian to analytic jacobian (inv(TA_PHI))
      std::array<double, 36> invT_array  = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, (std::sin(phi)*std::sin(theta))/(std::cos(theta)), std::cos(phi), -std::sin(phi)/(std::cos(theta)),
                                            0.0, 0.0, 0.0, -(std::cos(phi)*std::sin(theta))/(std::cos(theta)), std::sin(phi), std::cos(phi)/(std::cos(theta))};

      Eigen::Map<const Eigen::Matrix<double, 6, 6>> invT(invT_array.data());
      

      std::array<double, 6> pose_array = {Td0(12), Td0(13), Td0(14), phi, theta, psi};
      Eigen::Map<Eigen::Matrix<double, 6, 1>> pose(pose_array.data());

      //Trajectory planner
      Eigen::Matrix<double, 6, 1> pose_posed_array = trajectory_planner(n, 0, length_figure, time, t_fin, Ax, Ay, wx, wy, p0, R0); //return a column vector 6x1 with pd(t) and pd_dot(t)

      Eigen::Matrix<double, 6, 1> desired_pose;
      desired_pose << pose_posed_array[0], pose_posed_array[1], pose_posed_array[2], orient0;
      //desired_pose << p0(0), p0(1), p0(2), orient0;
      Eigen::Matrix<double, 6, 1> desired_pose_dot;
      desired_pose_dot << pose_posed_array[3], pose_posed_array[4], pose_posed_array[5], 0.0, 0.0, 0.0;
      //desired_pose_dot << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

      Eigen::Matrix<double, 6, 1> pose_tilde;
      pose_tilde = desired_pose - pose;

      double force_z = (-time*exp(-time/tau)*1/tau - exp(-time/tau) +1)*-7.0; //[N] desired value multiplied by a 2� order system step-response

      std::array<double, 6> desired_wrench_array = {0.0, 0.0, force_z, 0.0, 0.0, 0.0}; //desired_wrench expressed relative to base frame
      Eigen::Map<Eigen::Matrix<double, 6, 1>> desired_wrench(desired_wrench_array.data());

      external_wrench_array = state.O_F_ext_hat_K;
      Eigen::Map<Eigen::Matrix<double, 6, 1>> external_wrench(external_wrench_array.data()); //exstimated external wrench (forces, torques) acting on stiffness frame, expressed relative to the base frame

      external_wrench = external_wrench - h_offset;//remove offset from exstimated external wrench

      wrench_error_integral += delta_t * (desired_wrench - external_wrench);

      Eigen::Map<Eigen::Matrix<double, 6, 7>> geometric_jacobian(model.zeroJacobian(franka::Frame::kEndEffector, state).data());

      Eigen::Matrix<double, 6, 7> analytic_jacobian;
      analytic_jacobian = invT * geometric_jacobian;

      dq_array = state.dq;
      Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(dq_array.data());

      Eigen::Matrix<double, 6, 1> pose_dot;
      pose_dot = analytic_jacobian*dq;

      Eigen::Matrix<double, 6, 1> pose_dot_tilde;
      pose_dot_tilde = desired_pose_dot - pose_dot;

      std::thread t1(fill_buffer1, state.tau_J, state.q);
      //Eigen::Matrix<double, 6 ,1> desired_pose2;
      //desired_pose2 << pose_posed_array[0], pose_posed_array[1], pose_posed_array[2], orient0;
      //Eigen::Matrix<double, 6 ,1> desired_pose_dot2;
      //desired_pose_dot2 << pose_posed_array[3], pose_posed_array[4], pose_posed_array[5], 0.0, 0.0, 0.0;
      std::thread t2(fill_buffer2, pose_array, pose_dot, external_wrench, desired_pose, desired_pose_dot);

      Eigen::VectorXd tau_d(7);

      tau_d << analytic_jacobian.transpose()*( Kp*pose_tilde + Dx*pose_dot_tilde + Kpf*(desired_wrench - external_wrench) + Ki*wrench_error_integral) /*- Kd*dq*/;

      std::array<double, 7> tau_d_array{};
      Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

      t1.join();
      t2.join();

      x = x + n_rows_1;
      y = y + n_rows_2;

      franka::Torques tau_m = tau_d_array;

      if (time >= t_fin) {
        std::cout << std::endl << "Finished motion, job done" << std::endl;
        return franka::MotionFinished(tau_m);
      }      

      return tau_d_array;

    };

    // Start real-time control loop.
    robot.control(impedance_force_control_callback);

    //Write saved logs to files
    write_to_file_1(MyFile_1, fs, tf, n_rows_1, n_element_1);
    write_to_file_2(MyFile_2, fs, tf, n_rows_2, n_element_2);

    //Release the holder
    gripper.move(max_width, 0.05);
    gripper.stop();

    //Move to final configuration
    std::array<double, 7> q_goal_final = {{0.33908,0.259282,0.11345,-1.6321,-0.00624437,1.83089,-1.87721}};
    MotionGenerator motion_generator_final(0.2, q_goal_final);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator_final);
    std::cout << "Finished moving to initial joint configuration." << std::endl;
    

  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    gripper.move(max_width, 0.05);
    gripper.stop();

    //Write saved logs to files even when some error occours
    write_to_file_1(MyFile_1, fs, tf, n_rows_1, n_element_1);
    write_to_file_2(MyFile_2, fs, tf, n_rows_2, n_element_2);

    return -1;
  }

  return 0;

}