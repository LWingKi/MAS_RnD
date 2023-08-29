#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>
#include <math.h>
#include <iostream>
#include <time.h>
#include <queue>

#include "robif2b/functions/kinova_gen3.h"
#include "kdl/chain.hpp"
#include "kdl/frames_io.hpp"
#include "kdl/framevel_io.hpp"
#include "kdl/kinfam_io.hpp"
#include "kdl/frames.hpp"
#include "kdl/chainfksolver.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainfksolvervel_recursive.hpp"
#include "kdl/chainhdsolver_vereshchagin.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "robot.hpp"
#include "gnu_plotter.hpp"
#include "tf_utils.hpp"
#include "logger.hpp"

#define DEG_TO_RAD(x) (x) * M_PI / 180.0
#define RAD_TO_DEG(x) (x) * 180.0 / M_PI


int64_t GetTickUs() // returns time in milliseconds
{
    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);                      // get current time and store it in start
    return (start.tv_sec * 1000LLU) + (start.tv_nsec / 1000000); // start.tv_sec is in seconds, start.tv_nsec is in nanoseconds
}

//set alpah value in vereshagen solver
void setAlpha(std::vector<double>* linear_alpha, std::vector<double>* angular_alpha, KDL::Jacobian &alpha){
    // Check if alpha is a 3x1 vector
    if (linear_alpha->size() != 3 || angular_alpha->size() != 3)return;
    KDL::Twist unit_force_x_l(
        KDL::Vector((*linear_alpha)[0], 0.0, 0.0), 
        KDL::Vector(0.0, 0.0, 0.0));
    alpha.setColumn(0, unit_force_x_l); 
    KDL::Twist unit_force_y_l(
        KDL::Vector(0.0, (*linear_alpha)[1], 0.0), 
        KDL::Vector(0.0, 0.0, 0.0));
    alpha.setColumn(1, unit_force_y_l); 

    KDL::Twist unit_force_z_l(
        KDL::Vector(0.0, 0.0, (*linear_alpha)[2]), 
        KDL::Vector(0.0, 0.0, 0.0));
    alpha.setColumn(2, unit_force_z_l); 
    
    KDL::Twist unit_force_x_a(
        KDL::Vector(0.0, 0.0, 0.0), 
        KDL::Vector((*angular_alpha)[0], 0.0, 0.0));
    alpha.setColumn(3, unit_force_x_a); 

    KDL::Twist unit_force_y_a(
        KDL::Vector(0.0, 0.0, 0.0), 
        KDL::Vector(0.0, (*angular_alpha)[1], 0.0));
    alpha.setColumn(4, unit_force_y_a); 
    
    KDL::Twist unit_force_z_a(
        KDL::Vector(0.0, 0.0, 0.0), 
        KDL::Vector(0.0, 0.0, (*angular_alpha)[2]));
    alpha.setColumn(5, unit_force_z_a); 

}

// position error calcualtion for p controller
void getError(KDL::Frame *currentpos,KDL::Frame *tragetpos,KDL::Twist &pos_error){
    pos_error = KDL::diff(*currentpos,*tragetpos);
}
// velocity error calcualtion for p controller
void getError(KDL::Twist *currentvel,KDL::Twist *targetvel,KDL::Twist &vel_error){
    vel_error = KDL::diff(*currentvel,*targetvel);
}
// P controller
void P_controller(double *p_gain_linear,double *p_gain_angular,const KDL::Twist *error_signal,KDL::Twist &control_signal){
    control_signal.vel = (*error_signal).vel * (*p_gain_linear);
    control_signal.rot = (*error_signal).rot * (*p_gain_angular);

}


int main(int argc, char **argv)
{   
    //Initialization for kinova robot
    bool success = false;
    double cycle_time = 0.001;
    enum robif2b_ctrl_mode ctrl_mode = ROBIF2B_CTRL_MODE_FORCE;
    double pos_msr[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    double vel_msr[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    double eff_msr[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    double cur_msr[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    double vol_msr[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    double pos_cmd[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    double vel_cmd[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    double eff_cmd[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    double cur_cmd[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    double m_gripper_pos_cmd = 0.0;
    double m_gripper_pos_msr = 0.0;
    double imu_ang_vel_msr[] = { 0.0, 0.0, 0.0 };
    double imu_lin_acc_msr[] = { 0.0, 0.0, 0.0 };

    struct robif2b_kinova_gen3_nbx rob = {
        // Configuration c++ 20 style
        .conf {
            .ip_address     = "192.168.1.10",
            .port               = 10000,
            .port_real_time     = 10001,
            .user               = "admin",
            .password           = "admin",
            .session_timeout    = 60000,
            .connection_timeout = 2000},

        // Connections
        .cycle_time = &cycle_time,
        .ctrl_mode = &ctrl_mode,
        .jnt_pos_msr = pos_msr,
        .jnt_vel_msr = vel_msr,
        .jnt_trq_msr = eff_msr,
        .act_cur_msr = cur_msr,
        .act_vol_msr = vol_msr,
        .gripper_pos_msr = m_gripper_pos_msr,
        .jnt_pos_cmd = pos_cmd,
        .jnt_vel_cmd = vel_cmd,
        .jnt_trq_cmd = eff_cmd,
        .act_cur_cmd = cur_cmd,
        .gripper_pos_cmd = m_gripper_pos_cmd,
        .imu_ang_vel_msr = imu_ang_vel_msr,
        .imu_lin_acc_msr = imu_lin_acc_msr,


        .success = &success
    };

    //Create robot chain
    KDL::Tree my_tree;
    KDL::Chain robot;

    // get current file path
    std::filesystem::path path = __FILE__;
    std::string robot_urdf = (path.parent_path().parent_path()/ "gen3_robotiq_2f_85.urdf").string();
    // std::string robot_urdf = (path.parent_path().parent_path()/ "gen3_7dof_vision_arm_urdf_v12.urdf").string();

    if (!kdl_parser::treeFromFile(robot_urdf, my_tree)){
        std::cout << "Failed to construct kdl tree" << std::endl;
        return -1;
    }
    // set the base and tool frames
    std::string base_link = "base_link";
    std::string tool_frame = "bracelet_link";

    // create the kdl chain
    if (!my_tree.getChain(base_link, tool_frame, robot)){
        std::cout << "Failed to get kdl chain" << std::endl;
        return -1;
    }

    //Initialization for vereshchagin solver 
    //Define vairables for solver
    const int num_constraint = 6;
    KDL::JntArray q(robot.getNrOfJoints());                     //joint angle
    KDL::JntArray qdot(robot.getNrOfJoints());                  //joint vel
    KDL::JntArray qddot(robot.getNrOfJoints());                 //joint accel
    KDL::JntArray ff_torques(robot.getNrOfJoints());            //Feed-Forward Torque
    KDL::JntArray constrain_torque(robot.getNrOfJoints());      //constrain torque
    std::vector<KDL::Wrench> f_ext(robot.getNrOfSegments());     // External forces on the end effector
    
    //Define and initialize cartesian acceleration constraints
    KDL::Jacobian alpha(num_constraint);
    std::vector<double> linear_alpha = {1.0, 1.0, 1.0};
    std::vector<double> angular_alpha = {1.0, 1.0, 1.0};
    setAlpha(&linear_alpha,&angular_alpha,alpha);
    
    //Initialize the values
    KDL::Vector force(0.0, 0.0, 0.0);
    KDL::Vector torque(0.0, 0.0, 0.0);
    KDL::Wrench f_tool(force, torque); 
    f_ext[robot.getNrOfSegments() - 1] = f_tool;
    
    KDL::JntArray beta(num_constraint);
    for (int i = 0; i<num_constraint;i++){
            beta(i) = 0.0;
    }

    //Create FK solver
    KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(robot);
    KDL::ChainFkSolverVel_recursive fkvelsolver = KDL::ChainFkSolverVel_recursive(robot);

    //Create verechshagin solver
    //gravity vector set to opposit direction 
    KDL::Twist base_accl(KDL::Vector(0.0, 0.0, 9.81), KDL::Vector(0.0, 0.0, 0.0));
    KDL::ChainHdSolver_Vereshchagin vereshchaginSolver(robot, base_accl,num_constraint);

    // Variables for calculations
    double timeStep = 0.001;
    int sampling_size = 5;
    std::vector<KDL::Frame> cartesian_pose(robot.getNrOfSegments());     // cartesian pose of all links
    std::vector<KDL::Twist> cartesian_velocity(robot.getNrOfSegments()); // cartesian velocity of all links
    KDL::Twist currVel; //EE velocity
    KDL::Frame currPos , goalPos , initialPos; //EE pose w.r.t to base frame
    KDL::FrameVel joint_vel_cart; // output of the FK vel solver  
    KDL::JntArrayVel joint_array_velocity(q,qdot); //input of the FK vel solver
    std::queue<double> accumulate_velocity;
    double norm_velocity , sum_velocity;

    //Create variables for position and velocity controllers
    KDL::Twist pos_error;
    KDL::Twist vel_error;
    KDL::Twist vel_set_point;
    KDL::Twist accel_set_point;
    KDL::Twist goal_velocity;
    double k_p_position_linear = 20;
    double k_p_position_angular = 40;
    double k_p_velocity_linear = 30;
    double k_p_velocity_angular = 1.5;
\


    // Values to limit the torque, for safety
    double constraint_torque_limit = 15;
    // double constraint_torque_threshold = 30;
    double velocity_limit = 0.02;
            
    //for plotting and logging
    std::vector<KDL::JntArray> joint_angle,joint_velocity,joint_accel,constraint_tau;
    std::vector<KDL::Vector> current_position,target_position;
    std::vector<KDL::Vector> current_velocity,target_velocity;
    std::vector<KDL::Twist> control_signal;
    std::vector<std::array<double, 7>> joint_torque;
    std::vector<std::array<double, 7>> actuar_current;
    std::vector<std::array<double, 7>> actuar_voltage;

    GNUPlotter plot_graph(path.parent_path()/"log",false,false);
    std::queue<double> tempQueue = accumulate_velocity; // Create a temporary queue to preserve the original queue
    std::queue<double> printQueue;
    // Initial the frame transformation
    TfUtils frame_TL;
    frame_TL.setChain(&robot);

    //Configure the robot
    robif2b_kinova_gen3_configure(&rob);
    if (!success) {
        printf("Error during gen3_configure\n");
        // goto shutdown;
    }
    // Clear fault state
    robif2b_kinova_gen3_recover(&rob);
    if (!success) {
        printf("Error during gen3_recover\n");
        // goto shutdown;
    }

    printf("Starting\n");
    robif2b_kinova_gen3_start(&rob);
    if (!success) {
        printf("Error during gen3_start\n");
    }


    // m_gripper_pos_cmd = 100;
    // Read initial joint positions, velocities and torque values
    publish_measurement(&rob);
    // the initial touque command such that robot stay still at the beginning
    *eff_cmd = *eff_msr * -1;
    // Initialising joint angle and joint velocity
    for (int i = 0 ; i<robot.getNrOfJoints();i++){
        q(i) = pos_msr[i];
        qdot(i) = vel_msr[i];
    }

    // Tranform from joint space to cartesian space
    fksolver.JntToCart(q, initialPos);
    currPos = initialPos;

    // assume the inital velocity of the end effectot is zero
    currVel = cartesian_velocity[robot.getNrOfSegments()-1];
    std::cout << "Initialised joint angle and joint velocity\n";
    

    if (!success) {
        printf("Error during gripper_update\n");
    }
    //setting goal position 
    goalPos.p = initialPos.p + KDL::Vector(0.0,0,-0.3);
    // goalPos.M = currPos.M * KDL::Rotation::RotX(M_PI_4);
    goalPos.M = initialPos.M;
    std::cout << "Initial ee frame: \n"  << currPos.p << std::endl;
    // frame_TL.transformFrame(goalPos,&q,CoordinateSystem::EE,CoordinateSystem::BASE);
    // frame_TL.transformJacobian(alpha,&q,CoordinateSystem::EE,CoordinateSystem::BASE);
    Logger logger(true,false,path.parent_path()/"log",false);
    //Start solver
    std::cout << "Start Solver" << std::endl;
    ///////////////////////////////////////////////////////////////////
    // STEPS:
    // 1.Given target position (in cart space)
    // 2.Perform FK from KDL
    // 3.Use P controller
    // 4.Get new beta
    // 5.Perform vereshchagin solver -> get new joint angles
    // 6.Send command to robot
    ///////////////////////////////////////////////////////////////////

    int counter = 0;
    double distance_counter = 0.0;
    
    do{
        // get the joint angle from the robot
        for (int i = 0 ; i<robot.getNrOfJoints();i++){
            q(i) = pos_msr[i];
            qdot(i) = vel_msr[i];
        }

        // Run the position controller in 10 Hz
        if (counter % 100 ==0){
            getError(&currPos,&goalPos,pos_error);
            P_controller(&k_p_position_linear,&k_p_position_angular,&pos_error,vel_set_point);
        }
        // Limit joint velocity
        if (vel_set_point.vel.z() > velocity_limit){
            vel_set_point.vel.data[2] = velocity_limit;
        }
        if (vel_set_point.vel.z() < -velocity_limit){
            vel_set_point.vel.data[2] = -velocity_limit;
        }
        // Run the velocity controller in 100 Hz
        if (counter % 10 == 0){
            getError(&currVel,&vel_set_point,vel_error);
            P_controller(&k_p_velocity_linear,&k_p_velocity_angular,&vel_error,accel_set_point);
        }
        // Set controlled accel energy signal : aphla* Xddot = beta
        beta(0) = accel_set_point.vel.x();
        beta(1) = accel_set_point.vel.y();
        beta(2) = accel_set_point.vel.z()+ 9.81;
        beta(3) = accel_set_point.rot.x();
        beta(4) = accel_set_point.rot.y();
        beta(5) = accel_set_point.rot.z();


        // run the vereshchagin solver
        int isSuccess = vereshchaginSolver.CartToJnt(q, qdot, qddot,alpha,beta ,f_ext, ff_torques,constrain_torque);
        if (isSuccess !=0){
            std::cout <<"Error in Vereshchagin Solver!"<< isSuccess << std::endl;
        }
        for (int m = 0 ; m< robot.getNrOfJoints();m++){
            eff_cmd[m] = constrain_torque(m);
        }

        // get the updated cartesian pose from the solver
        vereshchaginSolver.getLinkCartesianPose(cartesian_pose);
        currPos = cartesian_pose[robot.getNrOfSegments()-1];

        // get the updated cartesian velocity from the solver
        vereshchaginSolver.getLinkCartesianVelocity(cartesian_velocity);
        currVel = cartesian_velocity[robot.getNrOfSegments()-1];

        // update goal pose -> keep going down for 5 cm in each step
        goalPos.p = initialPos.p + KDL::Vector(0.0,0,distance_counter);
        // goalPos.p = KDL::Vector(initialPos.p.x(),initialPos.p.y(),currPos.p.z())+KDL::Vector(0.0,0,0.0001);
        // goalPos.p = currPos.p + KDL::Vector(0.0,0,0.01);
        goalPos.M = initialPos.M;
        distance_counter = distance_counter - 0.1;
        // logger.logInfo("============steps %d==========",counter);
        // logger.logInfo("Current: %s",cur_msr);
        // logger.logInfo("Voltage: %s",vol_msr);

        // std::cout << "============steps " << counter << "========== "<<std::endl;
        // std::cout << "-----Solver calculation-----"<<std::endl;
        std::cout << "joint angle: "<< cur_msr[0] << cur_msr[1] << cur_msr[2] << cur_msr[3] <<cur_msr[4] << cur_msr[5] << cur_msr[6]<< std::endl;
        std::cout << "joint velocity: "<< vol_msr[0] << vol_msr[1] << vol_msr[2] << vol_msr[3] <<vol_msr[4] << vol_msr[5] << vol_msr[6] << std::endl;
        // std::cout << "joint accel: " << qddot << std::endl;
        // std::cout << "constrain torque: " << constrain_torque << std::endl;
        // std::cout << "beta: " << beta << std::endl << std::endl;
        // std::cout << "joint toruqe: " << eff_msr[0] <<eff_msr[6] << std::endl << std::endl;
        // std::cout << "-----Data in cartesian space -----"<<std::endl;
        // std::cout << "curr cartesian pos: " << currPos.p << std::endl;
        // std::cout << "goal cartesian pos: " << goalPos.p << std::endl;
        // std::cout << "current velocity: " << currVel.vel << std::endl;
        // std::cout << "-----Controller calculation-----"<<std::endl;
        // std::cout << "currVel: " << currVel << std::endl;
        // std::cout << "position err" << pos_error.rot << std::endl;
        // std::cout << "velovity err" << vel_error << std::endl << std::endl;
        // std::cout << "ctrl out" << vel_error << std::endl << std::endl;
        // std::cout << "norm velocity" << norm_velocity << std::endl << std::endl;
        // std::cout << "-----command to robot-----"<< std::endl;
        // printf("veloitcy command:%f,%f,%f,%f,%f,%f,%f\n",vel_cmd[0],vel_cmd[1], vel_cmd[2] , vel_cmd[3],vel_cmd[4] , vel_cmd[5] , vel_cmd[6]); 
        // printf("position command:%f,%f,%f,%f,%f,%f,%f\n",pos_cmd[0],pos_cmd[1], pos_cmd[2] , pos_cmd[3],pos_cmd[4] , pos_cmd[5] , pos_cmd[6]); 
        // printf("torque command:%f,%f,%f,%f,%f,%f,%f\n\n",eff_cmd[0],eff_cmd[1], eff_cmd[2] , eff_cmd[3],eff_cmd[4] , eff_cmd[5] , eff_cmd[6]); 
        accumulate_velocity.push(currVel.vel.z());
        tempQueue = accumulate_velocity; // Create a temporary queue to preserve the original queue
        while (!tempQueue.empty()) {
            sum_velocity += tempQueue.front();
            tempQueue.pop();
        }
        if (accumulate_velocity.size() == 5){
            norm_velocity = sum_velocity/5;
            accumulate_velocity.pop(); // Pop the first element in the queue
            // condition to switch the state
            if (abs(norm_velocity) < 0.00001){
                std::cout << "==Contact established== \n";
                std::cout << "Final ee frame: \n" << currPos.p << std::endl;

                break;  
            }
            sum_velocity = 0;
        }
    
        //log data for plotting 
        current_position.push_back(pos_error.rot);
        target_position.push_back(goalPos.p);
        current_velocity.push_back(currVel.vel);
        target_velocity.push_back(vel_set_point.vel);
        joint_angle.push_back(q);
        joint_velocity.push_back(qdot);
        joint_accel.push_back(qddot);
        constraint_tau.push_back(constrain_torque);
        control_signal.push_back(accel_set_point);
        joint_torque.push_back({eff_msr[0],eff_msr[1], eff_msr[2] , eff_msr[3] ,eff_msr[4] , eff_msr[5] , eff_msr[6]});
        actuar_current.push_back({cur_msr[0],cur_msr[1], cur_msr[2] , cur_msr[3] ,cur_msr[4] , cur_msr[5] , cur_msr[6]});
        actuar_voltage.push_back({vol_msr[0],vol_msr[1], vol_msr[2] , vol_msr[3] ,vol_msr[4] , vol_msr[5] , vol_msr[6]});

        // send command to robot
        robif2b_kinova_gen3_update(&rob);
        // std::cout << "current time: " << GetTickUs() << std::endl;
        usleep(500);
        counter++;
    }while (true);
    // publish_measurement(&rob);
    // MOVE FORWARD motion
    //Switch back to velocity mode
    // ctrl_mode = ROBIF2B_CTRL_MODE_VELOCITY;
    // robif2b_kinova_gen3_update(&rob);
    usleep(15000);

    
    std::cout << "approaching object\n";

    KDL::Vector force2(-20.0, 0.0, 0.0);
    KDL::Vector torque2(0.0, 0.0, 0.0);
    KDL::Wrench f_tool2(force2, torque2); 
    f_ext[robot.getNrOfSegments() - 1] = f_tool2;

    // Disable the constraint in linear z direction
    linear_alpha = {1.0, 1.0, 0.0};
    angular_alpha = {1.0,1.0,1.0};
    setAlpha(&linear_alpha,&angular_alpha,alpha);
    // move forward to the object
    for (int i = 0 ; i<robot.getNrOfJoints();i++){
            q(i) = pos_msr[i];
            qdot(i) = vel_msr[i];
    }
    fksolver.JntToCart(q, initialPos);
    goalPos.p = initialPos.p + KDL::Vector(0.1,0,0);
    goalPos.M = initialPos.M;
    k_p_position_linear = 30;
    k_p_position_angular = 45;
    int counter_2 = 0;
    do{
        // get the joint angle from the robot
        for (int i = 0 ; i<robot.getNrOfJoints();i++){
            q(i) = pos_msr[i];
            qdot(i) = vel_msr[i];
        }

        // Run the position controller in 10 Hz
        getError(&currPos,&goalPos,pos_error);
        P_controller(&k_p_position_linear,&k_p_position_angular,&pos_error,vel_set_point);
        // Set controlled accel energy signal : aphla* Xddot = beta
        beta(0) = vel_set_point.vel.x();
        beta(1) = vel_set_point.vel.y();
        beta(3) = vel_set_point.rot.x();
        beta(4) = vel_set_point.rot.y();
        beta(5) = vel_set_point.rot.z();
        // run the vereshchagin solver
        int isSuccess = vereshchaginSolver.CartToJnt(q, qdot, qddot,alpha,beta ,f_ext, ff_torques,constrain_torque);
        if (isSuccess !=0){
            std::cout <<"Error in Vereshchagin Solver!"<< isSuccess << std::endl;
        }
        for (int m = 0 ; m< robot.getNrOfJoints();m++){

            eff_cmd[m] = constrain_torque(m);
        }

        // get the updated cartesian pose from the solver
        vereshchaginSolver.getLinkCartesianPose(cartesian_pose);
        currPos = cartesian_pose[robot.getNrOfSegments()-1];

        // get the updated cartesian velocity from the solver
        // vereshchaginSolver.getLinkCartesianVelocity(cartesian_velocity);
        // currVel = cartesian_velocity[robot.getNrOfSegments()-1];

        //  std::cout << "============2 steps " << counter_2 << "========== "<<std::endl;
        // // std::cout << "-----Solver calculation-----"<<std::endl;
        // // std::cout << "joint angle: "<< q << std::endl;
        // // std::cout << "joint velocity: "<< qdot << std::endl;
        // std::cout << "joint accel: " << qddot << std::endl;
        // std::cout << "constrain torque: " << constrain_torque << std::endl;
        // std::cout << "beta: " << beta << std::endl << std::endl;
        // // std::cout << "joint toruqe: " << eff_msr[0] <<eff_msr[6] << std::endl << std::endl;
        // std::cout << "-----Data in cartesian space -----"<<std::endl;
       
        // std::cout << "current velocity: " << currVel.vel << std::endl;

        //log data for plotting 
        current_position.push_back(pos_error.rot);
        target_position.push_back(goalPos.p);
        current_velocity.push_back(currVel.vel);
        target_velocity.push_back(vel_set_point.vel);
        joint_angle.push_back(q);
        joint_velocity.push_back(qdot);
        joint_accel.push_back(qddot);
        constraint_tau.push_back(constrain_torque);
        control_signal.push_back(accel_set_point);
        joint_torque.push_back({eff_msr[0],eff_msr[1], eff_msr[2] , eff_msr[3] ,eff_msr[4] , eff_msr[5] , eff_msr[6]});
        actuar_current.push_back({cur_msr[0],cur_msr[1], cur_msr[2] , cur_msr[3] ,cur_msr[4] , cur_msr[5] , cur_msr[6]});
        actuar_voltage.push_back({vol_msr[0],vol_msr[1], vol_msr[2] , vol_msr[3] ,vol_msr[4] , vol_msr[5] , vol_msr[6]});

        robif2b_kinova_gen3_update(&rob);
        counter_2++;
    }while(abs(pos_error.vel.x()) > 0.01);
     std::cout << "curr cartesian pos: " << currPos.p << std::endl;
    std::cout << "goal cartesian pos: " << goalPos.p << std::endl;
    
    std::cout << "Run ended successfully. Start stableization\n";

    // Stablise the robot
    usleep(15000);
    for (int k = 0; k < robot.getNrOfJoints(); k++){
        pos_cmd[k] = q(k);
    }

    //Switch back to velocity mode
    ctrl_mode = ROBIF2B_CTRL_MODE_POSITION;
    robif2b_kinova_gen3_update(&rob);
    plot_graph.saveDataToCSV(joint_angle,joint_velocity,joint_accel,constraint_tau,current_velocity,target_velocity,current_position,target_position,control_signal,joint_torque,actuar_current,actuar_voltage,"run_log");
    // plot_graph.plotXYZ(current_position,target_position,"Position",0.05);
    // plot_graph.plotXYZ(current_velocity,target_velocity,"Velocity",0.05);
    // goto stop;

// stop:
//     robif2b_kinova_gen3_stop(&rob);
//     printf("Stopped\n");

// shutdown:
//     robif2b_kinova_gen3_shutdown(&rob);
//     if (!success) {
//         printf("Error during gen3_shutdown\n");
//         return 1;
//     }
    
}
