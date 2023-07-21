// SPDX-License-Identifier: LGPL-3.0
#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>
#include <math.h>
#include <iostream>

#include "robif2b/functions/kinova_gen3.h"
#include "kdl/chain.hpp"
#include "kdl/frames_io.hpp"
#include "kdl/kinfam_io.hpp"
#include "kdl/frames.hpp"
#include "kdl/chainfksolver.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainhdsolver_vereshchagin.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "robot.hpp"
#include "gnu_plotter.hpp"

#define DEG_TO_RAD(x) (x) * M_PI / 180.0
#define RAD_TO_DEG(x) (x) * 180.0 / M_PI
#define ROBIF2B_KINOVA_GEN3_NR_SEGMENTS 7

//set alpah value in vereshagen solver
void setAlpha(const std::vector<double>& linear_alpha, const std::vector<double>& angular_alpha, KDL::Jacobian* alpha){
    // Check if alpha is a 3x1 vector
    if (linear_alpha.size() != 3 || angular_alpha.size() != 3)return;
    KDL::Twist unit_force_x_l(
        KDL::Vector(linear_alpha[0], 0.0, 0.0), 
        KDL::Vector(0.0, 0.0, 0.0));
    alpha->setColumn(0, unit_force_x_l); 
    KDL::Twist unit_force_y_l(
        KDL::Vector(0.0, linear_alpha[1], 0.0), 
        KDL::Vector(0.0, 0.0, 0.0));
    alpha->setColumn(1, unit_force_y_l); 

    KDL::Twist unit_force_z_l(
        KDL::Vector(0.0, 0.0, linear_alpha[2]), 
        KDL::Vector(0.0, 0.0, 0.0));
    alpha->setColumn(2, unit_force_z_l); 
    
    KDL::Twist unit_force_x_a(
        KDL::Vector(0.0, 0.0, 0.0), 
        KDL::Vector(angular_alpha[0], 0.0, 0.0));
    alpha->setColumn(3, unit_force_x_a); 

    KDL::Twist unit_force_y_a(
        KDL::Vector(0.0, 0.0, 0.0), 
        KDL::Vector(0.0, angular_alpha[1], 0.0));
    alpha->setColumn(4, unit_force_y_a); 
    
    KDL::Twist unit_force_z_a(
        KDL::Vector(0.0, 0.0, 0.0), 
        KDL::Vector(0.0, 0.0, angular_alpha[2]));
    alpha->setColumn(5, unit_force_z_a); 

}

// position error calcualtion for p controller
void getError(KDL::Frame &currentpos,KDL::Frame &tragetpos,KDL::Twist &pos_error){
    pos_error = KDL::diff(currentpos,tragetpos);
}
// velocity error calcualtion for p controller
void getError(KDL::Twist &currentvel,KDL::Twist &targetvel,KDL::Twist &vel_error){
    vel_error = KDL::diff(currentvel,targetvel);
}
// P controller
void P_controller(double &p_gain,const KDL::Twist &error_signal,KDL::Twist &control_signal){
    control_signal = error_signal*p_gain;
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
    double pos_cmd[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    double vel_cmd[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    double eff_cmd[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    double cur_cmd[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
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
        .jnt_pos_cmd = pos_cmd,
        .jnt_vel_cmd = vel_cmd,
        .jnt_trq_cmd = eff_cmd,
        .act_cur_cmd = cur_cmd,
        .imu_ang_vel_msr = imu_ang_vel_msr,
        .imu_lin_acc_msr = imu_lin_acc_msr,
        .success = &success
    };
    //Create robot chain
    KDL::Tree my_tree;
    KDL::Chain robot;

    // get current file path
    std::filesystem::path path = __FILE__;
    // std::string robot_urdf = (path.parent_path().parent_path()/ "gen3_robotiq_2f_85.urdf").string();
    std::string robot_urdf = (path.parent_path().parent_path()/ "gen3_7dof_vision_arm_urdf_v12.urdf").string();

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
    KDL::JntArray q(ROBIF2B_KINOVA_GEN3_NR_JOINTS);          //joint angle
    KDL::JntArray qdot(ROBIF2B_KINOVA_GEN3_NR_JOINTS);       //joint vel
    KDL::JntArray qddot(ROBIF2B_KINOVA_GEN3_NR_JOINTS);    //joint accel
    KDL::JntArray ff_torques(ROBIF2B_KINOVA_GEN3_NR_JOINTS);   //Feed-Forward Torque
    KDL::JntArray constrain_torque(ROBIF2B_KINOVA_GEN3_NR_JOINTS);        //constrain torque
    std::vector<KDL::Wrench> f_ext(ROBIF2B_KINOVA_GEN3_NR_SEGMENTS);  // External forces on the end effector
    
    //Define and initialize cartesian acceleration constraints
    KDL::Jacobian alpha(num_constraint);
    const std::vector<double>& linear_alpha = {1.0, 1.0, 1.0};
    const std::vector<double>& angular_alpha = {1.0, 1.0, 1.0};
    setAlpha(linear_alpha,angular_alpha,&alpha);
    
    //Initialize the values
    KDL::Vector force(0.0, 0.0, 0.0);
    KDL::Vector torque(0.0, 0.0, 0.0);
    KDL::Wrench f_tool(force, torque); 
    f_ext[ROBIF2B_KINOVA_GEN3_NR_SEGMENTS - 1] = f_tool;
    
    KDL::JntArray beta(num_constraint);
    for (int i = 0; i<num_constraint;i++){
        if (i==2)
            beta(i) = 9.81;
        else
            beta(i) = 0.0;
    }

    //Create FK solver
    KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(robot);

    //Create verechshagin solver
    //gravity vector set to opposit direction 
    KDL::Twist base_accl(KDL::Vector(0.0, 0.0, 9.81), KDL::Vector(0.0, 0.0, 0.0));
    KDL::ChainHdSolver_Vereshchagin vereshchaginSolver(robot, base_accl,num_constraint);

    // Variables for calculations
    double timeStep = 0.001;
    std::vector<KDL::Frame> frames(ROBIF2B_KINOVA_GEN3_NR_SEGMENTS); // pose of all links
    std::vector<KDL::Twist> velocities(ROBIF2B_KINOVA_GEN3_NR_SEGMENTS); //velocity of all links
    KDL::Twist currentVel; //EE velocity
    KDL::Frame currpos , goalpos; //EE pose

    //Create variables for position and velocity controllers
    KDL::Twist pos_error;
    KDL::Twist vel_error;
    KDL::Twist vel_set_point;
    KDL::Twist accel_set_point;
    double k_p_position = 5;
    double k_p_velocity = 50;

    //for plotting
    std::vector<KDL::Vector> current_position,target_position;
    std::vector<KDL::Vector> current_velocity,target_velocity;
    GNUPlotter plot_graph(path.parent_path()/"log",true,false);
    
    //Configure the robot
    robif2b_kinova_gen3_configure(&rob);
    if (!success) {
        printf("Error during gen3_configure\n");
        goto shutdown;
    }
    robif2b_kinova_gen3_recover(&rob);
    if (!success) {
        printf("Error during gen3_recover\n");
        goto shutdown;
    }

    //Starting the robot in Force mode to read the initial pose and velocity
    printf("Starting\n");
    robif2b_kinova_gen3_start(&rob);
    if (!success) {
        printf("Error during gen3_start\n");
        goto stop;
    }
    robif2b_kinova_gen3_update(&rob);
    
    for (int i = 0 ; i<ROBIF2B_KINOVA_GEN3_NR_JOINTS;i++){
        q(i) = rob.jnt_pos_msr[i];
        qdot(i) = vel_msr[i];
    }
    //Switch back to position mode
    ctrl_mode = ROBIF2B_CTRL_MODE_POSITION;
    robif2b_kinova_gen3_update(&rob);

    //setting up goal pose
    fksolver.JntToCart(q, currpos);
    goalpos.p = currpos.p + KDL::Vector(0.10,0.0,0.0);
    // goalpos.M = currpos.M * KDL::Rotation::RotX(M_PI_4);
    goalpos.M = currpos.M;
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
    for (int counter = 0; counter < 3000; counter++) {
        // get cartesian pose from the solver
        vereshchaginSolver.getLinkCartesianPose(frames);
        currpos = frames[ROBIF2B_KINOVA_GEN3_NR_SEGMENTS-1];
            
        // get cartesian velocity from the solver
        vereshchaginSolver.getLinkCartesianVelocity(velocities);
        currentVel = velocities[ROBIF2B_KINOVA_GEN3_NR_SEGMENTS-1];

        if (counter % 100 ==0){
            getError(currpos,goalpos,pos_error);
            P_controller(k_p_position,pos_error,vel_set_point);
        }
        // Run the velocity controller in 100 Hz
        if (counter % 10 == 0){
            getError(currentVel,vel_set_point,vel_error);
            P_controller(k_p_velocity,vel_error,accel_set_point);
        }

        beta(0) = accel_set_point.vel.x();
        beta(1) = accel_set_point.vel.y();
        beta(2) = accel_set_point.vel.z() + 9.81;
        beta(3) = accel_set_point.rot.x();
        beta(4) = accel_set_point.rot.y();
        beta(5) = accel_set_point.rot.z();
        int isSuccess = vereshchaginSolver.CartToJnt(q, qdot, qddot,alpha,beta ,f_ext, ff_torques,constrain_torque);
        if (isSuccess !=0){
            std::cout <<"Error in Vereshchagin Solver!"<< isSuccess << std::endl;
            goto stop;
        }
        // calculate joint angle and velocity from solver
        for (int m = 0 ; m< ROBIF2B_KINOVA_GEN3_NR_JOINTS;m++){
            //setting joing accel and constraint torque limit
            if (abs(qddot(m)) >= 4){
                if (qddot(m) >=0) qddot(m) = 4;
                else qddot(m) = -4;
            }
            if (abs(constrain_torque(m)) >= 4){
                if (constrain_torque(m) >=0) constrain_torque(m) = 4;
                else constrain_torque(m) = -4;
            }
            //Also needs to limit joint velocity (how much?)
            qdot(m) = qdot(m) + qddot(m) * timeStep; //Euler Forward
            if (abs(qdot(m)) >= 1){
                if (qdot(m) >=0) qdot(m) = 1;
                else qdot(m) = -1;
            }

            q(m) = q(m) + qdot(m) * timeStep + 0.5 * qddot(m) * timeStep * timeStep;
            
            pos_cmd[m] = q(m);
            vel_cmd[m] = qdot(m);
            eff_cmd[m] = constrain_torque(m);
        }
        // send command to robot
        robif2b_kinova_gen3_update(&rob);

        //log data for plotting
        current_position.push_back(currpos.p);
        target_position.push_back(goalpos.p);
        current_velocity.push_back(currentVel.vel);
        target_velocity.push_back(vel_set_point.vel);
        usleep(1000);
        
    }
    std::cout << "Run ended successfully\n";
    plot_graph.plotXYZ(current_position,target_position,"Position",0.01);
    plot_graph.plotXYZ(current_velocity,target_velocity,"Velocity",0.05);

stop:
    robif2b_kinova_gen3_stop(&rob);
    printf("Stopped\n");

shutdown:
    robif2b_kinova_gen3_shutdown(&rob);
    if (!success) {
        printf("Error during gen3_shutdown\n");
        return 1;
    }
    return 0;
}
