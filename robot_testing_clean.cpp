//To be tested: with dropping only has velocity control
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
    beta(2) = 9.81;
    //Create FK olver
    KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(robot);

    //Create verechshagin solver
    //gravity vector set to opposit direction 
    KDL::Twist base_accl(KDL::Vector(0.0, 0.0, 9.81), KDL::Vector(0.0, 0.0, 0.0));
    KDL::ChainHdSolver_Vereshchagin vereshchaginSolver(robot, base_accl,num_constraint);

    // Variables for calculations
    double timeStep = 0.001;
    int sampling_size = 5;
    std::vector<KDL::Frame> cartesian_pose(robot.getNrOfSegments());     // cartesian pose of all links
    std::vector<KDL::Twist> cartesian_velocity(robot.getNrOfSegments()); // cartesian velocity of all links
    KDL::Twist currVel(KDL::Vector(0.0,0.0,0.0),KDL::Vector(0.0,0.0,0.0)); //EE velocity
    KDL::Frame currPos , goalPos , initialPos; //EE pose w.r.t to base frame
    std::queue<double> accumulate_velocity;
    double norm_velocity , sum_velocity;

    //Create variables for position and velocity controllers
    KDL::Twist vel_error;
    KDL::Twist accel_set_point;
    KDL::Twist goal_velocity(KDL::Vector(0.0,0.0,0.0),KDL::Vector(0.0,0.0,0.0));




    // Values to limit the torque, for safety
    double constraint_torque_limit = 15;
    double velocity_limit = 0.02;
            
    //for plotting
    std::vector<KDL::Vector> current_position,target_position;
    std::vector<KDL::Vector> current_velocity,target_velocity;
    GNUPlotter plot_graph(path.parent_path()/"log",true,false);
    Logger logger(true,false,path.parent_path()/"log",true);

    std::queue<double> tempQueue = accumulate_velocity; // Create a temporary queue to preserve the original queue
    std::queue<double> printQueue;


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

    // *eff_cmd = *eff_msr * -1;
    // publish_measurement(&rob);
    robif2b_kinova_gen3_update(&rob);
    // Tranform from joint space to cartesian space
    for(int i = 0 ;i <robot.getNrOfJoints();i++){
        q(i) = pos_msr[i];
        qdot(i) = vel_msr[i];
    }
    double k_p_velocity_linear = 5;
    double k_p_velocity_angular = 20;
    int counter = 0;
    // assume the inital velocity of the end effectot is zero
    // currVel = cartesian_velocity[robot.getNrOfSegments()-1];
        // KDL::Twist goal_velocity(KDL::Vector(0.0,0.0,0.0),KDL::Vector(0.0,0.0,0.0));
    logger.logInfo("=Before loop=");
    logger.logInfo("q: %s",q);
    logger.logInfo("qdot: %s",qdot);
    logger.logInfo("qddot: %s",qddot);
    logger.logInfo("constrain_torque: %s",constrain_torque);
    logger.logInfo("currVel vel: %s",currVel);
    logger.logInfo("goal vel: %s",goal_velocity);
    logger.logInfo("beta: %s",beta);
    logger.logInfo("vel error: %s",vel_error);
    while(true){
        getError(&currVel,&goal_velocity,vel_error);
        P_controller(&k_p_velocity_linear,&k_p_velocity_angular,&vel_error,accel_set_point);

        beta(0) = accel_set_point.vel.x();
        beta(1) = accel_set_point.vel.y();
        beta(2) = accel_set_point.vel.z()+ 9.81;
        beta(3) = accel_set_point.rot.x();
        beta(4) = accel_set_point.rot.y();
        beta(5) = accel_set_point.rot.z();

        int isSuccess = vereshchaginSolver.CartToJnt(q, qdot, qddot,alpha,beta ,f_ext, ff_torques,constrain_torque);
        if (isSuccess !=0){
            std::cout <<"Error in Vereshchagin Solver!"<< isSuccess << std::endl;
        }
        for (int m = 0 ; m < robot.getNrOfJoints();m++){
            eff_cmd[m] = constrain_torque(m);
        }

        // get the updated cartesian velocity from the solver
        vereshchaginSolver.getLinkCartesianVelocity(cartesian_velocity);
        currVel = cartesian_velocity[robot.getNrOfSegments()-1];

        logger.logInfo("=Step %d=",counter);
        logger.logInfo("q: %s",q);
        logger.logInfo("qdot: %s",qdot);
        logger.logInfo("qddot: %s",qddot);
        logger.logInfo("constrain_torque: %s",constrain_torque);
        logger.logInfo("currVel vel: %s",currVel);
        logger.logInfo("goal vel: %s",goal_velocity);
        logger.logInfo("beta: %s",beta);
        logger.logInfo("vel error: %s",vel_error);
        
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
            if (abs(norm_velocity) < 0.001){
                std::cout << "==Contact established== \n";
                std::cout << "Final ee frame: \n" << currPos.p << std::endl;
                break;  
            }
            sum_velocity = 0;
        }
        robif2b_kinova_gen3_update(&rob);
        usleep(500);
        counter++;
    }

    // plot_graph.plotXYZ(current_velocity,target_velocity,"Linear Velocity",0.05);
    // plot_graph.plotXYZ(current_position,target_position,"Angular Velocity",0.5);
    
}
