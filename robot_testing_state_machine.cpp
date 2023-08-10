// SPDX-License-Identifier: LGPL-3.0
#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>
#include <math.h>
#include <iostream>
#include <time.h>
#include <queue>
#include <thread>
#include <chrono>

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


class RobotTask;

class RobotState{
    public:
        enum class State{
            HOME,
            DOWN,
            FORWARD,
            GRASP,
            STOP
        };
        virtual ~RobotState(){}
        virtual void action(RobotTask& robotTask,robif2b_kinova_gen3_nbx& rob) =0;
        virtual State getCurrentState() const = 0;
};

class Home: public RobotState{
    public:
        void action(RobotTask& robotTask,robif2b_kinova_gen3_nbx& rob) override;
        State getCurrentState() const override{return State::HOME;}
};

class Down: public RobotState{
    public:
        void action(RobotTask& robotTask,robif2b_kinova_gen3_nbx& rob) override;
        State getCurrentState() const override{return State::DOWN;}
};

class Forward: public RobotState{
    public:
        void action(RobotTask& robotTask,robif2b_kinova_gen3_nbx& rob) override;
        State getCurrentState() const override{return State::FORWARD;}
};

class Grasp: public RobotState{
    public:
        void action(RobotTask& robotTask,robif2b_kinova_gen3_nbx& rob) override;
        State getCurrentState() const override{return State::GRASP;}
};

class Stop: public RobotState{
    public:
        void action(RobotTask& robotTask,robif2b_kinova_gen3_nbx& rob) override;
        State getCurrentState() const override{return State::STOP;}
};

class RobotTask{
    public:
        RobotTask(): currentState(new Home){}

        void transition(bool isError,bool isReady,bool isContact,bool isReach,bool isGrasp){
            if(isError){
                delete currentState;
                currentState = new Stop;
                return;
            }
            RobotState* nextState = currentState;
            switch (currentState->getCurrentState()){
                case RobotState::State::HOME:
                    nextState = new Down;
                    break;
                case RobotState::State::DOWN:
                    if(isContact)
                        nextState = new Forward;
                    break;
                case RobotState::State::FORWARD:
                    if(isReach)
                        nextState = new Grasp;;
                    break;
                case RobotState::State::GRASP:
                    if(isGrasp)
                        nextState = new Home;
                    break;
                default:
                    break;
            }
            if (nextState != currentState){
                delete currentState;
               currentState = nextState;
            }
        }
        void action(robif2b_kinova_gen3_nbx& rob) {
            currentState->action(*this,rob);
        }


        RobotState* getCurrentState() const {
            return currentState;
        }

    private:
        RobotState* currentState;
};

void Home::action(RobotTask& robotTask,robif2b_kinova_gen3_nbx& rob){
    //isError,isReady,isContact,isReach,isGrasp
    publish_measurement(&rob);
    double torque_value[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    double torque_command[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    rob.jnt_trq_msr = torque_value;
    rob.jnt_trq_cmd = torque_command;
    *torque_command = *torque_value * -1;
    robif2b_kinova_gen3_update(&rob);
    robotTask.transition(false,true,false,false,false);
}

void Down::action(RobotTask& robotTask,robif2b_kinova_gen3_nbx& rob){
      // get the joint angle from the robot
    for (int i = 0 ; i<robot.getNrOfJoints();i++){
        q(i) = pos_msr[i];
        qdot(i) = vel_msr[i];
    }
    robotTask.transition(false,false,true,false,false);
}
void Forward::action(RobotTask& robotTask,robif2b_kinova_gen3_nbx& rob){
    robotTask.transition(false,false,true,true,false);
}
void Grasp::action(RobotTask& robotTask,robif2b_kinova_gen3_nbx& rob){
    robotTask.transition(false,true,true,true,true);
}
void Stop::action(RobotTask& robotTask,robif2b_kinova_gen3_nbx& rob){
    
}

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

int main(){
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
    struct ArmRobot rob = {
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
    };
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
    std::vector<KDL::Frame> cartesian_pose(robot.getNrOfSegments());     // cartesian pose of all links
    std::vector<KDL::Twist> cartesian_velocity(robot.getNrOfSegments()); // cartesian velocity of all links
    KDL::Twist currVel , goalVel; //EE velocity
    KDL::Frame currPos , goalPos , initialPos; //EE pose w.r.t to base frame
    std::queue<double> accumulate_velocity;
    double norm_velocity , sum_velocity;
    int sampling_size = 5; // The size of the queue 
    std::queue<double> tempQueue;
    std::queue<double> printQueue;

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

    // Limit values for safety
    double constraint_torque_limit = 15;
    double velocity_limit = 0.02;
            
    //for plotting
    std::vector<KDL::Vector> current_position,target_position;
    std::vector<KDL::Vector> current_velocity,target_velocity;
    GNUPlotter plot_graph(path.parent_path()/"log",true,false);
    Logger logger(true,false,path.parent_path()/"log",true);

    // Initial the frame transformation
    TfUtils frame_TL;
    frame_TL.setChain(&robot);


    // Declare the state machine
    RobotTask robotTask;

    // Simulate different conditions
    bool isError = false;
    bool isContact = false;
    bool isReach = false;
    bool isGrasp = false;
    bool isFinish = false;
    bool isReady = false;

    //Configure the robot
    robif2b_kinova_gen3_configure(&rob);
    if (!success) {
        logger.logError("Error during gen3_configure\n");
        goto shutdown;
    }else{
        isReady = true;
    }

    // Clear fault state
    robif2b_kinova_gen3_recover(&rob);
    if (!success) {
        logger.logError("Error during gen3_recover\n");
        goto shutdown;
    }

    logger.logInfo("Starting\n");
    robif2b_kinova_gen3_start(&rob);
    if (!success) {
        logger.logError("Error during gen3_start\n");
        goto shutdown;
    }

    // Assume the initial cartisian velocity is zero
    currVel = cartesian_velocity[robot.getNrOfSegments()-1];
    // Read initial joint positions, velocities and torque values
       publish_measurement(&rob);
    // the initial touque command such that robot stay still at the beginning
    *eff_cmd = *eff_msr * -1;
    // Initialising joint angle and joint velocity
    for (int i = 0 ; i<robot.getNrOfJoints();i++){
        q(i) = pos_msr[i];
        qdot(i) = vel_msr[i];
    }

    goalVel.vel = KDL::Vector(0,0,-0.05);
    goalVel.rot = KDL::Vector(0,0,0);

    while (!isFinish){
        robotTask.transition(isError,isReady,isContact,isReach,isGrasp);
        robotTask.action(rob);
        if (dynamic_cast<Grasp*>(robotTask.getCurrentState()) &&
            dynamic_cast<Home*>(robotTask.getCurrentState())) {
            isFinish = true;
        }
        
        // Delay for a short duration (e.g., using sleep_for) if needed
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    
shutdown:
    robif2b_kinova_gen3_shutdown(&rob);
    if (!success) {
        logger.logError("Error during gen3_shutdown\n");
        return 1;
    }
}