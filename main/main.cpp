#include <stdio.h>
#include <iostream>
#include <queue>

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
#include "controller.hpp"
#include "gnu_plotter.hpp"
#include "logger.hpp"
#include "tf_utils.hpp"

using namespace KDL;

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
void getError(KDL::Frame &currentpos,KDL::Frame &tragetpos,KDL::Twist &pos_error){
    pos_error = KDL::diff(currentpos,tragetpos);
}

void getError(KDL::Twist &currVel,KDL::Twist &targetvel,KDL::Twist &vel_error){
    vel_error = KDL::diff(currVel,targetvel);
}

void P_controller(double &p_gain,const KDL::Twist &error_signal,KDL::Twist &control_signal){
    control_signal = error_signal*p_gain;
}



int main(int argc , char** argv){
    KDL::Tree my_tree;
    //create a robot
    // Chain robot = Kinova_gen3();
    // Chain robot = Create_gen3();
    Chain robot;

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

    //Define vairables for solver
    const int num_constraint = 6;
    JntArray q(robot.getNrOfJoints());          //joint angle
    JntArray qdot(robot.getNrOfJoints());       //joint vel
    JntArray qddot(robot.getNrOfJoints());    //joint accel
    JntArray ff_torques(robot.getNrOfJoints());   //Feed-Forward Torque
    JntArray constrain_torque(robot.getNrOfJoints());        //constrain torque
    std::vector<Wrench> f_ext(robot.getNrOfSegments());  // External forces on the end effector
    
    //Define and initialize cartesian acceleration constraints
    Jacobian alpha(num_constraint);
    std::vector<double> linear_alpha = {1.0, 1.0, 1.0};
    std::vector<double> angular_alpha = {1.0, 1.0, 1.0};
    setAlpha(&linear_alpha,&angular_alpha,alpha);

    //Initialize the values
    Vector force(0.0, 0.0, 0.0);
    Vector torque(0.0, 0.0, 0.0);
    Wrench f_tool(force, torque); 
    f_ext[robot.getNrOfSegments() - 1] = f_tool;

    std::cout << robot.getNrOfSegments();
    KDL::JntArray beta(num_constraint);
    beta(0) = 0.0;
    beta(1) = 0.0;
    beta(2) = 9.81;
    beta(3) = 0.0;
    beta(4) = 0.0;
    beta(5) = 0.0;


    // for simulation
    q(0) = 0; 
    q(1) = 0;
    q(2) = 0;
    q(3) = -M_PI_2;
    q(4) = 0;
    q(5) = -M_PI_2;
    q(6) = 0;

    //Create FK solver
    ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(robot);

    //Create FK vel solver
    ChainFkSolverVel_recursive fkvelsolver = ChainFkSolverVel_recursive(robot);
    KDL::JntArrayVel joint_array_velocity(q,qdot);

    //Create verechshagin solver`
    //gravity vector set to opposit direction 
    KDL::Twist base_accl(KDL::Vector(0.0, 0.0, 9.81), KDL::Vector(0.0, 0.0, 0.0));
    ChainHdSolver_Vereshchagin vereshchaginSolver(robot, base_accl,num_constraint);

    //Define variables for controller
    double timeStep = 0.001;
    std::vector<Frame> frames(robot.getNrOfSegments());
    std::vector<Twist> velocity(robot.getNrOfSegments()); //velocity of all links
    Twist currVel(Vector(0.0, 0.0, 0.0), Vector(0.0, 0.0, 0.0));; //EE velocity
    Twist targetVel(Vector(0.0, 0.0, 0.0), Vector(0.0, 0.0, 0.0));

    // output of the controllers
    Twist velocity_set_point(Vector(0.0, 0.0, 0.0), Vector(0.0, 0.0, 0.0));
    Twist acceleration_set_point(Vector(0.0, 0.0, 0.0), Vector(0.0, 0.0, 0.0));

    Frame initPos, currPos , goalPos;
    // getting the position error for while loop
    fksolver.JntToCart(q, initPos);
    currPos = initPos;
    goalPos.p = initPos.p + KDL::Vector(0.0,0.0,-0.1);
    goalPos.M = initPos.M;
    // KDL::FrameVel joint_vel_cart;
    // goalPos.p = KDL::Vector(0.3,0.0,0.0);
    // goalPos.M = currPos.M * KDL::Rotation::RotX(M_PI_4);

    //Create position and velocity controllers
    KDL::Twist pos_error;
    KDL::Twist vel_error;
    KDL::Twist vel_set_point;
    KDL::Twist accel_set_point;
    double k_p_position = 10;
    double k_p_velocity = 30;
    double velocity_limit = 0.01;
    double distance_counter = 0.0;
    int counter = 0;
    //for plotting
    std::vector<KDL::Vector> current_position,target_position;
    std::vector<KDL::Vector> current_velocity,target_velocity;
    std::queue<double> accumulate_velocity;
    double norm_velocity , sum_velocity;
    double constraint_torque_limit = 30;
    std::queue<double> tempQueue = accumulate_velocity;
    GNUPlotter plot_graph(path.parent_path()/"log",true,false);
    Logger logger(true,false,path.parent_path()/"log",true);
    do{ 

        for (int m = 0;m < robot.getNrOfJoints() ; m++){
            qdot(m) = qdot(m) + qddot(m) * timeStep; //Euler Forward
            q(m) = q(m) + qdot(m) * timeStep + 0.5 * qddot(m) * timeStep * timeStep ;
        }
        // Run the position controller in 10 Hz
        // if (counter % 10 ==0){
        //     getError(currPos,goalPos,pos_error);
        //     P_controller(k_p_position,pos_error,vel_set_point);
        // }

        // if (vel_set_point.vel.z() > velocity_limit){
        //     vel_set_point.vel.data[2] = velocity_limit;
        // }
        // if (vel_set_point.vel.z() < -velocity_limit){
        //     vel_set_point.vel.data[2] = -velocity_limit;
        // }
        // Run the velocity controller in 100 Hz
        if (counter % 1 == 0){
            getError(currVel,targetVel,vel_error);
            P_controller(k_p_velocity,vel_error,accel_set_point);
        }
        // // get cartesian velocity from fk solver
        // fkvelsolver.JntToCart(joint_array_velocity,joint_vel_cart,robot.getNrOfSegments());
        // KDL::Twist temp(joint_vel_cart.p.v,joint_vel_cart.p.p);
        // std::cout << "\n";
        // std::cout << "Joint velocity in cart space from fk solver:\n" << std::endl;
        // std::cout << joint_vel_cart.p.v <<std::endl;

        // std::cout << "Joint velocity in cart space from vereshchagin solver:\n" << std::endl;
        // std::cout << currVel << std::endl;


        // //Enable the cascade controller
        beta(0) = accel_set_point.vel.x();
        beta(1) = accel_set_point.vel.y();
        beta(2) = accel_set_point.vel.z()+9.81;
        beta(3) = accel_set_point.rot.x();
        beta(4) = accel_set_point.rot.y();
        beta(5) = accel_set_point.rot.z();


        int isSuccess = vereshchaginSolver.CartToJnt(q, qdot, qddot,alpha,beta ,f_ext, ff_torques,constrain_torque);
        if (isSuccess !=0){
            std::cout <<"ERROR!"<< isSuccess << std::endl;
            break;
        }
        // for (int m = 0 ; m< robot.getNrOfJoints();m++){
        //     // setting limit to contraint torque
        //     if (constrain_torque(m) >= constraint_torque_limit){
        //         constrain_torque(m) = constraint_torque_limit;
        //     }
        //     if (constrain_torque(m) <= -constraint_torque_limit){
        //         constrain_torque(m) = -constraint_torque_limit;
        //     }
        // }
        // get cartesian pose from the solver
        vereshchaginSolver.getLinkCartesianPose(frames);
        currPos = frames[robot.getNrOfSegments()-1];
        
        // get cartesian velocity from the solver
        vereshchaginSolver.getLinkCartesianVelocity(velocity);
        currVel = velocity[robot.getNrOfSegments()-1];

        //logging
        // std::cout << "=====step" << counter << "=====" <<  std::endl;
        
        logger.logWarning("======Step %d=====",counter);
        logger.logInfo("Current position:%s",currPos.p);
        logger.logInfo("Goal position:%s",goalPos.p);
        logger.logInfo("Beta:%s",beta);
        logger.logInfo("Contraint torque:%s",constrain_torque);
        logger.logInfo("Joint acceleration:%s",qddot);
        logger.logInfo("current velocuty:%s",currVel);
        
        // logger.Logger::logInfo("accel set point:%s",accel_set_point);
        
        // std::cout << "--Controller outputs--" << std:: endl;
        // std::cout << "current velocuty" << currVel << std::endl<< std::endl;
        // std::cout << "position error :" << pos_error << std::endl;
        // std::cout << "velocity set point" << vel_set_point<< std:: endl;
        // std::cout << "velocity error :" << vel_error << std::endl;
        // std::cout << "accel set point" << accel_set_point << std::endl<< std::endl;

        // std::cout << "beta" << beta<< std:: endl;
        // std::cout << "position error :" << pos_error.vel.z() << std::endl;
        // std::cout << "velocity set point" << velocity_set_point<< std:: endl;
        // std::cout << "velocity error :" << vel_error.vel << std::endl;
        // std::cout << "accel set point" << accel_set_point<< std::endl<< std::endl;

        // std::cout << "--Solver outputs--" << std:: endl;
        // std::cout << "qddot:" << qddot << std::endl;
        // std::cout << "contraint torque:" << constrain_torque << std::endl;
        
        // std::cout << "current EE position :" << currPos.p << std::endl;
        // std::cout << "current EE orientation :" << currPos.M << std::endl;
        // std::cout << "Goal position :" << goalPos.p << std::endl;
        // std::cout << "Goal orientation :" << goalPos.M << std::endl;
        // std::cout << std::endl;

        goalPos.p = initPos.p + KDL::Vector(0.0,0,distance_counter);
        goalPos.M = initPos.M;
        distance_counter = distance_counter - 0.05;

        tempQueue = accumulate_velocity; // Create a temporary queue to preserve the original queue
        std::cout << "The Queue: ";
        while (!tempQueue.empty()) {
            std::cout << tempQueue.front() << " ";
            tempQueue.pop();
        }

        std::cout << "\n";
        accumulate_velocity.push(currVel.vel.z());
        tempQueue = accumulate_velocity; // Create a temporary queue to preserve the original queue
        while (!tempQueue.empty()) {
            sum_velocity += tempQueue.front();
            tempQueue.pop();
        }
        printf("sum_velocity: %f\n",sum_velocity);
        if (accumulate_velocity.size() == 5){
            norm_velocity = sum_velocity/5;
            accumulate_velocity.pop(); // Pop the first element in the queue
            printf("norm_velocity: %f\n",norm_velocity);
            // condition to switch the state
            if (abs(norm_velocity) < 0.001){
                std::cout << "==Contact established== \n";
                break;  
            }
            sum_velocity = 0;
        }
        // logger.logWarning("======norm_velocity %f=====",norm_velocity);
        current_position.push_back(currPos.p);
        target_position.push_back(goalPos.p);
        current_velocity.push_back(currVel.vel);
        // target_velocity.push_back(velocity_set_point.vel);
        target_velocity.push_back(vel_set_point.vel);
        
        counter ++;

        // if (abs(currVel.vel.z()) < 0.01) break;
        // if (counter == 500) break;
    //}while (abs(linear_position_err.x()) > 0.01 || abs(linear_position_err.y()) > 0.01 || abs(linear_position_err.z()) > 0.01);
    }while(true);

  
    
    std::cout << "Target reached.";
    plot_graph.plotXYZ(current_position,target_position,"Position",0.01);
    plot_graph.plotXYZ(current_velocity,target_velocity,"Velocity",0.05);
}