#include "kdl/chain.hpp"
#include "kdl/frames_io.hpp"
#include "kdl/kinfam_io.hpp"
#include "kdl/frames.hpp"
#include "kdl/chainfksolver.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainhdsolver_vereshchagin.hpp"
#include "kdl_parser/kdl_parser.hpp"

#include <stdio.h>
#include <iostream>
#include "robot.hpp"
#include "controller.hpp"
#include "gnu_plotter.hpp"

using namespace KDL;

void setAlpha(const std::vector<double>& linear_alpha, const std::vector<double>& angular_alpha, KDL::Jacobian* alpha){
    // Check if alpha is a 3x1 vector
    if (linear_alpha.size() != 3 || angular_alpha.size() != 3)return;
    Twist unit_force_x_l(
        Vector(linear_alpha[0], 0.0, 0.0), 
        Vector(0.0, 0.0, 0.0));
    alpha->setColumn(0, unit_force_x_l); 
    Twist unit_force_y_l(
        Vector(0.0, linear_alpha[1], 0.0), 
        Vector(0.0, 0.0, 0.0));
    alpha->setColumn(1, unit_force_y_l); 

    Twist unit_force_z_l(
        Vector(0.0, 0.0, linear_alpha[2]), 
        Vector(0.0, 0.0, 0.0));
    alpha->setColumn(2, unit_force_z_l); 
    
    Twist unit_force_x_a(
        Vector(0.0, 0.0, 0.0), 
        Vector(angular_alpha[0], 0.0, 0.0));
    alpha->setColumn(3, unit_force_x_a); 

        Twist unit_force_y_a(
        Vector(0.0, 0.0, 0.0), 
        Vector(0.0, angular_alpha[1], 0.0));
    alpha->setColumn(4, unit_force_y_a); 
    
    Twist unit_force_z_a(
        Vector(0.0, 0.0, 0.0), 
        Vector(0.0, 0.0, angular_alpha[2]));
    alpha->setColumn(5, unit_force_z_a); 

}

void getError(KDL::Frame &currentpos,KDL::Frame &tragetpos,KDL::Twist &pos_error){
    pos_error = KDL::diff(currentpos,tragetpos);
}

void getError(KDL::Twist &currentvel,KDL::Twist &targetvel,KDL::Twist &vel_error){
    vel_error = KDL::diff(currentvel,targetvel);
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
    const std::vector<double>& linear_alpha = {1.0, 1.0, 1.0};
    const std::vector<double>& angular_alpha = {1.0, 1.0, 1.0};
    setAlpha(linear_alpha,angular_alpha,&alpha);

    //Initialize the values
    Vector force(0.0, 0.0, 0.0);
    Vector torque(0.0, 0.0, 0.0);
    Wrench f_tool(force, torque); 
    f_ext[robot.getNrOfSegments() - 1] = f_tool;

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

    //Create verechshagin solver
    //gravity vector set to opposit direction 
    KDL::Twist base_accl(KDL::Vector(0.0, 0.0, 9.81), KDL::Vector(0.0, 0.0, 0.0));
    ChainHdSolver_Vereshchagin vereshchaginSolver(robot, base_accl,num_constraint);

    //Define variables for controller
    double timeStep = 0.001;
    std::vector<Frame> frames(robot.getNrOfSegments());
    std::vector<Twist> velocity(robot.getNrOfSegments()); //velocity of all links
    Twist currentVel; //EE velocity
    Twist targetvel(
        Vector(0.5, 0.0, 0.0), 
        Vector(0.0, 0.0, 0.0));

    // output of the controllers
    Twist velocity_set_point(Vector(0.0, 0.0, 0.0), Vector(0.0, 0.0, 0.0));
    Twist acceleration_set_point(Vector(0.0, 0.0, 0.0), Vector(0.0, 0.0, 0.0));

    Frame currpos , goalpos;
    // getting the position error for while loop
    fksolver.JntToCart(q, currpos);
    goalpos.p = currpos.p + KDL::Vector(0.10,0.0,0.0);
    goalpos.M = currpos.M * KDL::Rotation::RotX(M_PI_4);

    //Create position and velocity controllers
    KDL::Twist pos_error;
    KDL::Twist vel_error;
    KDL::Twist vel_set_point;
    KDL::Twist accel_set_point;
    double k_p_position = 5;
    double k_p_velocity = 50;

    PController position_controller = PController(5,0,0);
    int counter = 0;
    //for plotting
    std::vector<KDL::Vector> current_position,target_position;
    std::vector<KDL::Vector> current_velocity,target_velocity;

    GNUPlotter plot_graph(path.parent_path()/"log",true,false);

    do{ 
        int isSuccess = vereshchaginSolver.CartToJnt(q, qdot, qddot,alpha,beta ,f_ext, ff_torques,constrain_torque);
        if (isSuccess !=0){
            std::cout <<"ERROR!"<< isSuccess << std::endl;
            break;
        }
        else{
            for (int m = 0;m < robot.getNrOfJoints() ; m++){
                qdot(m) = qdot(m) + qddot(m) * timeStep; //Euler Forward
                q(m) = q(m) + qdot(m) * timeStep + 0.5 * qddot(m) * timeStep * timeStep ;
            }

            // get cartesian pose from the solver
            vereshchaginSolver.getLinkCartesianPose(frames);
            currpos = frames[robot.getNrOfSegments()-1];
            
            // get cartesian velocity from the solver
            vereshchaginSolver.getLinkCartesianVelocity(velocity);
            currentVel = velocity[robot.getNrOfSegments()-1];

            // Run the position controller in 10Hz
            if (counter % 100 ==0){
                getError(currpos,goalpos,pos_error);
                P_controller(k_p_position,pos_error,vel_set_point);
            }
            // Run the velocity controller in 100 Hz
            if (counter % 10 == 0){
                getError(currentVel,vel_set_point,vel_error);
                P_controller(k_p_velocity,vel_error,accel_set_point);
            }
            //only enale the position controller
            // beta(0) =velocity_set_point.vel.x();
            // beta(1) =velocity_set_point.vel.y();
            // beta(2) =velocity_set_point.vel.z() + 9.81;
            // accel_set_point = vel_set_point;

            // //Enable the cascade controller
            beta(0) = accel_set_point.vel.x();
            beta(1) = accel_set_point.vel.y();
            beta(2) = accel_set_point.vel.z() + 9.81;
            beta(3) = accel_set_point.rot.x();
            beta(4) = accel_set_point.rot.y();
            beta(5) = accel_set_point.rot.z();

            counter ++;

            //logging
            // std::cout << "=====step" << counter << "=====" <<  std::endl;
            // std::cout << "--Controller outputs--" << std:: endl;

            // std::cout << "position error :" << pos_error << std::endl;
            // std::cout << "velocity set point" << vel_set_point<< std:: endl;
            // std::cout << "velocity error :" << vel_error << std::endl;
            // std::cout << "accel set point" << accel_set_point << std::endl<< std::endl;

            // std::cout << "beta" << beta<< std:: endl;
            // std::cout << "position error :" << position_error.vel << std::endl;
            // std::cout << "velocity set point" << velocity_set_point<< std:: endl;
            // std::cout << "velocity error :" << vel_error.vel << std::endl;
            // std::cout << "accel set point" << accel_set_point<< std::endl<< std::endl;

            // std::cout << "--Solver outputs--" << std:: endl;
            // std::cout << "qddot:" << qddot << std::endl;
            // std::cout << "contraint torque:" << constrain_torque << std::endl;
            // std::cout << "current EE position :" << currpos.p << std::endl;
            // std::cout << "current EE orientation :" << currpos.M << std::endl;
            // std::cout << "Goal position :" << goalpos.p << std::endl;
            // std::cout << "Goal orientation :" << goalpos.M << std::endl;
            // std::cout << std::endl;

            current_position.push_back(currpos.p);
            target_position.push_back(goalpos.p);
            current_velocity.push_back(currentVel.vel);
            // target_velocity.push_back(velocity_set_point.vel);
            target_velocity.push_back(vel_set_point.vel);


        }
        if (counter == 900) break;
    //}while (abs(linear_position_err.x()) > 0.01 || abs(linear_position_err.y()) > 0.01 || abs(linear_position_err.z()) > 0.01);
    }while(true);
    std::cout << "Target reached.";
    plot_graph.plotXYZ(current_position,target_position,"Position",0.01);
    plot_graph.plotXYZ(current_velocity,target_velocity,"Velocity",0.05);
    
}