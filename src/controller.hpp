#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP
#include "kdl/chain.hpp"
#include "kdl/frames_io.hpp"
#include "kdl/kinfam_io.hpp"
#include "kdl/frames.hpp"
#include "kdl/chainfksolver.hpp"

class PController{
    public:
        PController(double k_p,double k_i,double k_d);
        KDL::Twist getError(KDL::Frame currentpos,KDL::Frame tragetpos);
        KDL::Twist getError(KDL::Twist currentvel,KDL::Twist tragetvel);

        KDL::Twist getControlSignal(KDL::Frame currentpos,KDL::Frame tragetpos,double dt);
        KDL::Twist getControlSignal(KDL::Twist currentvel,KDL::Twist tragetvel,double dt);
    private:
        double k_p,k_i,k_d;
        //PID calculation
        KDL::Twist intergal;
        KDL::Twist derivative;
        KDL::Twist proportional;
        KDL::Twist last_error;

        //variable for pos control
        KDL::Vector linear_err;
        KDL::Twist ang_err;
        KDL::Twist pos_control_error;
        KDL::Twist pos_control_signal;
        KDL::Twist pos_control_error_sum;

        //variable for velocity control
        KDL::Vector lin_vel_err;
        KDL::Vector ang_vel_err;
        KDL::Twist vel_control_error;
        KDL::Twist velocity_control_signal;

};

#endif