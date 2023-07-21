#include "controller.hpp"

PController::PController(double k_p,double k_i,double k_d):
    k_p(k_p),k_i(k_i),k_d(k_d){}

KDL::Twist PController::getError(KDL::Frame currentpos,KDL::Frame tragetpos){
    pos_control_error = KDL::diff(tragetpos,currentpos);
    return pos_control_error;
}

KDL::Twist PController::getError(KDL::Twist currentvel,KDL::Twist tragetvel){
    vel_control_error = KDL::diff(currentvel,tragetvel);
    return vel_control_error;
}

//Position control
KDL::Twist PController::getControlSignal(KDL::Frame currentpos,KDL::Frame tragetpos,double dt){
    pos_control_error = KDL::diff(currentpos,tragetpos);
    pos_control_signal.vel = pos_control_error.vel*k_p;
    pos_control_signal.rot = pos_control_error.rot*k_p;
    return pos_control_signal;
}
//Velocity control
KDL::Twist PController::getControlSignal(KDL::Twist currentvel,KDL::Twist tragetvel,double dt){
    vel_control_error = KDL::diff(currentvel,tragetvel);
    velocity_control_signal.vel = vel_control_error.vel * k_p;
    velocity_control_signal.rot = vel_control_error.rot* k_p;
    return velocity_control_signal;
}
