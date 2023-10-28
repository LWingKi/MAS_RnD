#include <kdl/chain.hpp>
#include "robot.hpp"
// #include <utilities/utility.h>

namespace KDL{
    Chain Kinova_gen3(){
        Chain kinova_gen3;
        //joint 1
        kinova_gen3.addSegment(Segment(Joint(Joint::RotZ),
                            Frame::DH(0, PI/2, -(0.1564 + 0.1284), 0),
                            RigidBodyInertia(1.377,Vector(-0.000023, -0.010364, -0.073360),RotationalInertia(0.004570, 0.000001, 0.000002, 0.004831, 0.000448, 0.001409))));
        //joint 2
        kinova_gen3.addSegment(Segment(Joint(Joint::RotZ),
                            Frame::DH(0, PI/2, -(0.0054 + 0.0064), PI),
                            RigidBodyInertia(1.1636,Vector(-0.000044, -0.099580, -0.013278),RotationalInertia(0.011088, 0.000005, 0, 0.001072,-0.000691,0.011255))));
        //joint 3
        kinova_gen3.addSegment(Segment(Joint(Joint::RotZ),
                            Frame::DH(0, PI/2, -(0.2104 + 0.2104), PI),
                            RigidBodyInertia(1.1636,Vector(-0.000044, -0.006641, -0.117892),RotationalInertia(0.010932, 0, -0.000007, 0.011127, 0.000606, 0.001043))));
        //joint 4
        kinova_gen3.addSegment(Segment(Joint(Joint::RotZ),
                            Frame::DH(0, PI/2, -(0.0064 + 0.0064), PI),
                            RigidBodyInertia(0.930,Vector(-0.000018, -0.075478, -0.015006),RotationalInertia(0.008147, -0.000001, 0, 0.000631, -0.000500, 0.008316))));
        //joint 5
        kinova_gen3.addSegment(Segment(Joint(Joint::RotZ),
                            Frame::DH(0, PI/2, -(0.2084 + 0.1059), PI),
                            RigidBodyInertia(0.678, Vector(0.000001, -0.009432, -0.063883),RotationalInertia(0.001596, 0, 0, 0.001607, 0.000256, 0.000399))));
        //joint 6
        kinova_gen3.addSegment(Segment(Joint(Joint::RotZ),
                            Frame::DH(0, PI/2, 0, PI),
                            RigidBodyInertia(0.678,Vector(0.000001, -0.045483, -0.009650),RotationalInertia(0.001641, 0, 0, 0.000410, -0.000278, 0.001641))));
        //end effector
        kinova_gen3.addSegment(Segment(Joint(Joint::RotZ),
                            Frame::DH(0, PI, -(0.1059 + 0.0615), PI),
                            RigidBodyInertia(0.364,Vector(-0.000093, 0.000132, -0.022905),RotationalInertia(0.000214, 0, 0.000001, 0.000223, -0.000002, 0.000240))));
        return kinova_gen3;
    }
}