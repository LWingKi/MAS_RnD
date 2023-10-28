#include <kdl/chain.hpp>
#include "robot.hpp"

namespace KDL{
Chain Create_gen3(){
        Chain create_gen3;

        // joint 1
        Frame tip1(Rotation(1.0, 0.0,  0.0,
                                0.0, 0.0, -1.0,
                                0.0, 1.0,  0.0),
                Vector(0.0, 0.0054, -0.1284));
        create_gen3.addSegment(Segment(
                Joint(Joint::RotZ),
                tip1,
                tip1.Inverse() * RigidBodyInertia(1.377,
                        Vector(-0.000023, -0.010364, -0.073360),
                        RotationalInertia(0.004570, 0.004831, 0.001409, 0.000001, 0.000002, 0.000448))));

        // joint 2
        Frame tip2(Rotation(1.0,  0.0, 0.0,
                                0.0,  0.0, 1.0,
                                0.0, -1.0, 0.0),
                Vector(0.0, -0.2104, -0.0064));
        create_gen3.addSegment(Segment(
                Joint(Joint::RotZ),
                tip2,
                tip2.Inverse() * RigidBodyInertia(1.1636,
                        Vector(-0.000044, -0.09958, -0.013278),
                        RotationalInertia(0.011088, 0.001072, 0.011255, 0.000005, 0.000000, -0.000691))));

        // joint 3
        Frame tip3(Rotation(1.0, 0.0,  0.0,
                                0.0, 0.0, -1.0,
                                0.0, 1.0,  0.0),
                // According to transformation matrices in manual
                //Vector(0.0, -0.0064, -0.2104)),
                // According to DH parameters (and drawing) in manual
                // Also in URDF model:
                // <https://github.com/Kinovarobotics/ros_kortex/blob/noetic-devel/kortex_description/arms/gen3/7dof/urdf/GEN3_URDF_V12.urdf#L134>
                Vector(0.0, 0.0064, -0.2104));
        create_gen3.addSegment(Segment(
                Joint(Joint::RotZ),
                tip3,
                tip3.Inverse() * RigidBodyInertia(1.1636,
                        Vector(-0.000044, -0.006641, -0.117892),
                        RotationalInertia(0.010932, 0.011127, 0.001043, 0.000000, -0.000007, 0.000606))));

        // joint 4
        Frame tip4(Rotation(1.0,  0.0, 0.0,
                                0.0,  0.0, 1.0,
                                0.0, -1.0, 0.0),
                Vector(0.0, -0.2084, -0.0064));
        create_gen3.addSegment(Segment(
                Joint(Joint::RotZ),
                tip4,
                tip4.Inverse() * RigidBodyInertia(0.930,
                        Vector(-0.000018, -0.075478, -0.015006),
                        RotationalInertia(0.008147, 0.000631, 0.008316, -0.000001, 0.000000, -0.000050))));

        // joint 5
        Frame tip5(Rotation(1.0, 0.0,  0.0,
                                0.0, 0.0, -1.0,
                                0.0, 1.0,  0.0),
                Vector(0.0, 0.0, -0.1059));
        create_gen3.addSegment(Segment(
                Joint(Joint::RotZ),
                tip5,
                tip5.Inverse() * RigidBodyInertia(0.678,
                        Vector(0.000001, -0.009432, -0.063883),
                        RotationalInertia(0.001596, 0.001607, 0.000399, 0.000000, 0.000000, 0.000256))));

        // joint 6
        Frame tip6(Rotation(1.0,  0.0, 0.0,
                                0.0,  0.0, 1.0,
                                0.0, -1.0, 0.0),
                Vector(0.0, -0.1059, 0.0));
        create_gen3.addSegment(Segment(
                Joint(Joint::RotZ),
                tip6,
                tip6.Inverse() * RigidBodyInertia(0.678,
                        Vector(0.000001, -0.045483, -0.009650),
                        RotationalInertia(0.001641, 0.000410, 0.001641, 0.000000, 0.000000, -0.000278))));

        // joint 7
        Frame tip7(Rotation(1.0,  0.0,  0.0,
                                0.0, -1.0,  0.0,
                                0.0,  0.0, -1.0),
                Vector(0.0, 0.0, -0.0615));
        create_gen3.addSegment(Segment(
                Joint(Joint::RotZ),
                tip7,
                tip7.Inverse() * RigidBodyInertia(0.500,
                        Vector(-0.000281, -0.011402, -0.029798),
                        RotationalInertia(0.000587, 0.000369, 0.000609, 0.000003, 0.000003, 0.000118))));

        return create_gen3;
        }
}
