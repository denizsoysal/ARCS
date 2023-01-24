#include <chain.hpp>
#include <chainiksolver.hpp>
#include <chainfksolverpos_recursive.hpp>
#include <frames_io.hpp>
#include <stdio.h>
#include <iostream>

#include <chainidsolver_recursive_newton_euler.hpp>

#include <models.hpp>
 
using namespace KDL;
 
KDL::Chain kdl_kin_chain = KDL::KukaIIWA14();
int main( int argc, char** argv )
{
    Chain iiwa_robot=KukaIIWA14();

    JntArray q(iiwa_robot.getNrOfJoints());
    JntArray qd(iiwa_robot.getNrOfJoints());
    JntArray qdd(iiwa_robot.getNrOfJoints());
    JntArray tau(iiwa_robot.getNrOfJoints());
    Wrenches f(iiwa_robot.getNrOfSegments());

    Vector gravity = Vector(0.0,0.0,-9.81);

    // Forward Kinematics Solver
    ChainFkSolverPos_recursive fksolver(iiwa_robot);

    // Inverse Dynamics Solver
    ChainIdSolver_RNE idsolver(iiwa_robot,gravity);

    // Sample joint positions [rad], joint velocities [rad/s] and joint accelerations [rad/s^2]
    q(0) = 0; q(1) = 0; q(2) = 0; q(3) = -PI/2; q(4) = 0; q(5) = PI/2; q(6) = 0; 
    qd(0) = 0.1; qd(1) = 0; qd(2) = 0.2; qd(3) = 0; qd(4) = 0.3; qd(5) = -0.4; qd(6) = 0.0; 
    qdd(0) = 0.2; qdd(1) = 0; qdd(2) = -0.1; qdd(3) = 0.1; qdd(4) = 0.0; qdd(5) = 0.0; qdd(6) = 0.0;

    // Cartesian frame computation
    Frame T_ee;
    fksolver.JntToCart(q,T_ee);
    std::cout<<T_ee<<std::endl;

    // Joint torque computation
    idsolver.CartToJnt(q,qd,qdd,f,tau);
    std::cout<<"------------- Joint Torques --------------\n tau_1: " << tau(0)<<" tau_2: "<<tau(1)<<"tau_3: "<<
                            tau(2)<<" tau_4: "<<tau(3)<<" tau_5: "<<tau(4)<<" tau_6: "<<tau(5)<<" tau_7: "<<tau(6)<<std::endl;
}
