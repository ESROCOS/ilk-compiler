#include <iostream>
#include <ilk/eigen/misc.h>
#include "ur5.h"

static kul::RoundTheDifference<double> rounder(0.1);

int main(int argc, char** argv)
{
    ur5::ModelConstants mc;
    ur5::joint_state q;
    ur5::joint_state qd;
    kul::pose_t wrist_3__base;
    kul::twist_t v__wrist_3__base;
    ur5::Jacobian_t J_wrist_3_base;

    // Set your inputs here
    q.setZero();

    ur5::solver1(mc, q, qd, wrist_3__base, v__wrist_3__base, J_wrist_3_base);

    //std::cout << std::endl << wrist_3__base << std::endl;
    //std::cout << std::endl << v__wrist_3__base << std::endl;
    //std::cout << std::endl << J_wrist_3_base << std::endl;

    // Print the difference between quantities with something like
    // std::cout << kul::roundedDifference(A,B, rounder) << std::endl;

    return 0;
}