#include <iostream>
#include <fstream>

#include <ilk/eigen/gjac.h>
#include <ilk/eigen/ik.h>
#include <ilk/eigen/misc.h>
#include <ilk/eigen/rots.h>

#include "ur5.h"

static kul::RoundTheDifference<double> rounder(0.1);
static ur5::ModelConstants mc;

void test_ik_vel()
{
     ur5::joint_state q;
     kul::pose_t fr_wrist_3__fr_base;
     ur5::Jacobian_t J_fr_wrist_3_fr_base;

     q.setRandom();
     ur5::fk__ik1(mc, q, fr_wrist_3__fr_base, J_fr_wrist_3_fr_base);

     ur5::joint_state qd, qd_ik;
     qd.setRandom();

     kul::twist_t twist = J_fr_wrist_3_fr_base * qd;

     ur5::ik1(mc, q, kul::linearCoords(twist), qd_ik);

     std::cout << kul::roundedDifference(twist, J_fr_wrist_3_fr_base*qd_ik, rounder).transpose() << std::endl;
}

int main()
{
	test_ik_vel();
	return 0;
}