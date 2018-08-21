#include "ur5.h"

#include <ilk/eigen/gjac.h>
#include <ilk/eigen/ik.h>
#include <ilk/eigen/misc.h>
#include <ilk/eigen/rots.h>

#include <iostream>
#include <fstream>

using namespace std;
using namespace ur5;
using namespace kul;


static kul::RoundTheDifference<double> rounder(0.1);
static mc_config kk;

void test_ik_vel()
{
     joint_state q;
     kul::pose_t fr_wrist_3__wrt__fr_base;
     Jacobian_t J;

     q.setRandom();
     fk__ik1(kk, q, fr_wrist_3__wrt__fr_base, J);

     joint_state qd, qd_ik;
     qd.setRandom();

     vector6_t twist = J * qd;

     ik1(kk, q, kul::linearCoords(twist), qd_ik);

     cout << kul::linearCoords(kul::roundedDifference(twist, J*qd_ik, rounder)).transpose() << endl;
}


int main()
{
	test_ik_vel();
	return 0;
}
