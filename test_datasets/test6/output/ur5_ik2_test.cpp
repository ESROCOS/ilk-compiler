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

void test_ik_pos(std::ofstream& log)
{
	joint_state q, q_guess, q_rand, q_ik;

	// Generate a random set point in the ee pose space
	kul::pose_t fr_wrist_3__wrt__fr_base;
	Jacobian_t J;
	q.setRandom();
	q_rand.setRandom();
	q_guess = q + q_rand/10;

	fk__ik1(kk, q, fr_wrist_3__wrt__fr_base, J);
	vector3_t desp = eg_get_position(fr_wrist_3__wrt__fr_base);
	rot_m_t   deso = eg_get_rotation(fr_wrist_3__wrt__fr_base);

	// Call IK for position
	kul::ik_pos_cfg cfg;
	kul::ik_pos_dbg dbg;
	cfg.max_iter = 500;
	cfg.eps_or_err_norm = 1e-3;
	cfg.ls_damping = 0.08;
	ik2(kk, cfg, desp, deso, q_guess, q_ik, dbg);

	Matrix<6,4> prettyPrint;

	prettyPrint.col(0) = q;
	prettyPrint.col(1) = q_guess;
	prettyPrint.col(2) = q_ik;
	prettyPrint.col(3) = roundedDifference( q, q_ik, rounder );

	cout << "ee pos error:\t" << roundedDifference(desp, dbg.actual_pos, rounder).transpose() << endl;
	cout << "ee or  error:\t" << kul::orientationDistance(dbg.actual_or, deso).angle << endl << endl;
	cout << "q | q_guess | q_ik | q - q_ik" << endl;
	cout << prettyPrint << endl;

	cout << endl << "iterations count: " << dbg.iter_count << endl;

	log << "ik.q_des = [" << q.transpose() << "];" << endl;
	log << "ik.q_guess = [" << q_guess.transpose() << "];" << endl;
	log << "ik.q_found = [" << q_ik.transpose() << "];" << endl;
}

int main()
{
    std::ofstream octave("ur5_ik2_log.m");
	std::srand((unsigned int) time(0));
	test_ik_pos(octave);

	return 0;
}
