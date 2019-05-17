#include <iostream>
#include <fstream>

#include <ilk/eigen/gjac.h>
#include <ilk/eigen/ik.h>
#include <ilk/eigen/misc.h>
#include <ilk/eigen/rots.h>

#include "ur5.h"

using namespace std;

static kul::RoundTheDifference<double> rounder(0.1);
static ur5::ModelConstants mc;

void test_ik_pos(std::ofstream& log)
{
    ur5::joint_state q;
    kul::pose_t fr_wrist_3__fr_base;
    ur5::Jacobian_t J_fr_wrist_3_fr_base;
    ur5::joint_state q_guess, q_rand, q_ik;

    // Generate a random set point in the ee pose space
    q.setRandom();
    q_rand.setRandom();
    q_guess = q + q_rand/10;

    ur5::fk__ik1(mc, q, fr_wrist_3__fr_base, J_fr_wrist_3_fr_base);
    kul::position_t desp = kul::eg_get_position(fr_wrist_3__fr_base);
    kul::rot_m_t   deso = kul::eg_get_rotation(fr_wrist_3__fr_base);

    // Call IK for position
    kul::ik_pos_cfg ikcfg;
    kul::ik_pos_dbg dbg;
    ikcfg.max_iter = 500;
    ikcfg.eps_or_err_norm = 1e-3;
    ikcfg.ls_damping = 0.08;
    ur5::ik2(mc, ikcfg, desp, deso, q_guess, q_ik, dbg);

    kul::Matrix<ur5::dofs_count,4> prettyPrint;

    prettyPrint.col(0) = q;
    prettyPrint.col(1) = q_guess;
    prettyPrint.col(2) = q_ik;
    prettyPrint.col(3) = kul::roundedDifference( q, q_ik, rounder );

    cout << "ee pos error:\t" << kul::roundedDifference(desp, dbg.actual_pos, rounder).transpose() << endl;
    cout << "ee or  error:\t" << kul::orientationDistance(dbg.actual_or, deso).angle << endl << endl;
    cout << "q | q_guess | q_ik | q - q_ik" << endl;
    cout << prettyPrint << endl;

    cout << endl << "iterations count: " << dbg.iter_count << endl;

    log << "ik.q_des   = [" << q.transpose() << "];" << endl;
    log << "ik.q_guess = [" << q_guess.transpose() << "];" << endl;
    log << "ik.q_found = [" << q_ik.transpose() << "];" << endl;
}

int main()
{
    std::ofstream octave("octave_log.m");
    std::srand((unsigned int) time(0));
    test_ik_pos(octave);

    return 0;
}
