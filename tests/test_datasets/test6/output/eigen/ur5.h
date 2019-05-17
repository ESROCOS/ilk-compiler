/*
 * Header code generated by the ILK-compiler, C++/Eigen backend
 * File generated on: Thu Dec 20 14:37:07 2018 (UTC)
 *
 * License: BSD 2-clause
 */
#ifndef _KINGEN_UR5_H_
#define _KINGEN_UR5_H_

#include <ilk/eigen/core-types.h>
#include <ilk/eigen/joint-transforms.h>
#include <ilk/eigen/ik.h>
#include "robot-defs.h"

namespace ur5 {

struct ModelConstants
{
    ModelConstants();

    kul::pose_t fr_elbow__fr_upper_arm;
    kul::pose_t fr_shoulder_lift__fr_shoulder;
    kul::pose_t fr_shoulder_pan__fr_base;
    kul::pose_t fr_wr1__fr_forearm;
    kul::pose_t fr_wr2__fr_wrist_1;
    kul::pose_t fr_wr3__fr_wrist_2;
};

void fk1(const ModelConstants& mc, const joint_state& q, kul::pose_t& fr_wrist_3__fr_base);
void fk2(const ModelConstants& mc, const joint_state& q, kul::pose_t& fr_forearm__fr_base);
void fk__ik1(const ModelConstants& mc, const joint_state& q, kul::pose_t& fr_wrist_3__fr_base, Jacobian_t& J_fr_wrist_3_fr_base);
void ik1(const ModelConstants& mc, const joint_state& q, const kul::vector3_t& v, joint_state& q_ik);
void ik2(const ModelConstants& mc, const kul::ik_pos_cfg& cfg, const kul::position_t& desp, const kul::rot_m_t& deso, const joint_state& q_guess, joint_state& q_ik, kul::ik_pos_dbg& dbg);

}

#endif