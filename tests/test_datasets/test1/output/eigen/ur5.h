/*
 * Header code generated by the ILK-compiler, C++/Eigen backend
 * File generated on: Mon Nov 19 21:57:44 2018 (UTC)
 *
 * License: BSD 2-clause
 */
#ifndef _KINGEN_UR5_H_
#define _KINGEN_UR5_H_

#include <ilk/eigen/core-types.h>
#include <ilk/eigen/joint-transforms.h>
#include <ilk/eigen/ik.h>
#include <ilk/eigen/operators.h>
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

}

#endif
