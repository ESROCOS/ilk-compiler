/*
 * Header code generated by the ILK-compiler, C++/Eigen backend
 * File generated on: Mon Nov 19 21:48:51 2018 (UTC)
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

    kul::pose_t camera__fr_link2;
    kul::pose_t fr_link1__fr_jB;
    kul::pose_t fr_link2__fr_jC;
    kul::pose_t fr_link3__fr_jD;
};

void fk1(const ModelConstants& mc, const joint_state& q, kul::pose_t& fr_link1__fr_link3, kul::pose_t& fr_jA__fr_link4, kul::pose_t& fr_jA__fr_jC);

}

#endif