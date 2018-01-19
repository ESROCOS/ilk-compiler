#ifndef KUL_ESROCOS_ILK_GENERATOR_JOINT_TRANSFORM_EIGEN_H
#define KUL_ESROCOS_ILK_GENERATOR_JOINT_TRANSFORM_EIGEN_H

//namespace kul { //?

#include "core-types.h"

typedef pose_t hom_t_t;
/*
 * B is a frame rotated/translated by an angle/displacement with respect to A.
 * A_X_B or B_X_A are different transformation matrices then.
 */

void rot_z__a_x_b(double arg, hom_t_t& out);
void rot_z__b_x_a(double arg, hom_t_t& out);

void tr_z__a_x_b(double arg, hom_t_t& out);
void tr_z__b_x_a(double arg, hom_t_t& out);


//}

#endif
