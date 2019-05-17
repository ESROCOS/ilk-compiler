#ifndef KUL_ESROCOS_ILK_GENERATOR_COORDINATE_TRANSFORM_EIGEN_H
#define KUL_ESROCOS_ILK_GENERATOR_COORDINATE_TRANSFORM_EIGEN_H


#include "core-types.h"

namespace kul {

typedef pose_t hom_t_t;

/**
 * Coordinate transform of a spatial motion vector.
 *
 * Assumes the given vector is expressed in 'curent' coordinates, and transforms
 * it into 'newframe' coordinates. The relative pose between the two frames must
 * be encoded in the homogeneous transformation matrix 'newframe_H_current'.
*/
void ct_twist(const hom_t_t& newframe_H_current,
              const twist_t& velocity_current,
              twist_t& velocity_newframe);

}

#endif
