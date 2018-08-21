#ifndef EU_ESROCOS_KUL_CODEGENERATOR_ROBOT_UR5_DEFS_H
#define EU_ESROCOS_KUL_CODEGENERATOR_ROBOT_UR5_DEFS_H

#include <ilk/eigen/core-types.h>

namespace ur5 {

constexpr unsigned int dofs_count = 6;
typedef kul::Matrix< dofs_count, 1> joint_state;

typedef kul::Matrix<6, dofs_count> Jacobian_t;
typedef Jacobian_t t_J_fr_wrist_3_fr_base;
typedef Jacobian_t t_J_fr_wrist_3_fr_elbow;

} // robot namespace

#endif
