#ifndef EU_ESROCOS_KUL_CODEGENERATOR_ROBOT_UR5_DEFS_H
#define EU_ESROCOS_KUL_CODEGENERATOR_ROBOT_UR5_DEFS_H

#include <ilk/eigen/core-types.h>

namespace ur5 {

constexpr unsigned int dofs_count = 5;
typedef kul::Matrix< dofs_count, 1> joint_state;

} // robot namespace

#endif
