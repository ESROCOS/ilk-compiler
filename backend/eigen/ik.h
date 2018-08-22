#ifndef KUL_ESROCOS_ILK_GENERATOR_EIGEN_BACKEND_IK_UTILS
#define KUL_ESROCOS_ILK_GENERATOR_EIGEN_BACKEND_IK_UTILS

#include "core-types.h"
#include "rots.h"

namespace kul
{

struct ik_pos_cfg {
	double dt = 0.004;
	double eps_pos_err_norm = 1E-6;
	double eps_or_err_norm = 1E-6;
	unsigned int max_iter = 100;
	double ls_damping = 0.05; // damped-least-squares parameter
};

struct ik_pos_dbg {
	unsigned int iter_count;
	vector3_t actual_pos;
	rot_m_t   actual_or;
};

struct ik_cs_err {
	vector3_t pos;
	AxisAngle orient;
};

}



#endif

