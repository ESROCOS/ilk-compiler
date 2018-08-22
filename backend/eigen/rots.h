/*
 *  Author: Marco Frigerio
 */

#ifndef KUL_ILK_ROTATIONUTILS_H_
#define KUL_ILK_ROTATIONUTILS_H_

#include "core-types.h"

namespace kul {

struct AxisAngle {
	vector3_t axis;
	double    angle;
	AxisAngle() : axis(vector3_t(1.0,0.0,0.0)), angle(0.0) {}
	AxisAngle(const vector3_t& a, double th) : axis(a/a.norm()), angle(th) {}
	vector3_t omega() { return axis*angle; }
};

/**
 * The difference between two rotation matrices, as an axis-angle.
 *
 * \return the rotation required to go from the second to the first argument,
 * that is, the first "minus" the second.
 */
template<typename D1, typename D2>
AxisAngle orientationDistance(const D1& _R_desired, const D2& _R_actual)
{
    static constexpr double thresh = 1e-6;//TODO #magic-number
    //TODO compile time checks on the size, although the following already fails if they are not 3x3
	rot_m_t R = _R_actual.transpose() * _R_desired; // this is 'actual_R_desired'

	double x = R(Z,Y) - R(Y,Z);
	double y = R(X,Z) - R(Z,X);
	double z = R(Y,X) - R(X,Y);
	double norm = sqrt(x*x + y*y + z*z);
	if(norm < thresh) {
	    return AxisAngle(vector3_t(0.0, 0.0, 1.0), 0.0); // arbitrary choice of (0,0,1) axis
	}
	double theta= atan2( norm, R.trace()-1 );
	return AxisAngle(vector3_t(x/norm, y/norm, z/norm), theta);
	//TODO check corner cases, theta close to 0/PI, bad numerical
}


}

#endif
