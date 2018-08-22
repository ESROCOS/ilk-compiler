#include "core-types.h"
#include "rots.h"
#include "misc.h"

#include <iostream>
#include <ctime>
using namespace std;


kul::RoundTheDifference<double> rounder(0.1);

template<typename D1, typename D2>
kul::DiffReturnType<D1,D2> diff(const kul::MatrixBase<D1>& m1, const kul::MatrixBase<D2>& m2)
{
	return roundedDifference(m1, m2, rounder);
}

void testOrientationError()
{
	using namespace kul;
	// Generate a random axis/angle pair
	vector3_t axis;
	axis.setRandom();
	axis = axis / axis.norm();
	double theta = 0;//static_cast<double>(std::rand())/RAND_MAX * M_PI;

	// Now convert the axis/angle to rotation matrix
	double c = cos(theta);
	double s = sin(theta);
	double v = 1 - c;
	double x = axis(X);
	double y = axis(Y);
	double z = axis(Z);

	rot_m_t R;
	R << x*x*v+c  , x*y*v-z*s, x*z*v+y*s,
			     x*y*v+z*s, y*y*v + c, y*z*v-x*s,
				 x*z*v-y*s, y*z*v+x*s, z*z*v+c;

	// The orientation difference between the Identity and R
	// should be exactly the axis/angle we used in the first place
	rot_m_t I = rot_m_t::Identity();
	AxisAngle res = orientationDistance(I, I);

	cout << "Actual axis: " << axis.transpose() << endl;
	cout << "Found axis : " << res.axis.transpose() << endl;
	cout << "Axes diff  : " << diff(axis, res.axis).transpose() << endl;
	cout << "Angle (actual, found): " << theta << "  " << res.angle << endl;
//
//	cout << 0.5 *(I.col(0).cross(R.col(0)) + I.col(1).cross(R.col(1)) + I.col(2).cross(R.col(2))).transpose() << endl;
//	cout << (axis * sin(theta)).transpose() << endl;
}


int main()
{
	std::srand(std::time(nullptr));
	testOrientationError();
	return 0;
}