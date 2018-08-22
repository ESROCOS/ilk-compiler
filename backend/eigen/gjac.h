/*
 *  Author: Marco Frigerio
 */

#ifndef KUL_ILK_GEOMETRICJACOBIANS_H_
#define KUL_ILK_GEOMETRICJACOBIANS_H_

#include "core-types.h"

namespace kul {

namespace internal {

template<typename Derived>
inline
MatrixBase<Derived>& constCast(const MatrixBase<Derived>& x) {
    return const_cast<MatrixBase<Derived>&>(x);
}

}

#define block3x1 template block<3,1>



/**
 * \name Columns of geometric Jacobians
 */
///@{
/**
 * \param[in] poi Point-of-Interest, the Cartesian coordinates of the point whose
 *        velocity is of interest, expressed in some reference frame B
 * \param[in] jointOrigin, the Cartesian coordinates of the origin of the reference
 *        frame of the joint, expressed in reference frame B.
 * \param[in] jointAxis, the 3D unit vector aligned with the joint axis, expressed
 *        in reference frame B.
 * \param[out] column the column of the Jacobian corresponding to the joint
 */
template<typename Derived>
inline void geometricJacobianColumn_revolute(
        const position_t& poi,
        const position_t& jointOrigin,
        const vector3_t&  jointAxis,
        const MatrixBase<Derived>& column)
{
    //matrix_assert(column.rows() == 6  &&  column.cols() == 1);
    internal::constCast(column).block3x1(AX,0) = jointAxis;
    internal::constCast(column).block3x1(LX,0) = jointAxis.cross(poi - jointOrigin);
}

/**
 *
 */
template<typename Derived>
inline void geometricJacobianColumn_prismatic(
        const vector3_t& jointAxis,
        const MatrixBase<Derived>& column)
{
    //matrix_assert(column.rows() == 6  &&  column.cols() == 1);
    internal::constCast(column).block3x1(AX,0).setZero();
    internal::constCast(column).block3x1(LX,0) = jointAxis;
}
///@}

#undef block3x1


template<typename D1, typename D2, typename D3>
inline void leastSquaresSolve(const MatrixBase<D1>& J, const MatrixBase<D2>& v, const MatrixBase<D3>& out)
{
	Eigen::JacobiSVD<D1> svd( J, Eigen::ComputeFullU | Eigen::ComputeFullV);
	internal::constCast(out) = svd.solve( v );
}

template<typename D1, typename D2, typename D3>
inline void dampedLeastSquaresSolve(const MatrixBase<D1>& J, const MatrixBase<D2>& v, const MatrixBase<D3>& out, double damping)
{
	Eigen::JacobiSVD<D1> svd( J, Eigen::ComputeFullU | Eigen::ComputeFullV);
	auto svalues = svd.singularValues();
	double tmp = 0;
	typename Eigen::JacobiSVD<D1>::SingularValuesType evalues;
	for(int i=0; i<svd.rank(); i++) {
		tmp = svalues(i);
		evalues(i) = tmp / (tmp*tmp + damping*damping);
	}
	internal::constCast(out) = svd.matrixV() * evalues.asDiagonal() * svd.matrixU().transpose() * v;
}

}

#endif
