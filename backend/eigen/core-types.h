#ifndef KUL_ESROCOS_ILK_GENERATOR_EIGEN_BACKEND_CORE_TYPES_H
#define KUL_ESROCOS_ILK_GENERATOR_EIGEN_BACKEND_CORE_TYPES_H

#include <Eigen/Core>
#include <Eigen/Dense>

namespace kul {

template<typename Derived>
using MatrixBase = Eigen::MatrixBase< Derived >;

template<int R, int C>
using Matrix = Eigen::Matrix<double, R, C>;

template<typename XprType, int R, int C>
using MatrixBlock = Eigen::Block< XprType, R, C >;

/*Types*/
typedef Matrix<3,1> vector3_t;
typedef Matrix<6,1> vector6_t;
typedef Matrix<4,4> pose_t;
typedef Matrix<4,1> quaternion_t; // TODO change this?
typedef Matrix<3,3> rot_m_t;
typedef MatrixBlock<pose_t,3,3>  rot_m_v;
typedef MatrixBlock<pose_t,3,1>  position_v;
typedef MatrixBlock<pose_t,3,1>  axis_v;
typedef MatrixBlock<const pose_t,3,3>  const_rot_m_v;
typedef MatrixBlock<const pose_t,3,1>  const_position_v;
typedef MatrixBlock<const pose_t,3,1>  const_axis_v;

typedef vector3_t position_t;
typedef vector6_t twist_t;

typedef MatrixBlock<twist_t,3,1>       Part3D;     ///< a 3D subvector of a 6D vector
typedef MatrixBlock<const twist_t,3,1> Part3DConst;///< a const 3D subvector of a 6D vector

/**
 * \name Vector coordinates
 * Constants to index either 6D or 3D coordinate vectors.
 */
///@{
enum Coords3D { X=0, Y, Z};
/// To be used with 6D vectors. 'A' stands for angular, 'L' for linear.
enum Coords6D { AX=0, AY, AZ, LX, LY, LZ };
///@}

/**
 *  \name 6D vectors accessors
 *  These functions allow to access the linear and the angular coordinates of
 *  motion/force vectors.
 */
///@{
inline Part3D angularCoords(vector6_t& f) {
	return f.topRows<3>();
}
inline Part3D linearCoords(vector6_t& f) {
	return f.bottomRows<3>();
}
inline Part3DConst angularCoords(const vector6_t& f) {
    return f.topRows<3>();
}
inline Part3DConst linearCoords(const vector6_t& f) {
    return f.bottomRows<3>();
}
///@}


inline rot_m_v
eg_get_rotation( pose_t& m)
{
  return m.block<3,3>(0,0);
}

inline const_rot_m_v
get_rotation_const(const pose_t& m)
{
  return m.block<3,3>(0,0);
}

inline position_v
eg_get_position(pose_t& m)
{
  return m.block<3,1>(0,3);
}

inline const_position_v
eg_get_position(const pose_t& m)
{
  return m.block<3,1>(0,3);
}

inline const_position_v
get_position_const(const pose_t& m)
{
  return m.block<3,1>(0,3);
}

inline const_axis_v
eg_get_zaxis(const pose_t& H)
{
	return H.block<3,1>(0,Z);
}

inline void eg_set_position(Eigen::Block<Eigen::Matrix4d,1,3>& p,
                  double x, double y, double z) {
  p(0) = x;
  p(1) = y;
  p(2) = z;
}

inline void eg_set_position(pose_t& p,
                  double x, double y, double z) {
  p(0,3) = x; p(1,3) = y; p(2,3) =z;
}

inline void eg_set_rotation(pose_t& r,
                  double Xx, double Xy, double Xz,
                  double Yx, double Yy, double Yz,
                  double Zx, double Zy, double Zz) {
  r(0,0) = Xx; r(0,1) = Xy; r(0,2) = Xz;
  r(1,0) = Yx; r(1,1) = Yy; r(1,2) = Yz;
  r(2,0) = Zx; r(2,1) = Zy; r(2,2) = Zz;
}

inline void eg_set_rotation(Eigen::Block<Eigen::Matrix4d,3,3>& r,
                  double Xx, double Xy, double Xz,
                  double Yx, double Yy, double Yz,
                  double Zx, double Zy, double Zz) {
  r(0,0) = Xx; r(0,1) = Xy; r(0,2) = Xz;
  r(1,0) = Yx; r(1,1) = Yy; r(1,2) = Yz;
  r(2,0) = Zx; r(2,1) = Zy; r(2,2) = Zz;
}

inline void eg_set_pose(Eigen::Matrix4d& m,
   double x,  double y,  double z,
   double Xx, double Xy, double Xz,
   double Yx, double Yy, double Yz,
   double Zx, double Zy, double Zz) {
  m(0,0) = x;    m(1) = y;    m(2) = z;
  m(0,0) = Xx; m(0,1) = Xy; m(0,2) = Xz;
  m(0,1) = Yx; m(1,1) = Yy; m(1,2) = Yz;
  m(0,2) = Zx; m(1,2) = Zy; m(2,2) = Zz;
}

}

#endif
