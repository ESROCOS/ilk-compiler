#ifndef KUL_ESROCOS_ILK_GENERATOR_EIGEN_BACKEND_CORE_TYPES_H
#define KUL_ESROCOS_ILK_GENERATOR_EIGEN_BACKEND_CORE_TYPES_H

#include <Eigen/Core>

/*Types*/
typedef Eigen::Matrix4d pose_t;
typedef Eigen::Vector3d position_t;
typedef Eigen::Vector3d quaternion_t;
typedef Eigen::Matrix3d rot_m_t;
typedef Eigen::Block<Eigen::Matrix4d,3,3>  rot_m_v;
typedef Eigen::Block<Eigen::Matrix4d,1,3>  position_v;


inline rot_m_v
eg_get_rotation( Eigen::Matrix4d& m)
{
  return m.block<3,3>(0,0);
}


inline position_v
eg_get_position(Eigen::Matrix4d& m)
{
  return m.block<1,3>(3,0);
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

#endif
