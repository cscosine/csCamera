#pragma once
#include <cassert>
#include <csCamera/Utils.hpp>

namespace csCamera {

// 3D cartesian point to point on z=1
template <class Derived1, class Derived2>
void Utils::points3D_to_z1(const Eigen::MatrixBase<Derived1>& source, Eigen::MatrixBase<Derived2>& ret) {
  EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(Eigen::Matrix3Xf, Derived1);
  EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(Eigen::Matrix2Xf, Derived2);
  assert(source.rows() == 3);
  assert(ret.rows() == 2);
  assert(source.cols() == ret.cols());
  ret.row(0) = source.row(0).array() / source.row(2).array();
  ret.row(1) = source.row(1).array() / source.row(2).array();
}

// 3d point to z=1 plane for single point with jacobian
template <class Derived1, class Derived2>
Eigen::Matrix<typename Derived1::Scalar, 2, 1> Utils::points3D_to_z1_jacob(const Eigen::MatrixBase<Derived1>& in,
                                                                           Eigen::MatrixBase<Derived2>& d_out_d_in) {
  Eigen::Matrix<typename Derived1::Scalar, 2, 1> ret(in.x() / in.z(), in.y() / in.z());
  typename Derived1::Scalar z2 = in.z() * in.z();

  d_out_d_in(0, 0) = 1 / in.z();
  d_out_d_in(0, 1) = 0;
  d_out_d_in(0, 2) = -in.x() / z2;

  d_out_d_in(1, 0) = 0;
  d_out_d_in(1, 1) = 1 / in.z();
  d_out_d_in(1, 2) = -in.y() / z2;

  return ret;
}

template <class Derived1, class Derived2>
void Utils::unitSpherePoint_to_z1_mei(const Eigen::MatrixBase<Derived1>& source, typename Derived1::Scalar epsilon,
                                      Eigen::MatrixBase<Derived2>& ret) {
  EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(Eigen::Matrix3Xf, Derived1);
  EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(Eigen::Matrix2Xf, Derived2);

  assert(source.rows() == 3);
  assert(ret.rows() == 2);
  assert(source.cols() == ret.cols());

  ret.row(0) = source.row(0).array() / (source.row(2).array() + epsilon);
  ret.row(1) = source.row(1).array() / (source.row(2).array() + epsilon);
}

// mei h function for single point with z check
template <class Derived>
Eigen::Matrix<typename Derived::Scalar, 2, 1> Utils::unitSpherePoint_to_z1_mei(const Eigen::MatrixBase<Derived>& in,
                                                                               typename Derived::Scalar epsilon, bool& zOk) {
  typename Derived::Scalar z_e = in.z() + epsilon;
  typename Derived::Scalar z_e_sq = z_e * z_e;

  zOk = z_e > 0;

  Eigen::Matrix<typename Derived::Scalar, 2, 1> ret(in.x() / z_e, in.y() / z_e);

  return ret;
}

// mei h function for single point with jacobian
template <class Derived1, class Derived2, class Derived3>
Eigen::Matrix<typename Derived1::Scalar, 2, 1>
Utils::unitSpherePoint_to_z1_mei_jacob(const Eigen::MatrixBase<Derived1>& in, typename Derived1::Scalar epsilon,
                                       Eigen::MatrixBase<Derived2>& d_out_d_in, Eigen::MatrixBase<Derived3>& d_out_d_epsilon) {

  typename Derived1::Scalar z_e = in.z() + epsilon;
  typename Derived1::Scalar z_e_sq = z_e * z_e;
  Eigen::Matrix<typename Derived1::Scalar, 2, 1> ret(in.x() / z_e, in.y() / z_e);

  d_out_d_in(0, 0) = 1 / z_e;
  d_out_d_in(0, 1) = 0;
  d_out_d_in(0, 2) = -in.x() / z_e_sq;

  d_out_d_in(1, 0) = 0;
  d_out_d_in(1, 1) = 1 / z_e;
  d_out_d_in(1, 2) = -in.y() / z_e_sq;

  d_out_d_epsilon = d_out_d_in.col(2);

  return ret;
}

// mei h function for single point with jacobian (source only)
template <class Derived1, class Derived2>
Eigen::Matrix<typename Derived1::Scalar, 2, 1> Utils::unitSpherePoint_to_z1_mei_jacob(const Eigen::MatrixBase<Derived1>& in,
                                                                                      typename Derived1::Scalar epsilon,
                                                                                      Eigen::MatrixBase<Derived2>& d_out_d_in) {
  typename Derived1::Scalar z_e = in.z() + epsilon;
  typename Derived1::Scalar z_e_sq = z_e * z_e;
  Eigen::Matrix<typename Derived1::Scalar, 2, 1> ret(in.x() / z_e, in.y() / z_e);

  d_out_d_in(0, 0) = 1 / z_e;
  d_out_d_in(0, 1) = 0;
  d_out_d_in(0, 2) = -in.x() / z_e_sq;

  d_out_d_in(1, 0) = 0;
  d_out_d_in(1, 1) = 1 / z_e;
  d_out_d_in(1, 2) = -in.y() / z_e_sq;
  return ret;
}

template <class Derived1, class Derived2>
void Utils::pointsZ1_to_unitSpherePoint_mei(const Eigen::MatrixBase<Derived1>& in, typename Derived1::Scalar epsilon,
                                            Eigen::MatrixBase<Derived2>& ret) {
  EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(Eigen::Matrix2Xf, Derived1);
  EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(Eigen::Matrix3Xf, Derived2);
  using S = typename Derived1::Scalar;

  Eigen::Matrix<S, 1, Derived1::ColsAtCompileTime> x2y2 = in.colwise().squaredNorm();
  Eigen::Matrix<S, 1, Derived1::ColsAtCompileTime> frac =
      (epsilon + (1 + (1 - epsilon * epsilon) * x2y2.array()).cwiseSqrt()).array() / (x2y2.array() + 1).array();
  ret.row(0) = in.row(0).array() * frac.array();
  ret.row(1) = in.row(1).array() * frac.array();
  ret.row(2) = frac.array() - epsilon;
}

} // namespace csCamera
