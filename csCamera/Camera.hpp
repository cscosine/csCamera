#pragma once
#include "Camera.h"
#include "Utils.hpp"

namespace csCamera {

template <class T>
template <class Derived1, class Derived2>
void Camera<T>::pointsZ1_to_image(const Eigen::MatrixBase<Derived1>& source, Eigen::MatrixBase<Derived2>& ret) const {
  {
    EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(Eigen::Matrix2Xf, Derived1);
    assert(source.rows() == 2);

    EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(Eigen::Matrix2Xf, Derived2);
    assert(ret.rows() == 2);
  }

  ret = Eigen::Matrix<T, 2, 1>(this->fx(), this->fy()).asDiagonal() * source;
  ret.colwise() += Eigen::Matrix<T, 2, 1>(this->cx(), this->cy());
}

template <class T>
template <class Derived1, class Derived2>
void Camera<T>::pointsImage_to_Z1(const Eigen::MatrixBase<Derived1>& source, Eigen::MatrixBase<Derived2>& ret) const {
  {
    EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(Eigen::Matrix2Xf, Derived1);
    assert(source.rows() == 2);

    EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(Eigen::Matrix2Xf, Derived2);
    assert(ret.rows() == 2);
  }

  ret.row(0) = (source.row(0).array() - this->cx()) / this->fx();
  ret.row(1) = (source.row(1).array() - this->cy()) / this->fy();
}

// z=1 plane to camera image for single point with jacobian
template <class T>
template <class Derived1, class Derived2, class Derived3>
Eigen::Matrix<T, 2, 1> Camera<T>::pointsZ1_to_image_jacobian(const Eigen::MatrixBase<Derived1>& source,
                                                             Eigen::MatrixBase<Derived2>& d_out_d_cam,
                                                             Eigen::MatrixBase<Derived3>& d_out_d_in) const {
  {
    EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(Eigen::Vector2f, Derived1);
    assert(source.rows() == 2 && source.cols() == 1);

    typedef Eigen::Matrix<float, 2, 4> M24;
    EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(M24, Derived2);
    assert(d_out_d_cam.rows() == 2 && d_out_d_cam.cols() == 4);

    EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(Eigen::Matrix2f, Derived3);
    assert(d_out_d_in.rows() == 2 && d_out_d_in.cols() == 2);
  }

  Eigen::Matrix<T, 2, 1> ret(source.x() * this->fx() + this->cx(), source.y() * this->fy() + this->cy());

  d_out_d_cam(0, 0) = source.x();
  d_out_d_cam(0, 1) = 0;
  d_out_d_cam(0, 2) = 1;
  d_out_d_cam(0, 3) = 0;

  d_out_d_cam(1, 0) = 0;
  d_out_d_cam(1, 1) = source.y();
  d_out_d_cam(1, 2) = 0;
  d_out_d_cam(1, 3) = 1;

  d_out_d_in = Eigen::Matrix<T, 2, 1>(this->fx(), this->fy()).asDiagonal();

  return ret;
}

// z=1 plane to camera image for single point with jacobian (only source point)
template <class T>
template <class Derived1, class Derived2>
Eigen::Matrix<T, 2, 1> Camera<T>::pointsZ1_to_image_jacobian(const Eigen::MatrixBase<Derived1>& source,
                                                             Eigen::MatrixBase<Derived2>& d_out_d_in) const {
  {
    EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(Eigen::Vector2f, Derived1);
    assert(source.rows() == 2 && source.cols() == 1);

    EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(Eigen::Matrix2f, Derived2);
    assert(d_out_d_in.rows() == 2 && d_out_d_in.cols() == 2);
  }

  Eigen::Vector2d ret(source.x() * this->fx() + this->cx(), source.y() * this->fy() + this->cy());

  d_out_d_in = Eigen::Matrix<T, 2, 1>(this->fx(), this->fy()).asDiagonal();

  return ret;
}

} // namespace csCamera
