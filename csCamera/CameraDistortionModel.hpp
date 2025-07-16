#pragma once

#include <Eigen/Core>
#include <string>

namespace csCamera {

template <class T>
class CameraDistortionModel {
  T _k1, _k2, _k3;
  T _k4, _k5, _k6;
  T _p1, _p2;
  T _s1x, _s2x, _s3y, _s4y; // s1, s2, s3, s4 in opencv

public:
  CameraDistortionModel(T k1, T k2, T k3, T p1, T p2, T k4 = 0, T k5 = 0, T k6 = 0, T s1x = 0, T s2x = 0, T s3y = 0, T s4y = 0);
  CameraDistortionModel();
  virtual ~CameraDistortionModel();

  const T k1() const {
    return _k1;
  }
  const T k2() const {
    return _k2;
  }
  const T k3() const {
    return _k3;
  }

  const T k4() const {
    return _k4;
  }
  const T k5() const {
    return _k5;
  }
  const T k6() const {
    return _k6;
  }

  const T p1() const {
    return _p1;
  }
  const T p2() const {
    return _p2;
  }

  const T s1x() const {
    return _s1x;
  }
  const T s2x() const {
    return _s2x;
  }
  const T s3y() const {
    return _s3y;
  }
  const T s4y() const {
    return _s4y;
  }

  std::string toString() const;

  // distort
  template <class Derived>
  Eigen::Matrix<T, 2, Derived::ColsAtCompileTime> distort(const Eigen::MatrixBase<Derived>& source) const {
    Eigen::Matrix<T, 2, Derived::ColsAtCompileTime> ret(2, source.cols());
    distort(source, ret);
    return ret;
  }

  template <class Derived1, class Derived2>
  void distort(const Eigen::MatrixBase<Derived1>& source, Eigen::MatrixBase<Derived2>& dest) const;

  // distort and compute jacobians
  template <class Derived1, class Derived2, class Derived3, class Derived4, class Derived5, class Derived6>
  Eigen::Matrix<T, 2, 1>
  distort_jacobians(const Eigen::MatrixBase<Derived1>& source, Eigen::MatrixBase<Derived2>& d_out_d_source_22,
                    Eigen::MatrixBase<Derived3>& d_out_d_k1k2k3_23, Eigen::MatrixBase<Derived4>& d_out_d_k4k5k6_23,
                    Eigen::MatrixBase<Derived5>& d_out_d_p1p2_22, Eigen::MatrixBase<Derived6>& d_out_d_s1xs2xs3ys4y_24) const;

  // distort and compute jacobians (only source)
  template <class Derived1, class Derived2>
  Eigen::Matrix<T, 2, 1> distort_jacobians(const Eigen::MatrixBase<Derived1>& source,
                                           Eigen::MatrixBase<Derived2>& d_out_d_source_22) const;

  // undistort
  template <class Derived1>
  Eigen::Matrix<typename Derived1::Scalar, 2, Derived1::ColsAtCompileTime> undistort(const Eigen::MatrixBase<Derived1>& source,
                                                                                     int numit = 20, T tol = T(1e-6)) const {
    Eigen::Matrix<typename Derived1::Scalar, 2, Derived1::ColsAtCompileTime> ret(2, source.cols());
    undistort(source, ret, numit, tol);
    return ret;
  }

  template <class Derived1, class Derived2>
  void undistort(const Eigen::MatrixBase<Derived1>& source, Eigen::MatrixBase<Derived2>& dest, int numit = 20, T tol = T(1e-6)) const;

  // undistort single point
  template <class Derived1>
  Eigen::Matrix<T, 2, 1> undistortSinglePoint(const Eigen::MatrixBase<Derived1>& source, int numit = 20, T tol = T(1e-6)) const;
};

using CameraDistortionModelf = CameraDistortionModel<float>;
using CameraDistortionModeld = CameraDistortionModel<double>;

} // namespace csCamera
