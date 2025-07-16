#pragma once
#include <csCamera/CameraDistortionModel.hpp>

namespace csCamera {

template <class T>
template <class Derived1, class Derived2>
void CameraDistortionModel<T>::distort(const Eigen::MatrixBase<Derived1>& source, Eigen::MatrixBase<Derived2>& dest) const {
  EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(Eigen::Matrix2Xf, Derived1);
  EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(Eigen::Matrix2Xf, Derived2);
  assert(source.cols() == dest.cols());
  assert(source.rows() == dest.rows());

  constexpr int N = Derived1::ColsAtCompileTime;

  Eigen::Matrix<T, 2, N> x2y2 = source.cwiseAbs2();
  Eigen::Matrix<T, 1, N> r2 = x2y2.colwise().sum();
  Eigen::Matrix<T, 1, N> r4 = r2.array() * r2.array();
  Eigen::Matrix<T, 1, N> r6 = r4.array() * r2.array();
  Eigen::Matrix<T, 1, N> a1 = 2 * source.row(0).array() * source.row(1).array();
  Eigen::Matrix<T, 1, N> a2 = r2 + 2 * x2y2.row(0);
  Eigen::Matrix<T, 1, N> a3 = r2 + 2 * x2y2.row(1);

  Eigen::Matrix<T, 1, N> cdist = 1 + this->k1() * r2.array() + this->k2() * r4.array() + this->k3() * r6.array();
  Eigen::Matrix<T, 1, N> ddist = 1 + this->k4() * r2.array() + this->k5() * r4.array() + this->k6() * r6.array();

  dest.row(0) = source.row(0).array() * cdist.array() / ddist.array() + this->p1() * a1.array() + this->p2() * a2.array() +
                this->s1x() * r2.array() + this->s2x() * r4.array();

  dest.row(1) = source.row(1).array() * cdist.array() / ddist.array() + this->p1() * a3.array() + this->p2() * a1.array() +
                this->s3y() * r2.array() + this->s4y() * r4.array();
}

template <class T>
template <class Derived1, class Derived2, class Derived3, class Derived4, class Derived5, class Derived6>
Eigen::Matrix<T, 2, 1>
CameraDistortionModel<T>::distort_jacobians(const Eigen::MatrixBase<Derived1>& source, Eigen::MatrixBase<Derived2>& d_out_d_source,
                                            Eigen::MatrixBase<Derived3>& d_out_d_k1k2k3, Eigen::MatrixBase<Derived4>& d_out_d_k4k5k6,
                                            Eigen::MatrixBase<Derived5>& d_out_d_p1p2,
                                            Eigen::MatrixBase<Derived6>& d_out_d_s1xs2xs3ys4y) const {
  {
    EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(Eigen::Vector2f, Derived1);
    assert(source.rows() == 2 && source.cols() == 1);

    EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(Eigen::Matrix2f, Derived2);
    assert(d_out_d_source.rows() == 2 && d_out_d_source.cols() == 2);

    typedef Eigen::Matrix<float, 2, 3> M23;
    EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(M23, Derived3);
    assert(d_out_d_k1k2k3.rows() == 2 && d_out_d_k1k2k3.cols() == 3);

    EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(M23, Derived4);
    assert(d_out_d_k4k5k6.rows() == 2 && d_out_d_k4k5k6.cols() == 3);

    EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(Eigen::Matrix2f, Derived5);
    assert(d_out_d_p1p2.rows() == 2 && d_out_d_p1p2.cols() == 2);

    typedef Eigen::Matrix<float, 2, 4> M24;
    EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(M24, Derived6);
    assert(d_out_d_s1xs2xs3ys4y.rows() == 2 && d_out_d_s1xs2xs3ys4y.cols() == 4);
  }

  Eigen::Matrix<T, 2, 1> ret;
  T x = source.x();
  T y = source.y();
  T x2 = x * x;
  T y2 = y * y;

  T x3 = x2 * x;
  T y3 = y2 * y;

  T x5 = x2 * x3;
  T y5 = y2 * y3;

  T r2 = x2 + y2;
  Eigen::Matrix<T, 1, 2> d_r2_d_xy(2 * x, 2 * y);

  T r4 = r2 * r2;
  Eigen::Matrix<T, 1, 2> d_r4_d_xy(4 * x3, 4 * y3);

  T r6 = r4 * r2;
  Eigen::Matrix<T, 1, 2> d_r6_d_xy(6 * x5, 4 * y5);

  T a1 = 2 * x * y;
  Eigen::Matrix<T, 1, 2> d_a1_d_xy(2 * y, 2 * x);

  T a2 = r2 + 2 * x2;
  Eigen::Matrix<T, 1, 2> d_a2_d_xy = d_r2_d_xy + Eigen::Matrix<T, 1, 2>(4 * x, 0);

  T a3 = r2 + 2 * y2;
  Eigen::Matrix<T, 1, 2> d_a3_d_xy = d_r2_d_xy + Eigen::Matrix<T, 1, 2>(0, 4 * y);

  T num = 1 + this->k1() * r2 + this->k2() * r4 + this->k3() * r6;
  Eigen::Matrix<T, 1, 2> d_num_d_xy = this->k1() * d_r2_d_xy + this->k2() * d_r4_d_xy + this->k3() * d_r6_d_xy;
  Eigen::Matrix<T, 1, 3> d_num_d_k1k2k3 = Eigen::Matrix<T, 1, 3>(r2, r4, r6);

  T den = 1 + this->k4() * r2 + this->k5() * r4 + this->k6() * r6;
  Eigen::Matrix<T, 1, 2> d_den_d_xy = this->k4() * d_r2_d_xy + this->k5() * d_r4_d_xy + this->k6() * d_r6_d_xy;
  Eigen::Matrix<T, 1, 3> d_den_d_k3k4k5 = Eigen::Matrix<T, 1, 3>(r2, r4, r6);

  T r = num / den;
  T d_r_d_num = 1 / den;
  T d_r_d_den = -num / (den * den);

  //------------------------------------------------------------
  ret.x() = x * r + this->p1() * a1 + this->p2() * a2 + this->s1x() * r2 + this->s2x() * r4;

  d_out_d_source.row(0) = Eigen::Matrix<T, 1, 2>(r, 0) + x * (d_r_d_num * d_num_d_xy + d_r_d_den * d_den_d_xy) +
                          this->p1() * d_a1_d_xy + +this->p2() * d_a2_d_xy + +this->s1x() * d_r2_d_xy + +this->s2x() * d_r4_d_xy;

  d_out_d_k1k2k3.row(0) = x * (d_r_d_num * d_num_d_k1k2k3);
  d_out_d_k4k5k6.row(0) = x * (d_r_d_den * d_den_d_k3k4k5);
  d_out_d_p1p2.row(0) = Eigen::Matrix<T, 1, 2>(a1, a2);
  d_out_d_s1xs2xs3ys4y.row(0) = Eigen::Matrix<T, 1, 4>(r2, r4, 0, 0);

  //------------------------------------------------------------
  ret.y() = y * r + this->p1() * a3 + this->p2() * a1 + this->s3y() * r2 + this->s4y() * r4;

  d_out_d_source.row(1) = Eigen::Matrix<T, 1, 2>(0, r) + y * (d_r_d_num * d_num_d_xy + d_r_d_den * d_den_d_xy) +
                          this->p1() * d_a3_d_xy + +this->p2() * d_a1_d_xy + +this->s3y() * d_r2_d_xy + +this->s4y() * d_r4_d_xy;

  d_out_d_k1k2k3.row(1) = y * (d_r_d_num * d_num_d_k1k2k3);
  d_out_d_k4k5k6.row(1) = y * (d_r_d_den * d_den_d_k3k4k5);
  d_out_d_p1p2.row(1) = Eigen::Matrix<T, 1, 2>(a3, a1);
  d_out_d_s1xs2xs3ys4y.row(1) = Eigen::Matrix<T, 1, 4>(0, 0, r2, r4);

  return ret;
}

template <class T>
template <class Derived1, class Derived2>
Eigen::Matrix<T, 2, 1> CameraDistortionModel<T>::distort_jacobians(const Eigen::MatrixBase<Derived1>& source,
                                                                   Eigen::MatrixBase<Derived2>& d_out_d_source) const {

  {
    EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(Eigen::Vector2f, Derived1);
    assert(source.rows() == 2 && source.cols() == 1);

    EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(Eigen::Matrix2f, Derived2);
    assert(d_out_d_source.rows() == 2 && d_out_d_source.cols() == 2);
  }

  Eigen::Matrix<T, 2, 1> ret;
  T x = source.x();
  T y = source.y();
  T x2 = x * x;
  T y2 = y * y;

  T x3 = x2 * x;
  T y3 = y2 * y;

  T x5 = x2 * x3;
  T y5 = y2 * y3;

  T r2 = x2 + y2;
  Eigen::Matrix<T, 1, 2> d_r2_d_xy(2 * x, 2 * y);

  T r4 = r2 * r2;
  Eigen::Matrix<T, 1, 2> d_r4_d_xy(4 * x3, 4 * y3);

  T r6 = r4 * r2;
  Eigen::Matrix<T, 1, 2> d_r6_d_xy(6 * x5, 4 * y5);

  T a1 = 2 * x * y;
  Eigen::Matrix<T, 1, 2> d_a1_d_xy(2 * y, 2 * x);

  T a2 = r2 + 2 * x2;
  Eigen::Matrix<T, 1, 2> d_a2_d_xy = d_r2_d_xy + Eigen::Matrix<T, 1, 2>(4 * x, 0);

  T a3 = r2 + 2 * y2;
  Eigen::Matrix<T, 1, 2> d_a3_d_xy = d_r2_d_xy + Eigen::Matrix<T, 1, 2>(0, 4 * y);

  T num = 1 + this->k1() * r2 + this->k2() * r4 + this->k3() * r6;
  Eigen::Matrix<T, 1, 2> d_num_d_xy = this->k1() * d_r2_d_xy + this->k2() * d_r4_d_xy + this->k3() * d_r6_d_xy;

  T den = 1 + this->k4() * r2 + this->k5() * r4 + this->k6() * r6;
  Eigen::Matrix<T, 1, 2> d_den_d_xy = this->k4() * d_r2_d_xy + this->k5() * d_r4_d_xy + this->k6() * d_r6_d_xy;

  T r = num / den;
  T d_r_d_num = 1 / den;
  T d_r_d_den = -num / (den * den);

  //------------------------------------------------------------
  ret.x() = x * r + this->p1() * a1 + this->p2() * a2 + this->s1x() * r2 + this->s2x() * r4;

  d_out_d_source.row(0) = Eigen::Matrix<T, 1, 2>(r, 0) + x * (d_r_d_num * d_num_d_xy + d_r_d_den * d_den_d_xy) +
                          this->p1() * d_a1_d_xy + +this->p2() * d_a2_d_xy + +this->s1x() * d_r2_d_xy + +this->s2x() * d_r4_d_xy;

  //------------------------------------------------------------
  ret.y() = y * r + this->p1() * a3 + this->p2() * a1 + this->s3y() * r2 + this->s4y() * r4;

  d_out_d_source.row(1) = Eigen::Matrix<T, 1, 2>(0, r) + y * (d_r_d_num * d_num_d_xy + d_r_d_den * d_den_d_xy) +
                          this->p1() * d_a3_d_xy + +this->p2() * d_a1_d_xy + +this->s3y() * d_r2_d_xy + +this->s4y() * d_r4_d_xy;

  return ret;
}

template <class T>
template <class Derived1, class Derived2>
void CameraDistortionModel<T>::undistort(const Eigen::MatrixBase<Derived1>& source, Eigen::MatrixBase<Derived2>& ret, int numit,
                                         T tol) const {
  // so far do it one by one... how to check convergence on the group?
  for (int i = 0; i < source.cols(); i++) {
    ret.col(i) = undistortSinglePoint(source.col(i), numit, tol);
  }
}

template <class T>
template <class Derived1>
Eigen::Matrix<T, 2, 1> CameraDistortionModel<T>::undistortSinglePoint(const Eigen::MatrixBase<Derived1>& source, int numit,
                                                                      T tol) const {
  const T tolSq = tol * tol;

  Eigen::Matrix<T, 2, 1> p;
  p = source;

  Eigen::Matrix<T, 2, 1> diff(-1, -1); // put here to allow end for checks

  int i = 0; // just to know afterwards num it done
  for (; i < numit; i++) {
    T x2 = p.x() * p.x();
    T y2 = p.y() * p.y();
    T xy = p.x() * p.y();

    T r2 = x2 + y2;
    T r4 = r2 * r2;
    T r6 = r4 * r2;

    T a1 = 2 * xy;
    T a2 = r2 + 2 * x2;
    T a3 = r2 + 2 * y2;

    T cdist = 1 + this->k1() * r2 + this->k2() * r4 + this->k3() * r6;
    T ddist = 1 + this->k4() * r2 + this->k5() * r4 + this->k6() * r6;

    T icdist = ddist / cdist;
    if (icdist < 0) {
      // opencv do this...
      // test: undistortPoints.regression_14583
      // openvc/modules/calib3d/src/undistort.dispatch.cpp in function cvUndistortPointsInternal
      assert(false && "check this beahvior");
      break;
    }

    Eigen::Matrix<T, 2, 1> dxy(this->p1() * a1 + this->p2() * a2 + this->s1x() * r2 + this->s2x() * r4,
                               this->p1() * a3 + this->p2() * a1 + this->s3y() * r2 + this->s4y() * r4);

    Eigen::Matrix<T, 2, 1> pnew = (source - dxy) * icdist;
    diff = pnew - p;

    p = pnew;

    if (diff.squaredNorm() < tolSq) {
      break;
    }
  }

  if (diff.squaredNorm() >= tolSq) {
    do {
    } while (0); // no op, just to put a breakpoint here in debug :-p
  }

  return p;
}

} // namespace csCamera
