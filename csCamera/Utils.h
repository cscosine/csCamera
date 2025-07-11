#pragma once
#include <Eigen/Core>

namespace csCamera {

  class Utils {

  public:

    // 3D cartesian point to point on z=1 plane, output is 2d, z=1 is omitted
    template<class Derived>
    static Eigen::Matrix<typename Derived::Scalar, 2, Derived::ColsAtCompileTime> points3D_to_z1(const Eigen::MatrixBase<Derived>& source) {
      Eigen::Matrix<typename Derived::Scalar, 2, Derived::ColsAtCompileTime> ret(2, source.cols());
      points3D_to_z1(source, ret);
      return ret;
    }

    // 3D cartesian point to point on z=1 plane, output is 2d, z=1 is omitted
    template<class Derived1, class Derived2>
    static void points3D_to_z1(const Eigen::MatrixBase<Derived1>& source, Eigen::MatrixBase<Derived2>& dest);

    // 3d point to z=1 plane for single point with jacobian
    template<class Derived1, class Derived2>
    static Eigen::Matrix<typename Derived1::Scalar, 2, 1> points3D_to_z1_jacob(const Eigen::MatrixBase<Derived1>& in, Eigen::MatrixBase<Derived2>& d_out_d_in);

    // mei h function
    template<class Derived>
    static Eigen::Matrix<typename Derived::Scalar, 2, Derived::ColsAtCompileTime> unitSpherePoint_to_z1_mei(const Eigen::MatrixBase<Derived>& source, typename Derived::Scalar epsilon) {
      Eigen::Matrix<typename Derived::Scalar, 2, Derived::ColsAtCompileTime> ret(2, source.cols());
      unitSpherePoint_to_z1_mei(source, epsilon, ret);
      return ret;
    }

    template<class Derived1, class Derived2>
    static void unitSpherePoint_to_z1_mei(const Eigen::MatrixBase<Derived1>& source, typename Derived1::Scalar epsilon, Eigen::MatrixBase<Derived2>& dest);

    // mei h function for single point with z check
    template<class Derived>
    static Eigen::Matrix<typename Derived::Scalar, 2, 1> unitSpherePoint_to_z1_mei(const Eigen::MatrixBase<Derived>& in, typename Derived::Scalar epsilon, bool& zOk);

    // mei h function for single point with jacobian
    template<class Derived1, class Derived2, class Derived3>
    static Eigen::Matrix<typename Derived1::Scalar, 2, 1> unitSpherePoint_to_z1_mei_jacob(const Eigen::MatrixBase<Derived1>& in, typename Derived1::Scalar epsilon, Eigen::MatrixBase<Derived2> & d_out_d_in, Eigen::MatrixBase<Derived3>& d_out_d_epsilon);

    // mei h function for single point with jacobian (only source point)
    template<class Derived1, class Derived2>
    static Eigen::Matrix<typename Derived1::Scalar, 2, 1>  unitSpherePoint_to_z1_mei_jacob(const Eigen::MatrixBase<Derived1>& in, typename Derived1::Scalar epsilon, Eigen::MatrixBase<Derived2>& d_out_d_in);


    // image points to 3D z=1 points - mei
    template<class Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, Derived::ColsAtCompileTime> pointsZ1_to_unitSpherePoint_mei(const Eigen::MatrixBase<Derived>& in, typename Derived::Scalar epsilon) {
      Eigen::Matrix<typename Derived::Scalar, 3, Derived::ColsAtCompileTime> ret(3, in.cols());
      pointsZ1_to_unitSpherePoint_mei(in, epsilon, ret);
      return ret;
    }

    template<class Derived1, class Derived2>
    static void pointsZ1_to_unitSpherePoint_mei(const Eigen::MatrixBase<Derived1>& source, typename Derived1::Scalar epsilon, Eigen::MatrixBase<Derived2>& dest);

  };

}