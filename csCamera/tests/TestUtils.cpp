#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>

#include "csCamera/Utils.hpp"

TEMPLATE_TEST_CASE("Utils", "[Utils::points3D_to_z1]", float, double) {
  using T = TestType;

  T epsilon = 1;
  {
    Eigen::Matrix<T, 3, Eigen::Dynamic> source(3, 100);
    source.setRandom();
    {
      auto ret = csCamera::Utils::points3D_to_z1(source);
      REQUIRE(ret.cols() == source.cols());
      REQUIRE(
          ((ret.row(0).array() - source.row(0).array() / source.row(2).array()).array() < Eigen::NumTraits<float>::dummy_precision())
              .all());
    }
    {
      auto ret = csCamera::Utils::unitSpherePoint_to_z1_mei(source, epsilon);
      REQUIRE(ret.cols() == source.cols());
      REQUIRE(((ret.row(0).array() - source.row(0).array() / (source.row(2).array() + epsilon)).array() <
               Eigen::NumTraits<float>::dummy_precision())
                  .all());
    }

    {
      Eigen::Matrix<T, 2, Eigen::Dynamic> ret2(2, source.cols());
      csCamera::Utils::points3D_to_z1(source, ret2);
    }
    {
      Eigen::Matrix<T, 2, Eigen::Dynamic> ret2(2, source.cols());
      csCamera::Utils::unitSpherePoint_to_z1_mei(source, epsilon, ret2);
      REQUIRE(ret2.cols() == source.cols());
    }

    {
      Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> ret3(2, source.cols());
      csCamera::Utils::points3D_to_z1(source, ret3);
      REQUIRE(ret3.cols() == source.cols());
    }
    {
      Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> ret3(2, source.cols());
      csCamera::Utils::unitSpherePoint_to_z1_mei(source, epsilon, ret3);
      REQUIRE(ret3.cols() == source.cols());
    }
  }

  {
    Eigen::Matrix<T, 3, 10> source;
    source.setRandom();
    source.colwise().normalize();
    {
      auto ret = csCamera::Utils::points3D_to_z1(source);
      REQUIRE(ret.cols() == source.cols());
      REQUIRE(
          ((ret.row(0).array() - source.row(0).array() / source.row(2).array()).array() < Eigen::NumTraits<float>::dummy_precision())
              .all());
    }
    {
      auto ret = csCamera::Utils::unitSpherePoint_to_z1_mei(source, epsilon);
      auto ret2 = csCamera::Utils::pointsZ1_to_unitSpherePoint_mei(ret, epsilon);
      REQUIRE(ret.cols() == source.cols());
      REQUIRE(((ret.row(0).array() - source.row(0).array() / (source.row(2).array() + epsilon)).array() <
               Eigen::NumTraits<float>::dummy_precision())
                  .all());
    }

    Eigen::Matrix<T, 2, 10> ret2(2, source.cols());
    {
      csCamera::Utils::points3D_to_z1(source, ret2);
      REQUIRE(ret2.cols() == source.cols());
    }
    {
      csCamera::Utils::unitSpherePoint_to_z1_mei(source, epsilon, ret2);
      Eigen::Matrix<T, 3, 10> retus;
      csCamera::Utils::pointsZ1_to_unitSpherePoint_mei(ret2, epsilon, retus);
      REQUIRE(ret2.cols() == source.cols());
    }

    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> ret3(2, source.cols());
    {
      csCamera::Utils::points3D_to_z1(source, ret3);
      REQUIRE(ret3.cols() == source.cols());
    }
    {
      csCamera::Utils::unitSpherePoint_to_z1_mei(source, epsilon, ret3);
      REQUIRE(ret3.cols() == source.cols());
    }
  }

  {
    Eigen::Matrix<T, 3, 1> source;
    source.setRandom();
    {
      Eigen::Matrix<T, 2, 3> d_out_d_in;
      auto ret = csCamera::Utils::points3D_to_z1_jacob(source, d_out_d_in);
    }
    // test mei
    {
      bool ok;
      auto ret = csCamera::Utils::unitSpherePoint_to_z1_mei(source, epsilon, ok);
    }
    {
      Eigen::Matrix<T, 2, 3> d_out_d_in;
      Eigen::Matrix<T, 2, 1> d_out_d_epsilon;
      auto ret = csCamera::Utils::unitSpherePoint_to_z1_mei_jacob(source, epsilon, d_out_d_in, d_out_d_epsilon);
    }
    {
      Eigen::Matrix<T, 2, 3> d_out_d_in;
      auto ret = csCamera::Utils::unitSpherePoint_to_z1_mei_jacob(source, epsilon, d_out_d_in);
    }
  }

  // multiple jacobians
  {
    Eigen::Matrix<T, 3, Eigen::Dynamic> source(3, 100);
    source.setRandom();
    Eigen::Matrix<T, 2, Eigen::Dynamic> d_out_d_in(2, 3 * source.cols());
    for (int i = 0; i < source.cols(); i++) {
      auto block = d_out_d_in.middleCols(i * 3, 3);
      auto ret = csCamera::Utils::points3D_to_z1_jacob(source.col(i), block);
      REQUIRE(d_out_d_in(0, 3 * i) == 1 / source.col(i).z());
    }
  }
}
