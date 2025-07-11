#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>

#include "csCamera/CameraDistortionModel.hpp"

TEST_CASE("CameraDistortionModel", "[CameraDistortionModel]")
{

  csCamera::CameraDistortionModeld cdm(
    -0.323809, 0.006031, 0.099565,
    0.000000, 0.000000, 0.000000,
    -0.018420, 0.034603,
    -0.133942, 0.004625, 0.050119, -0.011123
  );

  {
    auto dist = cdm.distort(Eigen::Matrix2Xd::Random(2, 10));
    cdm.distort(Eigen::Matrix2Xd::Random(2, 10), dist);
    cdm.distort(Eigen::Matrix<double, 2, 10>::Random(2, 10), dist);

    Eigen::Matrix2d d_out_d_source_22, d_out_d_p1p2_22;
    Eigen::Matrix<double, 2, 3> d_out_d_k1k2k3_23, d_out_d_k4k5k6_23;
    Eigen::Matrix<double, 2, 4> d_out_d_s1xs2xs3ys4y_24;
    auto dist2 = cdm.distort_jacobians(Eigen::Vector2d::Random(), d_out_d_source_22, d_out_d_k1k2k3_23, d_out_d_k4k5k6_23, d_out_d_p1p2_22, d_out_d_s1xs2xs3ys4y_24);
    auto dist3 = cdm.distort_jacobians(Eigen::Vector2d::Random(), d_out_d_source_22);

    auto undist = cdm.undistort(dist);

    auto undist1 = cdm.undistortSinglePoint(Eigen::Vector2d::Random());
  }
  {
    auto dist = cdm.distort(Eigen::Matrix<double, 2, 10>::Random(2, 10));
    cdm.distort(Eigen::Matrix2Xd::Random(2, 10), dist);
    cdm.distort(Eigen::Matrix<double, 2, 10>::Random(2, 10), dist);
  }
  {
    auto dist = cdm.distort(Eigen::Matrix<double, 2, 10>::Random(2, 10));
    cdm.distort(Eigen::Matrix2Xd::Random(2, 10), dist);
    cdm.distort(Eigen::Matrix<double, 2, 10>::Random(2, 10), dist);
  }

  for (int i = 0; i < 1000; i++) {
    Eigen::Vector2d source = Eigen::Vector2d::Random() * 0.01;
    Eigen::Vector2d dist = cdm.distort(source);
    Eigen::Vector2d undist = cdm.undistort(dist,100);
    REQUIRE((source - undist).squaredNorm() <= 1e-6);
  }

}
