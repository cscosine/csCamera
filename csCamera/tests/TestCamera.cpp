#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>

#include <csCamera/CameraDistortionModel_impl.hpp>
#include <csCamera/Camera_impl.hpp>

TEST_CASE("Camera", "[Camera]") {

  csCamera::Camerad cam(100, 100, 50, 50, 100, 100);
  csCamera::CameraMeid camMei(100, 100, 50, 50, 100, 100, 1.0);

  auto im = cam.pointsZ1_to_image(Eigen::Matrix2Xd::Random(2, 100));
  cam.pointsZ1_to_image(Eigen::Matrix2Xd::Random(2, 100), im);

  Eigen::Matrix<double, 2, 4> d_out_d_cam;
  Eigen::Matrix<double, 2, 2> d_out_d_in;

  auto i1 = cam.pointsZ1_to_image_jacobian(Eigen::Vector2d::Random(), d_out_d_in);
  i1 = cam.pointsZ1_to_image_jacobian(Eigen::Vector2d::Random(), d_out_d_cam, d_out_d_in);

  auto pz1 = cam.pointsImage_to_Z1(Eigen::Matrix2Xd::Random(2, 100));

  csCamera::CameraDistortionModeld cdm(-0.323809, 0.006031, 0.099565, 0.000000, 0.000000, 0.000000, -0.018420, 0.034603, -0.133942,
                                       0.004625, 0.050119, -0.011123);

  auto pi = cam.points3D_to_image(Eigen::Matrix3Xd::Random(3, 100));
  pi = cam.points3D_to_image(Eigen::Matrix3Xd::Random(3, 100), cdm);

  csCamera::Camerad& camRef = camMei;
  pi = camRef.points3D_to_image(Eigen::Matrix3Xd::Random(3, 100));
  pi = camRef.points3D_to_image(Eigen::Matrix3Xd::Random(3, 100), cdm);
}
