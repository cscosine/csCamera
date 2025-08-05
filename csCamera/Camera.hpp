#pragma once
#include <Eigen/Core>

#include <memory>
#include <string>

namespace csCamera {

template <class T>
class CameraDistortionModel;

template <class T>
class Camera {
public:
  Camera();
  Camera(T fx, T fy, T cx, T cy, int w, int h);
  virtual ~Camera();

  void scale(double sx, double sy);

  T fx() const {
    return m_fx;
  }

  void setFx(T v) {
    m_fx = v;
    init();
  }

  T fy() const {
    return m_fy;
  }

  void setFy(T v) {
    m_fy = v;
    init();
  }

  void setFxFy(T fx, T fy) {
    m_fx = fx;
    m_fy = fy;
    init();
  }

  T cx() const {
    return m_cx;
  }

  void setCx(T v) {
    m_cx = v;
    init();
  }

  T cy() const {
    return m_cy;
  }

  void setCy(T v) {
    m_cy = v;
    init();
  }

  void setCxCy(T cx, T cy) {
    m_cx = cx;
    m_cy = cy;
    init();
  }

  void setImageSize(T w, T h) {
    m_w = w;
    m_h = h;
  }

  Eigen::Matrix<T, 2, 1> center() const {
    return Eigen::Matrix<T, 2, 1>(m_cx, m_cy);
  }

  int w() const {
    return m_w;
  }
  int h() const {
    return m_h;
  }

  virtual T epsilon() const {
    return 1.0;
  }

  Eigen::Vector2i size() const {
    return Eigen::Vector2i(m_w, m_h);
  }

  const Eigen::Matrix<T, 3, 3>& K() const {
    return m_K;
  }

  const Eigen::Matrix<T, 3, 3>& invK() const {
    return m_invK;
  }

  T angleHorizontal() const {
    return m_angleHorizontal;
  }

  T angleVertical() const {
    return m_angleVertical;
  }

  virtual bool isMeiGeneralModel() const {
    return false;
  }

  virtual bool requireMeiGeneralModel() const {
    return false;
  }

  virtual std::string toString() const;

  // z=1 plane to camera image
  template <class Derived>
  Eigen::Matrix<T, 2, Derived::ColsAtCompileTime> pointsZ1_to_image(const Eigen::MatrixBase<Derived>& source) const {
    Eigen::Matrix<T, 2, Derived::ColsAtCompileTime> ret(2, source.cols());
    pointsZ1_to_image(source, ret);
    return ret;
  }

  template <class Derived1, class Derived2>
  void pointsZ1_to_image(const Eigen::MatrixBase<Derived1>& source, Eigen::MatrixBase<Derived2>& dest) const;

  // z=1 plane to camera image for single point with jacobian
  template <class Derived1, class Derived2, class Derived3>
  Eigen::Matrix<T, 2, 1> pointsZ1_to_image_jacobian(const Eigen::MatrixBase<Derived1>& source,
                                                    Eigen::MatrixBase<Derived2>& d_out_d_cam_24,
                                                    Eigen::MatrixBase<Derived3>& d_out_d_in_22) const;

  template <class Derived1, class Derived2>
  Eigen::Matrix<T, 2, 1> pointsZ1_to_image_jacobian(const Eigen::MatrixBase<Derived1>& source,
                                                    Eigen::MatrixBase<Derived2>& d_out_d_in_22) const;

  // image points to 3D z=1 points
  template <class Derived>
  Eigen::Matrix<T, 2, Derived::ColsAtCompileTime> pointsImage_to_Z1(const Eigen::MatrixBase<Derived>& source) const {
    Eigen::Matrix<T, 2, Derived::ColsAtCompileTime> ret(2, source.cols());
    pointsImage_to_Z1(source, ret);
    return ret;
  }

  template <class Derived1, class Derived2>
  void pointsImage_to_Z1(const Eigen::MatrixBase<Derived1>& source, Eigen::MatrixBase<Derived2>& dest) const;

  template <class Derived>
  Eigen::Matrix<T, 2, Derived::ColsAtCompileTime> points3D_to_image(const Eigen::MatrixBase<Derived>& source) const {
    Eigen::Matrix<T, 2, Derived::ColsAtCompileTime> ret(2, source.cols());
    points3D_to_image(source, ret);
    return ret;
  }

  virtual void points3D_to_image(const Eigen::Ref<const Eigen::Matrix<T, 3, Eigen::Dynamic>> in,
                                 Eigen::Ref<Eigen::Matrix<T, 2, Eigen::Dynamic>> dest) const;

  Eigen::Matrix<T, 2, Eigen::Dynamic> points3D_to_image(const Eigen::Ref<const Eigen::Matrix<T, 3, Eigen::Dynamic>>& source,
                                                        const CameraDistortionModel<T>& distModel) const {
    Eigen::Matrix<T, 2, Eigen::Dynamic> ret(2, source.cols());
    points3D_to_image(source, distModel, ret);
    return ret;
  }

  virtual void points3D_to_image(const Eigen::Ref<const Eigen::Matrix<T, 3, Eigen::Dynamic>>& source,
                                 const CameraDistortionModel<T>& distModel,
                                 Eigen::Ref<Eigen::Matrix<T, 2, Eigen::Dynamic>> dest) const;

private:
  T m_fx, m_fy, m_cx, m_cy;
  Eigen::Matrix<T, 3, 3> m_K;
  Eigen::Matrix<T, 3, 3> m_invK;
  int m_w, m_h;

  T m_angleHorizontal;
  T m_angleVertical;

  void init();
};

template <class T>
class CameraMei : public Camera<T> {

public:
  CameraMei();
  CameraMei(T fx, T fy, T cx, T cy, int w, int h, T epsilon);
  virtual ~CameraMei();

  T epsilon() const override {
    return m_epsilon;
  }

  void setEpsilon(T e) {
    m_epsilon = e;
  }

  bool isMeiGeneralModel() const override {
    return true;
  }

  virtual bool requireMeiGeneralModel() const override {
    return m_epsilon != 0;
  }

  virtual std::string toString() const override;

  void points3D_to_image(const Eigen::Ref<const Eigen::Matrix<T, 3, Eigen::Dynamic>> in,
                         Eigen::Ref<Eigen::Matrix<T, 2, Eigen::Dynamic>> dest) const override;
  void points3D_to_image(const Eigen::Ref<const Eigen::Matrix<T, 3, Eigen::Dynamic>>& source,
                         const CameraDistortionModel<T>& distModel,
                         Eigen::Ref<Eigen::Matrix<T, 2, Eigen::Dynamic>> dest) const override;

private:
  T m_epsilon;
};

using Cameraf = Camera<float>;
using Camerad = Camera<double>;

using CameraMeif = CameraMei<float>;
using CameraMeid = CameraMei<double>;

} // namespace csCamera
