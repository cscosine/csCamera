#include <csCamera/CameraDistortionModel_impl.hpp>
#include <csCamera/Camera_impl.hpp>
#include <csCamera/Utils_impl.hpp>

#include <sstream>

#define _USE_MATH_DEFINES
#include <math.h>

namespace csCamera {

template <class T>
Camera<T>::Camera()
    : m_fx(-1)
    , m_fy(-1)
    , m_cx(-1)
    , m_cy(-1)
    , m_w(-1)
    , m_h(-1)
    , m_K(Eigen::Matrix<T, 3, 3>::Zero())
    , m_invK(Eigen::Matrix<T, 3, 3>::Zero()) {
  init();
}

template <class T>
Camera<T>::Camera(T fx, T fy, T cx, T cy, int w, int h)
    : m_fx(fx)
    , m_fy(fy)
    , m_cx(cx)
    , m_cy(cy)
    , m_w(w)
    , m_h(h)
    , m_K(Eigen::Matrix<T, 3, 3>::Zero())
    , m_invK(Eigen::Matrix<T, 3, 3>::Zero()) {
  init();
}

template <class T>
Camera<T>::~Camera() {}

template <class T>
void Camera<T>::scale(double sx, double sy) {
  m_fx *= sx;
  m_fy *= sy;
  m_cx *= sx;
  m_cy *= sy;
  m_w *= sx;
  m_h *= sy;
  init();
}

template <class T>
void Camera<T>::init() {
  // K
  m_K(0, 0) = m_fx;
  m_K(1, 1) = m_fy;
  m_K(0, 2) = m_cx;
  m_K(1, 2) = m_cy;
  m_K(2, 2) = 1.0;

  // invK
  m_invK(0, 0) = 1 / m_fx;
  m_invK(1, 1) = 1 / m_fy;
  m_invK(0, 2) = -m_cx * m_invK(0, 0);
  m_invK(1, 2) = -m_cy * m_invK(1, 1);
  m_invK(2, 2) = 1.0;

  // angles
  m_angleHorizontal = 2 * std::atan(m_w / T(2.0) / m_fx);
  m_angleVertical = 2 * std::atan(m_h / T(2.0) / m_fy);
}

template <class T>
std::string Camera<T>::toString() const {
  std::string ret("--- Camera ---\n");

  std::ostringstream os;

  os << "  fx: " << m_fx << " px, fy: " << m_fy << " px" << std::endl;
  os << "  cx: " << m_cx << " px, cy: " << m_cy << " px" << std::endl;
  os << "  w: " << m_w << " px, h: " << m_h << " px" << std::endl;
  os << "  FOV horizontal: " << m_angleHorizontal * 180.0 / T(M_PI) << " deg, vertical: " << m_angleVertical * 180.0 / T(M_PI)
     << " deg " << std::endl;

  return ret + os.str();
}

template <class T>
void Camera<T>::points3D_to_image(const Eigen::Ref<const Eigen::Matrix<T, 3, Eigen::Dynamic>> in,
                                  Eigen::Ref<Eigen::Matrix<T, 2, Eigen::Dynamic>> dest) const {
  Eigen::Matrix<T, 2, Eigen::Dynamic> pz1 = Utils::points3D_to_z1(in);
  this->pointsZ1_to_image(pz1, dest);
}

template <class T>
void Camera<T>::points3D_to_image(const Eigen::Ref<const Eigen::Matrix<T, 3, Eigen::Dynamic>>& in,
                                  const CameraDistortionModel<T>& distModel,
                                  Eigen::Ref<Eigen::Matrix<T, 2, Eigen::Dynamic>> dest) const {
  Eigen::Matrix<T, 2, Eigen::Dynamic> pz1 = Utils::points3D_to_z1(in);
  Eigen::Matrix<T, 2, Eigen::Dynamic> pz1d = distModel.distort(pz1);
  this->pointsZ1_to_image(pz1d, dest);
}

//---------------------- MEI ----------------------

template <class T>
CameraMei<T>::CameraMei()
    : Camera<T>()
    , m_epsilon(-1) {}

template <class T>
CameraMei<T>::CameraMei(T fx, T fy, T cx, T cy, int w, int h, T epsilon)
    : Camera<T>(fx, fy, cx, cy, w, h)
    , m_epsilon(epsilon) {}

template <class T>
CameraMei<T>::~CameraMei() {}

template <class T>
std::string CameraMei<T>::toString() const {
  auto s = Camera<T>::toString();

  std::ostringstream os;

  os << "  epsilon: " << m_epsilon << std::endl;

  return s + os.str();
}

template <class T>
void CameraMei<T>::points3D_to_image(const Eigen::Ref<const Eigen::Matrix<T, 3, Eigen::Dynamic>> in,
                                     Eigen::Ref<Eigen::Matrix<T, 2, Eigen::Dynamic>> dest) const {
  // unit sphere
  Eigen::Matrix<T, 3, Eigen::Dynamic> pu = in.colwise().normalized();

  // point moved of epsilon, according to mei model, and projected on z1 plane
  Eigen::Matrix<T, 2, Eigen::Dynamic> pz1 = Utils::unitSpherePoint_to_z1_mei(pu, this->epsilon());

  // standard projection
  Camera<T>::pointsZ1_to_image(pz1, dest);
}

template <class T>
void CameraMei<T>::points3D_to_image(const Eigen::Ref<const Eigen::Matrix<T, 3, Eigen::Dynamic>>& in,
                                     const CameraDistortionModel<T>& distModel,
                                     Eigen::Ref<Eigen::Matrix<T, 2, Eigen::Dynamic>> dest) const {
  // unit sphere
  Eigen::Matrix<T, 3, Eigen::Dynamic> pu = in.colwise().normalized();

  // point moved of epsilon, according to mei model, and projected on z1 plane
  Eigen::Matrix<T, 2, Eigen::Dynamic> pz1 = Utils::unitSpherePoint_to_z1_mei(pu, this->epsilon());

  // distort
  Eigen::Matrix<T, 2, Eigen::Dynamic> pz1d = distModel.distort(pz1);

  // standard projection
  Camera<T>::pointsZ1_to_image(pz1d, dest);
}

// ------------------------------------------------

template class Camera<float>;
template class Camera<double>;

template class CameraMei<double>;
template class CameraMei<float>;
} // namespace csCamera
