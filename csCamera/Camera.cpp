#include "Camera.h"
#include "Camera.hpp"
#include "CameraDistortionModel.hpp"

#include <sstream>

#define _USE_MATH_DEFINES
#include <math.h>


namespace csCamera {

  template<class T>
  Camera<T>::Camera()
    :_fx(-1), _fy(-1), _cx(-1), _cy(-1), _w(-1), _h(-1),
    _K(Eigen::Matrix<T, 3, 3>::Zero()),
    _invK(Eigen::Matrix<T, 3, 3>::Zero())
  {
    init();
  }

  template<class T>
  Camera<T>::Camera(T fx, T fy, T cx, T cy, int w, int h)
    :_fx(fx), _fy(fy), _cx(cx), _cy(cy), _w(w), _h(h),
    _K(Eigen::Matrix<T, 3, 3>::Zero()),
    _invK(Eigen::Matrix<T, 3, 3>::Zero())
  {
    init();
  }

  template<class T>
  Camera<T>::~Camera() {

  }

  template<class T>
  void Camera<T>::scale(double sx, double sy) {
    _fx *= sx;
    _fy *= sy;
    _cx *= sx;
    _cy *= sy;
    _w *= sx;
    _h *= sy;
    init();
  }

  template<class T>
  void Camera<T>::init() {
    //K
    _K(0, 0) = _fx;
    _K(1, 1) = _fy;
    _K(0, 2) = _cx;
    _K(1, 2) = _cy;
    _K(2, 2) = 1.0;

    //invK
    _invK(0, 0) = 1 / _fx;
    _invK(1, 1) = 1 / _fy;
    _invK(0, 2) = -_cx * _invK(0, 0);
    _invK(1, 2) = -_cy * _invK(1, 1);
    _invK(2, 2) = 1.0;

    // angles
    _angleHorizontal = 2 * std::atan(_w / T(2.0) / _fx);
    _angleVertical = 2 * std::atan(_h / T(2.0) / _fy);
  }

  template<class T>
  std::string Camera<T>::toString() const {
    std::string ret("--- Camera ---\n");

    std::ostringstream os;

    os << "  fx: " << _fx << " px, fy: " << _fy << " px" << std::endl;
    os << "  cx: " << _cx << " px, cy: " << _cy << " px" << std::endl;
    os << "  w: " << _w << " px, h: " << _h << " px" << std::endl;
    os << "  FOV horizontal: " << _angleHorizontal * 180.0 / T(M_PI) << " deg, vertical: " << _angleVertical * 180.0 / T(M_PI) << " deg " << std::endl;

    return ret + os.str();
  }

  template<class T>
  void Camera<T>::points3D_to_image(const Eigen::Ref<const Eigen::Matrix<T, 3, Eigen::Dynamic>> in, Eigen::Ref<Eigen::Matrix<T, 2, Eigen::Dynamic>> dest) const {
    Eigen::Matrix<T,2, Eigen::Dynamic> pz1 = Utils::points3D_to_z1(in);
    this->pointsZ1_to_image(pz1, dest);
  }

  template<class T>
  void Camera<T>::points3D_to_image(const Eigen::Ref<const Eigen::Matrix<T, 3, Eigen::Dynamic>>& in, const CameraDistortionModel<T>& distModel, Eigen::Ref<Eigen::Matrix<T, 2, Eigen::Dynamic>> dest) const {
    Eigen::Matrix<T, 2, Eigen::Dynamic> pz1 = Utils::points3D_to_z1(in);
    Eigen::Matrix<T, 2, Eigen::Dynamic> pz1d = distModel.distort(pz1);
    this->pointsZ1_to_image(pz1d, dest);

  }

  //---------------------- MEI ----------------------

  template<class T>
  CameraMei<T>::CameraMei()
    : Camera<T>(),
    _epsilon(-1)
  {

  }

  template<class T>
  CameraMei<T>::CameraMei(T fx, T fy, T cx, T cy, int w, int h, T epsilon)
    : Camera<T>(fx, fy, cx, cy, w, h),
    _epsilon(epsilon)
  {

  }

  template<class T>
  CameraMei<T>::~CameraMei() {

  }

  template<class T>
  std::string CameraMei<T>::toString() const {
    auto s = Camera<T>::toString();

    std::ostringstream os;

    os << "  epsilon: " << _epsilon << std::endl;

    return s + os.str();
  }

  template<class T>
  void CameraMei<T>::points3D_to_image(const Eigen::Ref<const Eigen::Matrix<T, 3, Eigen::Dynamic>> in, Eigen::Ref<Eigen::Matrix<T, 2, Eigen::Dynamic>> dest) const {
    // unit sphere
    Eigen::Matrix<T, 3, Eigen::Dynamic> pu = in.colwise().normalized();

    // point moved of epsilon, according to mei model, and projected on z1 plane
    Eigen::Matrix<T, 2, Eigen::Dynamic> pz1 = Utils::unitSpherePoint_to_z1_mei(pu, this->epsilon());


    // standard projection
    Camera<T>::pointsZ1_to_image(pz1, dest);
  }

  template<class T>
  void CameraMei<T>::points3D_to_image(const Eigen::Ref<const Eigen::Matrix<T, 3, Eigen::Dynamic>>& in, const CameraDistortionModel<T>& distModel, Eigen::Ref<Eigen::Matrix<T, 2, Eigen::Dynamic>> dest) const {
    // unit sphere
    Eigen::Matrix<T, 3, Eigen::Dynamic> pu = in.colwise().normalized();

    // point moved of epsilon, according to mei model, and projected on z1 plane
    Eigen::Matrix<T, 2, Eigen::Dynamic>  pz1 = Utils::unitSpherePoint_to_z1_mei(pu, this->epsilon());

    // distort
    Eigen::Matrix<T, 2, Eigen::Dynamic>  pz1d = distModel.distort(pz1);

    // standard projection
    Camera<T>::pointsZ1_to_image(pz1d, dest);
  }

  // ------------------------------------------------

  template class Camera<float>;
  template class Camera<double>;

  template class CameraMei<double>;
  template class CameraMei<float>;
}
