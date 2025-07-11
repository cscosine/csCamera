#include "CameraDistortionModel.h"
#include "CameraDistortionModel.hpp"
#include "Camera.h"

#include <iostream>
#include <ostream>


#define DEBUGME if(false) std::cout << "CameraDistortionModel::"

namespace csCamera {

  
  template<class T>
  CameraDistortionModel<T>::CameraDistortionModel(
    T k1, T k2, T k3,
    T p1, T p2,
    T k4, T k5, T k6,
    T s1x, T s2x,
    T s3y, T s4y
  ) :
    _k1(k1), _k2(k2), _k3(k3),
    _k4(k4), _k5(k5), _k6(k6),
    _p1(p1), _p2(p2),
    _s1x(s1x), _s2x(s2x),
    _s3y(s3y), _s4y(s4y)
  {
  }

  template<class T>
  CameraDistortionModel<T>::CameraDistortionModel() :
    _k1(0), _k2(0), _k3(0),
    _k4(0), _k5(0), _k6(0),
    _p1(0), _p2(0),
    _s1x(0), _s2x(0),
    _s3y(0), _s4y(0)
  {
  }

  template<class T>
  CameraDistortionModel<T>::~CameraDistortionModel() {

  }

  template<class T>
  std::string CameraDistortionModel<T>::toString() const {
    std::ostringstream ss;
    ss << "--- CameraDistortionModel ---" << std::endl;
    ss << "  k1: " << _k1 << ", k2: " << _k2 << ", k3: " << _k3 << std::endl;
    ss << "  k4: " << _k4 << ", k5: " << _k5 << ", k6: " << _k6 << std::endl;
    ss << "  p1: " << _p1 << ", p2: " << _p2 << std::endl;
    ss << "  s1x: " << _s1x << ", s2x: " << _s2x << ", s3x: " << _s3y << ", s4y: " << _s4y << std::endl;

    return ss.str();
  }


  template class CameraDistortionModel<double>;
  template class CameraDistortionModel<float>;

  
}
