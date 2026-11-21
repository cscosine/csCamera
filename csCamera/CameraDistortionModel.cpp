#include <csCamera/CameraDistortionModel.hpp>
#include <csCamera/CameraDistortionModel_impl.hpp>

#include <iostream>
#include <ostream>

#define DEBUGME                                                                                                        \
  if (false)                                                                                                           \
  std::cout << "CameraDistortionModel::"

namespace csCamera {

template <class T>
CameraDistortionModel<T>::CameraDistortionModel(T k1, T k2, T k3, T p1, T p2, T k4, T k5, T k6, T s1x, T s2x, T s3y,
                                                T s4y)
    : m_k1(k1), m_k2(k2), m_k3(k3), m_k4(k4), m_k5(k5), m_k6(k6), m_p1(p1), m_p2(p2), m_s1x(s1x), m_s2x(s2x),
      m_s3y(s3y), m_s4y(s4y) {}

template <class T>
CameraDistortionModel<T>::CameraDistortionModel()
    : m_k1(0), m_k2(0), m_k3(0), m_k4(0), m_k5(0), m_k6(0), m_p1(0), m_p2(0), m_s1x(0), m_s2x(0), m_s3y(0), m_s4y(0) {}

template <class T> CameraDistortionModel<T>::~CameraDistortionModel() {}

template <class T> std::string CameraDistortionModel<T>::toString() const {
  std::ostringstream ss;
  ss << "--- CameraDistortionModel ---" << std::endl;
  ss << "  k1: " << m_k1 << ", k2: " << m_k2 << ", k3: " << m_k3 << std::endl;
  ss << "  k4: " << m_k4 << ", k5: " << m_k5 << ", k6: " << m_k6 << std::endl;
  ss << "  p1: " << m_p1 << ", p2: " << m_p2 << std::endl;
  ss << "  s1x: " << m_s1x << ", s2x: " << m_s2x << ", s3x: " << m_s3y << ", s4y: " << m_s4y << std::endl;

  return ss.str();
}

template class CameraDistortionModel<double>;
template class CameraDistortionModel<float>;

} // namespace csCamera
