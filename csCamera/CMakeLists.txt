project(csCamera LANGUAGES CXX)

# prepare custom project asserts, note: use standard find_package
find_package(csCore REQUIRED COMPONENTS assert CONFIG)

list(APPEND CMAKE_MODULE_PATH "${csCore_BASE_DIR}/cmake/csCore/assert/")
include(csAssert)

# set defaults for Paranoid additional configuration
cssetassertdefaultonforconfig(${PROJECT_NAME} ASSERT RELEASE Paranoid ON)
cssetassertdefaultonforconfig(${PROJECT_NAME} ASSERT RELWITHDEBINFO Paranoid ON)
cssetassertdefaultonforconfig(${PROJECT_NAME} ASSERT DEBUG Paranoid ON)
cssetassertdefaultonforconfig(${PROJECT_NAME} ASSERT PARANOID Paranoid ON)
cssetassertdefaultonforconfig(${PROJECT_NAME} ASSERT SAFE_MATH Paranoid ON)

csassertgenerateproject(csCamera csCamera) #generate target csCamera::csCameraAssert
list(POP_BACK CMAKE_MODULE_PATH)

# prepare standard library
cs_add_library(csCamera STATIC)
cs_add_library(csCamera::csCamera ALIAS csCamera)

cs_find_package(csCamera PUBLIC Eigen3 REQUIRED CONFIG)
cs_find_package(csCamera PUBLIC csCore COMPONENTS core eigenUtils REQUIRED CONFIG)

cs_target_sources(
  csCamera
  PRIVATE_INSTALL
    Camera.hpp
    Camera_impl.hpp
    CameraDistortionModel.hpp
    CameraDistortionModel_impl.hpp
    Utils_impl.hpp
    Utils.hpp
  PRIVATE Camera.cpp CameraDistortionModel.cpp
)

cs_target_link_libraries(csCamera PUBLIC Eigen3::Eigen csCamera::csCameraAssert csCamera::cscmake_base_include)

add_subdirectory(tests)

cs_install_target(csCamera COMPONENT csCamera DEFAULT)
