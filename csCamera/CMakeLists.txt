project(csCamera LANGUAGES CXX)


# prepare custom project asserts, note: use standard find_package
find_package(csCore REQUIRED COMPONENTS assert CONFIG)

list(APPEND CMAKE_MODULE_PATH "${csCore_BASE_DIR}/cmake/csCore/assert/")
include(csAssert)

# set defaults for Paranoid additional configuration
csSetAssertDefaultOnForConfig(${PROJECT_NAME} ASSERT RELEASE Paranoid ON)
csSetAssertDefaultOnForConfig(${PROJECT_NAME} ASSERT RELWITHDEBINFO Paranoid ON)
csSetAssertDefaultOnForConfig(${PROJECT_NAME} ASSERT DEBUG Paranoid ON)
csSetAssertDefaultOnForConfig(${PROJECT_NAME} ASSERT PARANOID Paranoid ON)
csSetAssertDefaultOnForConfig(${PROJECT_NAME} ASSERT SAFE_MATH Paranoid ON)

csAssertGenerateProject(csCamera csCamera) #generate target csCamera::csCameraAssert
list(POP_BACK CMAKE_MODULE_PATH)

# prepare standard library
cs_add_library(csCamera STATIC)
cs_add_library(csCamera::csCamera ALIAS csCamera)

cs_find_package(csCamera PUBLIC Eigen3 REQUIRED CONFIG)
cs_find_package(csCamera PUBLIC csCore COMPONENTS core eigenUtils REQUIRED CONFIG)

cs_target_sources(csCamera
	PRIVATE_INSTALL
    Camera.h
    Camera.hpp
    CameraDistortionModel.h
    CameraDistortionModel.hpp
    Utils.h
    Utils.hpp
	PRIVATE
    Camera.cpp
    CameraDistortionModel.cpp
    Utils.cpp
		)


cs_target_link_libraries(csCamera
	PUBLIC
		Eigen3::Eigen
		csCamera::csCameraAssert
		csCameraLib::csCameraLib_csCMakeBaseInclude

)

add_subdirectory(tests)

cs_install_target(csCamera csCamera DEFAULT)

cs_finalize_target(csCamera)
