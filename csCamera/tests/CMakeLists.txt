project(csCameraTest  LANGUAGES CXX)

cs_add_executable(csCameraTest)

cs_find_package(csCameraTest PRIVATE csCore COMPONENTS testUtils REQUIRED CONFIG)

include(CTest)
cs_find_package(csCameraTest PRIVATE Catch2 3 REQUIRED CONFIG)
include(Catch)

cs_target_sources(csCameraTest
	PRIVATE
	TestCamera.cpp
	TestCameraDistortionModel.cpp
  TestUtils.cpp
)

cs_target_link_libraries(csCameraTest
  PRIVATE
  csCamera::csCamera
  csCore::testUtils
  Catch2::Catch2WithMain)

# add to tests
catch_discover_tests(csCameraTest DISCOVERY_MODE "PRE_TEST")


cs_finalize_target(csCameraTest)
