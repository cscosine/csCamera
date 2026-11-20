# csCamera

Camera models for C++17, on Eigen3: the pinhole model (`Camera<T>`) and the Mei / unified model (`CameraMei<T>`), with
an optional OpenCV-style distortion model (`CameraDistortionModel<T>`: rational, tangential, thin prism). Projection
and unprojection with analytic Jacobians, for float and double.

Built and released as part of [csfoundation](https://github.com/cscosine/csfoundation), with
[csCMake](https://github.com/cscosine/csCMake). Where the library is going: [docs/evolution-assessment.md](docs/evolution-assessment.md).
