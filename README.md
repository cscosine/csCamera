# csCamera

Camera models in C++ on Eigen: pinhole and Mei, with OpenCV-style distortion

<!-- cs-template:begin (this block is owned by cs-project-template: do not edit; the rest of the file is the project's) -->
## Getting started

csCamera is built by the project that consumes it: no setup or CI of its own. Formatting and checks:
`pre-commit install` once, then `pre-commit run --all-files`.

The shared files (setup, checks, CI, agent rules) come from
[cs-project-template](https://github.com/cscosine/cs-project-template); to bring them to its latest version, on a
branch: `copier update --defaults`.
<!-- cs-template:end -->

Camera models for C++17, on Eigen3: the pinhole model (`Camera<T>`) and the Mei / unified model (`CameraMei<T>`), with
an optional OpenCV-style distortion model (`CameraDistortionModel<T>`: rational, tangential, thin prism). Projection
and unprojection with analytic Jacobians, for float and double.

Built and released as part of [csfoundation](https://github.com/cscosine/csfoundation), with
[csCMake](https://github.com/cscosine/csCMake). Where the library is going: [docs/evolution-assessment.md](docs/evolution-assessment.md).
