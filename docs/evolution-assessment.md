# csCamera — Evolution Assessment

Status: draft for discussion · Date: 2026-11-15 · Scope: library at `dev` @ `241f3aa`

csCamera should grow from a small pinhole/Mei implementation into a library of
camera models, following the same principles as csLie (see
`csLie/docs/evolution-assessment.md`): speed, numerical correctness in float32
and float64, scalar-generic kernels usable with SIMD batches and on GPU,
header-only core. This document assesses the current code, compares camera
models and existing implementations, and proposes a design and roadmap.

---

## 1. Goals

| # | Goal |
|---|------|
| G1 | Support several camera models: at least **pinhole** and **Mei**, not limited to them |
| G2 | Same principles as csLie: **fast**, **numerically correct** (float32 and float64, explicit validity at the edges), **analytic Jacobians** |
| G3 | Usable in optimization (calibration, BA in csNelson / csBlockMatrix), in trackers, in visualization (csVisOpenGL "view through camera") |
| G4 | CPU batch/SIMD and GPU friendly; header-only core |
| G5 | Interoperable with common calibration tools (OpenCV, Kalibr, Basalt, COLMAP) |

---

## 2. Where the code is today

### 2.1 Content (~0.9 kLOC + small tests)

| Class / file | Content |
|---|---|
| `Camera<T>` | pinhole intrinsics fx, fy, cx, cy, image size, K / K⁻¹, field-of-view angles; z=1 plane ↔ image (with Jacobians for one point); 3D → image with an optional distortion model; virtual `points3D_to_image` |
| `CameraMei<T> : Camera<T>` | Mei / unified model: normalize to the unit sphere, shift by ξ (`epsilon`), project; distortion optional |
| `CameraDistortionModel<T>` | OpenCV-style rational + tangential + thin prism (k1–k6, p1, p2, s1–s4): distort, Jacobians (point and parameters), iterative undistort |
| `Utils` | 3D → z=1 with Jacobian, Mei sphere → z=1 (+ Jacobian w.r.t. point and ξ), z=1 → Mei sphere (unprojection) |

Packaging: **static library** (explicit instantiations for float/double), with
most code in `_impl.hpp` headers. Consumers: csSkiTracker (camera nodes,
reprojection errors) and csoptimization (csNelson BA samples, csBlockMatrix).

### 2.2 Verified defects

1. **Distortion Jacobian w.r.t. the point is wrong** whenever k2, k3, k5, k6,
   s2 or s4 are non-zero. The code uses `d(r⁴)/dx = 4x³` and `d(r⁶)/dx = 6x⁵`
   (and `4y⁵` for y), instead of `4·r²·x` and `6·r⁴·x`. Measured against
   central finite differences: **max error 4.4e-3** (≈0.5 %) with all
   coefficients set, 5e-11 with those coefficients at zero. Both `distort_jacobians`
   overloads are affected.
   Impact: Gauss–Newton/LM with a wrong Jacobian converges more slowly, and with
   non-zero residuals it converges to a point where the *wrong* gradient vanishes,
   which is not the least-squares optimum. **Calibration and BA results that use
   this Jacobian are biased.**
2. **`Camera<T>::epsilon()` returns 1.0 for the pinhole camera.** In the Mei/unified
   family the pinhole camera is ξ = 0. Code that treats every camera through the
   base class gets the wrong ξ.
3. **No validity handling.**
   - Pinhole: points with z ≤ 0 are projected (mirrored), with no flag.
   - Mei batch projection: z + ξ ≤ 0 is not flagged (only one single-point overload returns `zOk`).
   - Mei unprojection: `sqrt(1 + (1 − ξ²) r²)` becomes NaN outside the valid image region when ξ > 1.
4. **Undistortion is fixed-point iteration** (OpenCV style): 20 iterations,
   tolerance 1e-6 for both float and double. Non-convergence returns silently.
   The `icdist < 0` case is an `assert(false)` in debug and a silent `break`
   in release. Fixed-point iteration converges slowly or diverges for strong
   distortion.
5. **State inconsistencies.** `setImageSize(T, T)` stores ints and does not update
   the field-of-view angles. `scale()` truncates the image size and ignores the
   pixel-center convention (scaling should be `c' = (c + ½)·s − ½` if pixel centers
   are at integer coordinates).
6. **Asserts**: plain `assert` everywhere. The generated `csCameraAssert` target is
   linked but unused. The size checks compare against `float` types even for
   `double` (they only check sizes, so it works, but it is misleading).
   `pointsZ1_to_image_jacobian` (single-point overload) returns `Vector2d`
   regardless of `T`.
7. **Performance**: virtual calls and `Eigen::Ref<Matrix<T,3,Dynamic>>` plus
   temporaries mean **heap allocations on every call**, including single points.
8. **Tests check almost nothing**: `TestCamera` only calls functions. The
   distortion test checks distort/undistort round trip for tiny inputs, and no
   Jacobian is verified. That is why defect 1 went unnoticed.

### 2.3 Strengths worth keeping

- Mei/unified model already present (few libraries have it with Jacobians).
- Full OpenCV rational + thin-prism distortion with parameter Jacobians
  (useful for calibration).
- Templated on float/double; Jacobian-oriented API.

---

## 3. Camera models: which ones, and is Mei "the best"?

### 3.1 Main families

| Model | Params (+fx,fy,cx,cy) | Max FOV | Unprojection | Notes | Used by |
|-------|-----|-----|-----|-----|-----|
| **Pinhole** | 0 | < 180° (practically < ~120°) | closed form | baseline | everywhere |
| **Pinhole + radtan** (OpenCV 4/5/8/12/14: radial, rational, tangential, thin prism, tilt) | 4–14 | < 180° | **iterative** | de-facto standard for normal lenses | OpenCV, Kalibr, COLMAP, mrcal |
| **Kannala–Brandt (KB4, "equidistant", OpenCV fisheye)** | 4 | > 180° possible | iterative (1D polynomial root) | the most common fisheye model; KB8 is the most accurate in the Double Sphere study but 5–10× slower | OpenCV fisheye, Kalibr, Basalt, ORB-SLAM3, COLMAP |
| **Mei** = UCM + radtan | 1 + 4/5 | > 180° (ξ > 1) | iterative (because of radtan) | well known, catadioptric and fisheye; **ξ correlates strongly with focal length** | Kalibr "omni-radtan", OpenCV `omnidir`, CamOdoCal |
| **UCM** (unified, without distortion) | 1 | > 180° | closed form | Mei without distortion; reparameterized with α ∈ [0,1] (Basalt) for better conditioning | Basalt, Kalibr |
| **EUCM** (extended unified) | 2 | > 180° | closed form | very good accuracy/speed balance | Basalt, Kalibr |
| **Double Sphere (DS)** | 2 | > 180° | closed form | very low error, fast; designed for VIO | Basalt, Kalibr |
| **FOV** (Devernay–Faugeras) | 1 | < 180° | closed form | simple, used by some datasets | Basalt, COLMAP |
| **Scaramuzza OCam** (polynomial) | 5+ | > 180° | polynomial | catadioptric / fisheye | OCamCalib |
| **Generic / spline** (mrcal splined stereographic, Schöps et al. CVPR 2020) | 100s–1000s | any | iterative | the most accurate; needed for honest uncertainty estimates (mrcal) | mrcal, research |

### 3.2 About Mei

Mei is a **solid, widely used model**, and keeping it as first-class is right,
but the evidence says it is **not the best** on accuracy/speed:

- The Double Sphere paper (Usenko et al. 2018) evaluated UCM, EUCM, DS, KB4/KB8
  and FOV on 16 sequences and 6 lenses. EUCM and DS reach very low reprojection
  error with **closed-form unprojection** and few parameters. KB8 had the lowest
  error but is 5–10× slower.
- Mei adds radtan distortion on top of UCM. This makes **unprojection iterative**
  and adds parameters, and its ξ is strongly correlated with the focal length,
  which hurts calibration conditioning.
- Practical recommendation: **Mei for compatibility** (existing calibrations,
  Kalibr `omni-radtan`), **DS/EUCM as the recommended wide-angle models**,
  **KB4 for OpenCV-fisheye compatibility**, **pinhole+radtan for normal lenses**.
  Spline/generic models come later if maximum accuracy is needed.

---

## 4. Existing implementations

| Library | Language | Models | Jacobians | Batch / GPU | License | Notes |
|---------|----------|--------|-----------|-------------|---------|-------|
| **OpenCV** calib3d / fisheye / ccalib `omnidir` | C++ | pinhole+rational+thin prism+tilt, KB4, Mei | for calibration only | ❌ (some CUDA remaps) | Apache-2.0 | the compatibility reference; API is not Jacobian-oriented |
| **Basalt** (basalt-headers) | C++ header-only, Eigen | pinhole, KB4, UCM, EUCM, DS, FOV (+BAL) | ✅ point and params | ❌ | BSD-3 | closest to the desired design; templated; from the DS paper authors |
| **Kalibr** | C++/Python | pinhole, omni (Mei), DS, EUCM × radtan/equidistant/FOV | ✅ | ❌ | BSD-3 | calibration tool; its YAML format is a common interchange format |
| **COLMAP** | C++ | SIMPLE_PINHOLE … FULL_OPENCV, OPENCV_FISHEYE, FOV, THIN_PRISM_FISHEYE, RAD_TAN_THIN_PRISM_FISHEYE, (SIMPLE_)DIVISION | via Ceres autodiff | ❌ | BSD-3 | SfM; its model names are widely used |
| **mrcal** (NASA JPL) | C / Python | OpenCV4–12, CAHVOR(E), stereographic, **splined stereographic** | ✅ | ❌ | Apache-2.0 (NASA) | best reference for **accuracy and uncertainty** of calibrations |
| **CamOdoCal** | C++ | pinhole, Mei, Scaramuzza | partial | ❌ | BSD | older |
| **ORB-SLAM3** | C++ | pinhole, KB8 | ✅ | ❌ | GPL-3 | license unsuitable for reuse |
| **apex-camera-models** | Rust | pinhole, radtan, KB, FOV, UCM, EUCM, DS | ✅ | ❌ | check crate | unified trait: project / unproject / Jacobians |
| **kornia / nerfstudio / PyTorch3D** | Python | pinhole, fisheye variants | autograd | ✅ GPU | Apache-2.0 / BSD | GPU batch references |

**Verdict:** as with Lie groups, **no C++ library combines** a broad model set,
analytic Jacobians, explicit validity, float32 accuracy, SIMD batches and GPU
kernels. **Basalt's camera headers are the closest design**, and the best
starting reference for formulas and parameterizations (UCM α, EUCM, DS). OpenCV,
Kalibr and COLMAP define the **interchange conventions**, and mrcal the
**accuracy/uncertainty standard**.

---

## 5. Target design (same principles as csLie)

### 5.1 Model = parameter vector + pure kernels

Each model is a small type with a **fixed number of parameters** (compile-time)
and scalar-generic, branch-light kernels:

```cpp
template <class T> struct DoubleSphere {          // params: fx fy cx cy xi alpha
  static constexpr int N = 6;
  CSCAM_HD static bool project(const T* p, const Vec3<T>& P, Vec2<T>& uv,
                               Mat23<T>* J_P = nullptr, Mat2N<T>* J_p = nullptr);
  CSCAM_HD static bool unproject(const T* p, const Vec2<T>& uv, Vec3<T>& bearing,
                                 Mat32<T>* J_uv = nullptr, Mat3N<T>* J_p = nullptr);
  CSCAM_HD static bool isValidPoint(const T* p, const Vec3<T>& P);
};
```

- **Unproject to bearing vectors (unit vectors, S2)**, not to the z = 1 plane:
  z = 1 cannot represent FOV ≥ 180°. This ties directly into csLie's **S2**
  (tangent-space Jacobians for optimization on bearings).
- **Validity is part of every result** (`bool`, or a mask in batch mode): behind
  the camera, outside the model's valid region (e.g. DS/UCM conditions,
  negative square roots), outside the image. Never a silent NaN or a mirrored point.
- **Jacobians optional** (null pointer, or a separate overload), computed together
  with the value to share subexpressions.
- **Static polymorphism for speed** (templates, no virtual call per point).
  **Runtime polymorphism for convenience**: `std::variant<Pinhole, PinholeRadTan, KB4, Mei, UCM, EUCM, DS, FOV>`
  or a type-erased `AnyCamera` with **batch** virtual calls (one call per array, not per point).
- **Batch API** over spans / SoA, with the same SIMD backend and `select`-based
  validity masks as csLie. The same kernels compile with `__host__ __device__` for GPU.

### 5.2 Unprojection

| Model | Method |
|-------|--------|
| pinhole, UCM, EUCM, DS, FOV | closed form |
| radtan (pinhole+radtan, Mei) | **Newton / Gauss–Newton** with the analytic Jacobian (replace fixed-point iteration), per-precision tolerance, iteration cap, **convergence flag** |
| KB4 | 1-D Newton on θ (polynomial), convergence flag |

Optional **precomputed undistortion/unprojection maps** (LUT per pixel) for
dense image use (CPU remap, GPU texture). These go in an optional component.

### 5.3 Conventions (document once, test everywhere)

- **Pixel coordinates**: origin at the center of the top-left pixel (OpenCV);
  resize/crop transforms intrinsics with the `+½ / −½` rule.
- **Camera frame**: optical (z forward, x right, y down). The rotation to the
  robotics body frame is provided by csLie / csVis conventions.
- **Parameter order and names** matching Kalibr/Basalt per model, plus import/export
  for OpenCV, Kalibr YAML, COLMAP `cameras.txt`, Basalt JSON.
- **Rigs**: multi-camera extrinsics as csLie `SE3`. This is a thin layer over
  models, not part of them.

### 5.4 Numerical care (float32 and float64)

- Models with `sqrt(1 + …)` and divisions near the FOV boundary (UCM/EUCM/DS/Mei)
  need validity checks **before** the operation and stable formulations.
- KB θ = `atan2(r, z)`, not `atan(r/z)`.
- Tolerances and iteration counts per precision; float32 tested on its own.
- `using std::sqrt;` style or the csCore math traits, so float stays float
  (the same lesson as csLie's float→double promotion).

### 5.5 Dependencies and packaging

- **Header-only core** (like csLie); drop the static library and explicit instantiations.
- Depends on **csCore's base layer** only (asserts, types, math traits; see the
  csCore assessment, §4.3), never on its utilities, except in the optional `io`
  component. Optionally on **csLie** for bearing (S2) Jacobians and rig SE3. Keep
  the core usable without it, or accept the dependency, since both are foundation libraries.
- Optional components: `io` (YAML/JSON import/export, which would use csCore's future
  serialization component), `maps` (undistortion LUTs), `cuda`.

### 5.6 Python bindings (optional)

**Worth adding as a thin optional layer once the kernel API is stable (after
phase 2)**, with the same approach as csLie (§3.8 of its assessment):

- **nanobind**, with a **batch NumPy API**: `project(model, params, P[N,3]) →
  (uv[N,2], valid[N])`, `unproject(model, params, uv[N,2]) → (bearing[N,3],
  valid[N])`, optional Jacobians as `[N,2,3]` / `[N,2,K]` arrays.
- **Uses**: cross-validation against OpenCV/kornia from Python (§6), calibration
  and dataset-conversion scripts, import/export tools (§7 phase 3), notebooks
  that plot reprojection errors or FOV coverage.
- Later, **DLPack** for GPU arrays (phase 4).
- Separate optional target or package, built with scikit-build-core and
  cibuildwheel. The C++ build never depends on Python.

Not a goal in itself: kornia and OpenCV already cover Python. The value is
running *these* models and Jacobians from Python.

---

## 6. Tests and benchmarks (what would have caught the current bugs)

Per model, float32 and float64:

- **Round trip**: `unproject(project(P)) ∥ P` over the whole valid FOV, including
  edges; `project(unproject(uv)) = uv` over the whole image.
- **Jacobians vs central finite differences** in extended precision (and/or
  Ceres `Jet`), for point and parameters, at random points across the FOV.
- **Validity**: points behind the camera, at the FOV boundary, outside the image;
  no NaN for valid inputs, flags set for invalid ones.
- **Cross-validation** with golden values from OpenCV, Basalt and Kalibr for
  the same parameters.
- **Benchmarks**: ns per projection and unprojection (+Jacobians), single and
  batch, against Basalt and OpenCV.

---

## 7. Proposed roadmap

**Phase 0: Fix and test.** Correct the distortion Jacobian (§2.2-1) and pinhole
ξ (§2.2-2); add Jacobian finite-difference tests and round-trip tests; add
validity flags to existing projections. Notify csSkiTracker and csoptimization
users, because calibration/BA results computed with the old Jacobian may change.

**Phase 1: Kernel design.** Model-as-params + kernels (§5.1), bearing-vector
unprojection, Newton undistortion with convergence flag, header-only, csCore
asserts and math traits. Re-implement pinhole, pinhole+radtan (OpenCV 4–14)
and Mei. Old classes become thin wrappers.

**Phase 2: Wide-angle models.** KB4, UCM (α), EUCM, DS, FOV, following
Basalt's formulations; cross-validated against Basalt/Kalibr.

**Phase 3: Interop and rigs.** Import/export (OpenCV, Kalibr, COLMAP, Basalt);
`AnyCamera` runtime type; multi-camera rigs with csLie SE3; FOV/coverage helpers
for csVisOpenGL frustums and "view through camera".

**Phase 4: Batch, GPU, maps.** SIMD batch API, CUDA kernels, undistortion/unprojection LUTs.

**Phase 5 (on demand): Generic models.** Scaramuzza OCam, spline-based models
(mrcal-style) when calibration accuracy/uncertainty becomes a goal.

**Optional, any time after phase 2: Python bindings** (§5.6).

---

## 8. Summary

- csCamera has a good seed (pinhole, **Mei**, full OpenCV distortion with
  parameter Jacobians) but a **verified Jacobian bug** that biases optimization
  results, a wrong ξ for the pinhole camera, no validity handling, and tests that
  check almost nothing.
- **Mei is solid and should stay**, mainly for compatibility. Evidence (Double
  Sphere study) favours **DS/EUCM** for wide-angle lenses: closed-form
  unprojection, few parameters, very low error. **KB4** is needed for OpenCV
  fisheye compatibility.
- Apply the **csLie principles**: model = parameter vector + scalar-generic
  kernels, analytic Jacobians verified by tests, explicit validity, bearing-vector
  (S2) unprojection, header-only, SIMD/GPU ready.
- **Basalt's camera headers** are the closest existing design. OpenCV, Kalibr
  and COLMAP define the interchange formats; mrcal sets the accuracy bar.
- Depends on csCore's **base layer** only (and optionally csLie). **Python
  bindings** are an optional nanobind batch layer once the API is stable.

### Sources

- Double Sphere camera model (Usenko et al.): <https://arxiv.org/pdf/1807.08957>
- Wide-angle calibration review: <https://arxiv.org/pdf/2306.09014>
- apex-camera-models (Rust): <https://docs.rs/crate/apex-camera-models/0.2.0/source/README.md>
- COLMAP camera models: <https://colmap.readthedocs.io/en/latest/cameras.html>
- mrcal lens models and uncertainty: <https://sources.debian.org/src/mrcal/2.5.2-4/doc/lensmodels.org>, <https://sources.debian.org/src/mrcal/2.5.2-4/doc/uncertainty.org>
- Basalt, Kalibr, OpenCV, CamOdoCal, ORB-SLAM3, Schöps et al. (CVPR 2020): from knowledge, not re-verified in this session.
