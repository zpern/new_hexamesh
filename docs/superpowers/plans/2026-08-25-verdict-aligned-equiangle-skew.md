# Verdict-Aligned Equiangle Skew Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Correct internal equiangle skew so oriented reflex corners can produce values above one and the cell-quality gate responds consistently with ParaView/Verdict.

**Architecture:** Keep the public face-quality interfaces unchanged and replace the unsigned corner aggregation inside `face_skewness.cpp` with a fixed-size ordered-polygon reference normal plus signed corner angles. Prism and hexahedron evaluators continue taking the maximum over their faces; focused tests prove propagation and threshold rejection before the full Release regression.

**Tech Stack:** C++17, Eigen vectors through project core types, CMake/CTest, MSVC Release, CGNS CLI benchmark.

## Global Constraints

- Do not add Verdict, VTK, or another runtime/build dependency.
- Do not write an `EquiangleSkew` array or any other new field to VTK.
- Keep `triangleEquiangularSkewness` and `quadEquiangularSkewness` signatures unchanged.
- Keep `VolumeCellQualityOptions::maximum_skewness` restricted to `[0, 1]`.
- Preserve existing validity, volume, inversion, collision, and stop-reason behavior.
- This plan does not change the skewness-aware search objective, thresholds, angles, heights, or iteration counts.

---

### Task 1: Oriented face equiangle skew

**Files:**
- Modify: `tests/unit/surface/face_skewness_test.cpp`
- Modify: `src/surface/face_skewness.cpp`
- Modify: `include/boundary_mesh/surface/face_skewness.hpp`

**Interfaces:**
- Consumes: ordered `std::array<Point3, 3>` or `std::array<Point3, 4>` and `Scalar length_tolerance`.
- Produces: unchanged `Result<Scalar, FaceEvaluationError> triangleEquiangularSkewness(...)` and `quadEquiangularSkewness(...)`; finite reflex-face results may exceed one.

- [ ] **Step 1: Add failing oriented-angle tests**

Extend `face_skewness_test.cpp` with helpers that reverse, translate, and scale fixtures, then add assertions for:

```cpp
const std::array<Point3, 4> concave_quad{
    Point3{0.0, 0.0, 0.0},
    Point3{2.0, 0.0, 0.0},
    Point3{1.0, 0.5, 0.0},
    Point3{2.0, 1.0, 0.0}};

const auto concave = quadEquiangularSkewness(
    concave_quad, length_tolerance);
if (!concave.hasValue() || !(concave.value() > Scalar{1})) return 6;

const std::array<Point3, 4> reversed{
    concave_quad[3], concave_quad[2], concave_quad[1], concave_quad[0]};
const auto reversed_result = quadEquiangularSkewness(
    reversed, length_tolerance);
if (!reversed_result.hasValue() ||
    !near(reversed_result.value(), concave.value())) return 7;
```

Add a stable warped quadrilateral and verify reversal, translation, and positive uniform scaling preserve its result. Add a bow-tie or cancelling-area quadrilateral with nonzero edges and require `FaceEvaluationError::DegenerateAreaVector`.

- [ ] **Step 2: Build and run the focused test to verify RED**

Run:

```powershell
cmake --build build --config Release --target boundary_mesh_surface_face_skewness_test
ctest --test-dir build -C Release -R boundary_mesh_surface_face_skewness_test --output-on-failure
```

Expected: the focused test fails because the current unsigned `acos` path clamps the concave fixture to at most one and does not reject a cancelling orientation.

- [ ] **Step 3: Implement the minimal oriented-corner calculation**

In `face_skewness.cpp`, add a fixed-size helper that computes the ordered polygon area vector without allocation:

```cpp
template <std::size_t VertexCount>
Result<Vector3, FaceEvaluationError> orderedUnitNormal(
    const std::array<Point3, VertexCount> &points,
    Scalar length_tolerance)
{
    Vector3 area_vector = Vector3::Zero();
    for (std::size_t i = 0; i < VertexCount; ++i)
    {
        const Point3 &current = points[i];
        const Point3 &next = points[(i + 1) % VertexCount];
        area_vector += current.cross(next);
    }
    const Scalar norm = area_vector.norm();
    if (!std::isfinite(norm) || norm <= length_tolerance * length_tolerance)
        return Result<Vector3, FaceEvaluationError>::failure(
            FaceEvaluationError::DegenerateAreaVector);
    return Result<Vector3, FaceEvaluationError>::success(area_vector / norm);
}
```

For every corner, retain the existing finite/edge validation, compute the minor angle with `atan2(cross_norm, clamped_dot)`, and classify it using the common ordered normal:

```cpp
const Scalar minor_angle = std::atan2(
    first_unit.cross(second_unit).norm(),
    std::clamp(first_unit.dot(second_unit), Scalar{-1}, Scalar{1}));
const Scalar orientation =
    first_unit.cross(second_unit).dot(unit_normal);
const Scalar interior_angle = orientation > Scalar{0}
    ? Scalar{2} * pi - minor_angle
    : minor_angle;
```

Track minimum and maximum interior angles directly, evaluate the existing equiangle formula, and return the finite value without `std::clamp(skewness, 0, 1)`. Update header comments to state that reflex geometry may return a value above one.

- [ ] **Step 4: Run the focused test to verify GREEN**

Run the same build and CTest commands from Step 2.

Expected: `boundary_mesh_surface_face_skewness_test` passes.

- [ ] **Step 5: Commit the face metric correction**

```powershell
git add include/boundary_mesh/surface/face_skewness.hpp src/surface/face_skewness.cpp tests/unit/surface/face_skewness_test.cpp
git commit -m "fix: detect reflex angles in equiangle skew"
```

### Task 2: Propagate values above one through cell quality

**Files:**
- Modify: `tests/unit/quality/hexa_evaluator_test.cpp`
- Modify only if the RED test exposes a defect: `src/quality/hexa_evaluator.cpp`
- Modify only if the RED test exposes a defect: `src/quality/prism_evaluator.cpp`

**Interfaces:**
- Consumes: corrected face skewness results from Task 1 and `VolumeCellQualityOptions{.maximum_skewness = 1.0}`.
- Produces: `VolumeCellEvaluation::skewness > 1.0` for a cell containing a reflex face and `acceptable == false`.

- [ ] **Step 1: Add a failing cell-level propagation test before Task 1 implementation is used**

Add a hexahedron fixture whose upper ordered face contains the same concave corner while retaining finite coordinates:

```cpp
HexaPoints reflex_hexa{
    Point3{0.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0},
    Point3{1.0, 0.5, 0.0}, Point3{2.0, 1.0, 0.0},
    Point3{0.0, 0.0, 1.0}, Point3{2.0, 0.0, 1.0},
    Point3{1.0, 0.5, 1.0}, Point3{2.0, 1.0, 1.0}};
VolumeCellQualityOptions permissive;
permissive.maximum_skewness = Scalar{1};
const auto reflex_result = evaluateHexa(reflex_hexa, permissive);
if (!reflex_result.hasValue() ||
    !(reflex_result.value().skewness > Scalar{1}) ||
    reflex_result.value().acceptable) return 5;
```

The evaluator may also classify the geometry as locally inverted; the assertion deliberately checks only skewness propagation and rejection so existing validity semantics remain unchanged.

- [ ] **Step 2: Run the cell-level test to verify RED on the old metric**

```powershell
cmake --build build --config Release --target boundary_mesh_hexa_evaluator_test
ctest --test-dir build -C Release -R boundary_mesh_hexa_evaluator_test --output-on-failure
```

Expected before Task 1 is applied: FAIL because skewness is clamped to one. Expected after Task 1: PASS without production changes to either volume evaluator.

- [ ] **Step 3: Make only evidence-required evaluator changes**

If Step 2 still fails after Task 1, remove only a cell-level clamp that prevents the already finite face maximum from propagating. Do not change cell validity or acceptance logic; acceptance remains:

```cpp
evaluation.acceptable =
    evaluation.validity == VolumeCellValidity::Valid &&
    evaluation.skewness <= options.maximum_skewness;
```

- [ ] **Step 4: Run face and quality regression tests**

```powershell
cmake --build build --config Release --target boundary_mesh_surface_face_skewness_test boundary_mesh_prism_evaluator_test boundary_mesh_hexa_evaluator_test boundary_mesh_volume_cell_validity_test
ctest --test-dir build -C Release -R "face_skewness|prism_evaluator|hexa_evaluator|volume_cell_validity" --output-on-failure
```

Expected: all selected tests pass.

- [ ] **Step 5: Commit cell-level regression coverage**

```powershell
git add tests/unit/quality/hexa_evaluator_test.cpp src/quality/hexa_evaluator.cpp src/quality/prism_evaluator.cpp
git commit -m "test: propagate reflex face skewness through cells"
```

### Task 3: Full verification and 2dot5 Release benchmark

**Files:**
- Modify: `docs/superpowers/specs/2026-08-25-verdict-aligned-equiangle-skew-design.md`
- Generated, not committed: `build-cgns/2dot5_verdict_skew_layer20/*`

**Interfaces:**
- Consumes: corrected face and volume metric, existing CGNS CLI, `C:\Users\zpern\Desktop\todo\jiuyuan-quailty\test_case\2dot5_cf\2dot5_cf.cgns`.
- Produces: full test evidence, optimized Release configuration evidence, 20-layer timing and maximum-skewness diagnostics.

- [ ] **Step 1: Build the non-CGNS Release and run all tests**

```powershell
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

Expected: build succeeds and all registered tests pass.

- [ ] **Step 2: Build the true CGNS Release and verify optimization flags**

```powershell
cmake --build build-cgns --config Release --target boundary_mesh_cli
Select-String -Path build-cgns\src\BoundaryMeshCore.dir\Release\BoundaryMeshCore.tlog\CL.command.1.tlog -Pattern '/O2|/Ob2|NDEBUG'
```

Expected: build succeeds and compile command evidence includes optimized Release flags such as `/O2`, `/Ob2`, and `NDEBUG`. If the target-specific tlog path differs, locate it with `rg -l "/O2" build-cgns -g "CL.command.1.tlog"` and inspect the matching file.

- [ ] **Step 3: Run the corrected 20-layer real case**

```powershell
$output = 'C:\Users\zpern\Desktop\program\BoundaryLayer\new_boundaryMesh\.worktrees\skewness-aware-normal-smoothing\build-cgns\2dot5_verdict_skew_layer20\2dot5_cf'
New-Item -ItemType Directory -Force (Split-Path -Parent $output) | Out-Null
Measure-Command {
    .\build-cgns\Release\boundary_mesh_cli.exe `
        --input 'C:\Users\zpern\Desktop\todo\jiuyuan-quailty\test_case\2dot5_cf\2dot5_cf.cgns' `
        --first-height 0.1 `
        --growth-ratio 1.2 `
        --layer-count 20 `
        --maximum-skewness 1.0 `
        --output-prefix $output
}
```

Expected: the process exits successfully, layer progress reaches its natural stopping state, and both boundary-layer and farfield VTK outputs exist. Record elapsed time, cell count, stop counts, and corrected maximum-skewness diagnostics. Do not add quality arrays to either VTK file.

- [ ] **Step 4: Compare against ParaView/Verdict evidence**

Open the generated volume mesh in ParaView, run `Mesh Quality` with `Equiangle Skew` for wedge and hexahedron, and compare the aggregate maximum with the internal diagnostic. A small documented floating-point difference is acceptable; a material discrepancy blocks upper-surface optimizer changes until a minimal fixture reproduces it.

- [ ] **Step 5: Record verification results in the design report**

Append a dated verification section to the design spec containing exact test counts, Release flag evidence, runtime, cells, stop counts, internal maximum, ParaView maximum when available, and any explained difference. Do not claim ParaView agreement if it has not been measured.

- [ ] **Step 6: Commit the verification report**

```powershell
git add docs/superpowers/specs/2026-08-25-verdict-aligned-equiangle-skew-design.md
git commit -m "docs: report corrected equiangle skew verification"
```

- [ ] **Step 7: Confirm the branch is clean**

```powershell
git status --short --branch
```

Expected: branch header only, with no modified or untracked files other than intentionally ignored generated benchmark output.
