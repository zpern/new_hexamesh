# Side Quad `same_count` Collision Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make candidate self-collision skip identical complete side Quads before triangle narrow phase and accept only machine-scale coplanar residue along a legitimate shared side-Quad edge.

**Architecture:** Keep complete parent-face metadata on `CollisionTriangle` as the source of topology truth. Compute the complete-face relation before triangle contact classification; return legal immediately for four-key-identical Quads, and use scale-aware area and distance evidence only for two-key-adjacent shared Quad edges. All unrelated contacts retain the existing classifier.

**Tech Stack:** C++17, Eigen, TiGER geometry predicates, CMake/CTest, Release CLI real-case verification.

## Global Constraints

- Key equality includes `source_vertex_id`, `layer`, and `branch_id`.
- `same_count == 4` applies only when both complete parent faces are Quads.
- `same_count == 2` is legal only for two adjacent keys in both Quads and only for machine-scale residue confined to the shared edge.
- Do not add a fixed model-unit tolerance.
- Do not suppress non-adjacent curved-case intersections.
- Use test-first red-green development.

---

### Task 1: Complete-Quad relation and shared-edge residue

**Files:**
- Modify: `tests/unit/spatial/triangle_contact_test.cpp`
- Modify: `src/spatial/triangle_contact.cpp:465-510,804-850`

**Interfaces:**
- Consumes: `CollisionTriangle::boundary_vertex_keys`, `boundary_vertex_count`, existing `sameKey()`, and internal `TriangleContactEvidence`.
- Produces: internal `BoundaryRelation` with `same_count`, shared key/point storage, adjacency, and local scale; internal coplanar overlap measure; existing public `hasIllegalTriangleContact(const CollisionTriangle &, const CollisionTriangle &)` remains unchanged.

- [ ] **Step 1: Add the exact plane-failure regression and controls**

Keep the existing identical-Quad tests, which already establish the required `same_count == 4` public behavior. Add two parent Quads modeled on the recorded plane failure:

```cpp
const Point3 center_bottom{-58.0, -19.0, 4.64708e-10};
const Point3 center_top{-58.0, -18.8772, -6.39795e-06};
const std::array<Point3, 4> left_side{{
    {-58.0, -18.8889, -2.05136}, center_bottom,
    center_top, {-58.0, -18.7703, -2.03684}}};
const std::array<Point3, 4> right_side{{
    center_bottom, {-58.0, -18.9, 2.05},
    {-58.0, -18.7705, 2.03684}, center_top}};
```

Assign complete keys so the Quads share only `{1843,0,0}` and
`{1843,1,0}`. Select the recorded contacting splits and assert
`hasIllegalTriangleContact(...) == false`.

Add a clear-overlap control with the same two shared keys but move each
non-shared vertex to the same side of the shared edge by `0.1`; assert it is
illegal. Add an unrelated-key crossing control and assert it is illegal. Add a
branch-difference identical-geometry control and assert it is illegal.

- [ ] **Step 2: Run the targeted test and verify RED**

Run:

```powershell
cmake --build build --config Debug --target boundary_mesh_spatial_triangle_contact_test -j 8
ctest --test-dir build -C Debug -R '^boundary_mesh_spatial_triangle_contact_test$' --output-on-failure
```

Expected: the recorded plane shared-edge assertion fails because current code treats every positive coplanar polygon area as illegal; all controls pass.

- [ ] **Step 3: Implement complete-face relation, early identity, and residue evidence**

In `src/spatial/triangle_contact.cpp`, replace the implicit shared-key collection in `makeSharedFeature()` with a focused internal relation. The essential shape is:

```cpp
struct BoundaryRelation
{
    std::size_t first_count{};
    std::size_t second_count{};
    std::size_t same_count{};
    std::array<CollisionVertexKey, 4> shared_keys{};
    std::array<Point3, 4> shared_points{};
    bool shared_keys_adjacent{};
};

BoundaryRelation boundaryRelation(
    const CollisionTriangle &first,
    const CollisionTriangle &second);
```

Count every unique full-boundary key once, require exactly identical coordinates for a topological match as current behavior does, and calculate adjacency with `adjacentBoundaryKeys()`. At the beginning of `hasIllegalTriangleContact(const CollisionTriangle &, ...)`, before `contactEvidence()`:

```cpp
const BoundaryRelation relation = boundaryRelation(first, second);
if (relation.first_count == 4 &&
    relation.second_count == 4 &&
    relation.same_count == 4)
{
    return Result<bool, SpatialError>::success(false);
}
```

Build the existing `SharedFeature` from this relation so the remaining rules do not change. Add `Scalar coplanar_overlap_area{}` to internal `TriangleContactEvidence` and assign it from `polygonArea(polygon)` in the coplanar branch.

Compute:

```cpp
scale_squared = maximum squared edge length of both complete faces;
length_tolerance = 128 * epsilon * sqrt(scale_squared);
area_tolerance = 128 * epsilon * scale_squared;
```

For two parent Quads with `same_count == 2` and adjacent shared keys in both
Quads, accept a `CoplanarOverlap` only when its area is at most
`area_tolerance` and every evidence point is within `length_tolerance` of the
closed shared segment. Use a clamped segment projection and finite checks.

- [ ] **Step 4: Run the targeted test and verify GREEN**

Run the Step 2 commands. Expected: one test passed, zero failed.

- [ ] **Step 5: Run spatial collision regression tests**

```powershell
ctest --test-dir build -C Debug -R 'boundary_mesh_(spatial_triangle_contact|spatial_collision_index|layer_collision_checker)_test' --output-on-failure
```

Expected: all selected tests pass.

- [ ] **Step 6: Commit Task 1**

```powershell
git add -- tests/unit/spatial/triangle_contact_test.cpp src/spatial/triangle_contact.cpp
git commit -m "fix: handle side quad topology before narrow phase"
```

---
### Task 2: Full verification and real cases

**Files:**
- Verify: `src/spatial/triangle_contact.cpp`
- Verify: `tests/unit/spatial/triangle_contact_test.cpp`
- Outputs outside repository: `%TEMP%/side_quad_same_count/plane*`, `%TEMP%/side_quad_same_count/curve*`

**Interfaces:**
- Consumes: unchanged CLI and collision APIs from Task 1.
- Produces: fresh build/test and real-case evidence; no production interface changes.

- [ ] **Step 1: Build Debug and Release**

```powershell
cmake --build build --config Debug -j 8
cmake --build build --config Release -j 8
```

Expected: both commands exit `0`.

- [ ] **Step 2: Run the full test suite**

```powershell
ctest --test-dir build -C Debug --output-on-failure
ctest --test-dir build -C Release --output-on-failure
```

Expected: all configured tests pass in both configurations.

- [ ] **Step 3: Run plane**

```powershell
.\build\Release\boundary_mesh_cli.exe `
  --input 'C:\Users\zpern\Desktop\todo\jiuyuan-quailty\internal\test_case\plane\plane.cgns' `
  --first-height 0.1 --growth-ratio 1.2 --layer-count 20 `
  --maximum-skewness 1 --isotropic-height 1 `
  --output-prefix (Join-Path $env:TEMP 'side_quad_same_count\plane')
```

Expected: first-layer added-cell count increases from baseline `11296` to
`11302`; the six source faces `3627, 3841, 4011, 4566, 4910, 5251` are not
first-layer collision stops.

- [ ] **Step 4: Run curve**

```powershell
.\build\Release\boundary_mesh_cli.exe `
  --input 'C:\Users\zpern\Desktop\todo\jiuyuan-quailty\internal\test_case\curve\curve.cgns' `
  --first-height 0.1 --growth-ratio 1.2 --layer-count 20 `
  --maximum-skewness 1 --isotropic-height 1 `
  --output-prefix (Join-Path $env:TEMP 'side_quad_same_count\curve')
```

Expected: first two layers remain `4746` cells each and later non-adjacent
collision stops are not reduced to zero.

- [ ] **Step 5: Inspect final changes**

```powershell
git diff --check
git status --short
git log -3 --oneline
```

Expected: no whitespace errors; only intentional committed changes remain.
