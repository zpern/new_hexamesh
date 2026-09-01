# Symmetry and Internal Surface Growth Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make Symmetry and Internal regions constrain smoothed boundary-layer directions and final positions, remain absent from collision obstacles, and produce correctly tagged side surfaces in final boundary output.

**Architecture:** Parse Symmetry/Internal zones into the existing surface tags, build one reusable `SlidingSurfaceSet` per input mesh, and pass it through regular-layer stepping. Axis-aligned regions use analytic projection; other regions use a deterministic nearest-triangle index. Accepted volume-cell boundary faces carry an explicit boundary kind so shared internal faces cancel while exposed sliding faces retain their original kind and region.

**Tech Stack:** C++17, Eigen, existing `Result`/variant errors, existing `BinaryAabbTree`, CMake/CTest, legacy VTK output.

## Global Constraints

- Reuse the algorithms and ordering from BLMesh `symmetry.h`, `NormalSmoothStrategy`, `GenerateBLMesh()`, `PropagateNode()`, and `UpdateSymmetry()` without importing BLMesh node types or libigl.
- Keep Wall as the only source of `GrowthPatch` and `GrowthFront` faces.
- Treat Symmetry and Internal identically for direction constraint, final-position projection, collision exclusion, propagation, and side-face generation; preserve their distinct enum values in output.
- Detect AxisX/AxisY/AxisZ with `reference_length = 0.02 * average_edge_length` and `axis_epsilon = 0.1 * reference_length`; fall back to Curved when no axis matches.
- Apply the final constraint after direction/height field smoothing and project the final candidate position before isotropic, quality, and collision checks.
- Use at most 20 alternating-projection iterations in ascending region-ID order.
- For an edge shared by multiple sliding regions, assign its single side face to the smallest shared region ID, matching reference `UpdateSymmetry()` behavior.
- Exclude both original Symmetry and Internal faces from the collision index.
- `farfield_boundary` contains Farfield, BoundaryLayerInterface, Symmetry, and Internal; `boundary_layer_top` contains only BoundaryLayerInterface.
- Preserve unrelated user changes, especially `src/transition/reserved_layer_transition.cpp`.

---

## File Map

- `src/io/boundary_condition_map.cpp`: parse the two new boundary-map sections.
- `include/boundary_mesh/spatial/triangle_surface_index.hpp`, `src/spatial/triangle_surface_index.cpp`: deterministic closest-point queries over triangulated surfaces.
- `include/boundary_mesh/growth/sliding_surface.hpp`: public sliding-surface data, projection results, and catalog interface.
- `include/boundary_mesh/growth/sliding_surface_builder.hpp`, `src/growth/sliding_surface_builder.cpp`: build axis or curved surfaces by region.
- `include/boundary_mesh/growth/sliding_constraints.hpp`, `src/growth/sliding_constraint_builder.cpp`: consume the catalog and constrain direction/final position.
- `include/boundary_mesh/growth/regular_layer_stepper.hpp`, `src/growth/regular_layer_stepper.cpp`: apply constraints after smoothing and locally reject affected faces.
- `include/boundary_mesh/growth/regular_layer_growth.hpp`, `include/boundary_mesh/growth/regular_layer_growth_error.hpp`, `src/growth/regular_layer_generator.cpp`: propagate the catalog, stop reason, and accepted projected coordinates.
- `include/boundary_mesh/growth/exposed_boundary.hpp`, `src/growth/exposed_boundary.cpp`: carry `SurfaceBoundaryKind` and classify sliding side faces.
- `src/growth/farfield_boundary_builder.cpp`: retain sliding faces in the complete boundary while top extraction remains interface-only.
- `src/spatial/collision_index.cpp`: skip Symmetry and Internal.
- `CMakeLists.txt`, `tests/CMakeLists.txt`: register sources/tests.

---

### Task 1: Boundary-map compatibility

**Files:**
- Modify: `src/io/boundary_condition_map.cpp:95-100`
- Modify: `tests/unit/io/boundary_condition_map_test.cpp`
- Modify: `README.md` boundary-map example and limitations

**Interfaces:**
- Consumes: `SurfaceBoundaryKind::{Wall,Farfield,Symmetry,Internal}`.
- Produces: `BoundaryZoneEntry{zone_id, kind, region_id}` for all four sections.

- [ ] **Step 1: Extend the parser test with all four sections**

Add a case that writes:

```cpp
if (!writeText(path,
        "Internal:\n4\nSymmetry:\n3\nFar:\n1\nWall:\n2\n"))
    return 20;
const auto mixed = readBoundaryConditionMap(path, {1, 2, 3, 4});
if (!mixed.hasValue() ||
    mixed.value().find(1)->kind != SurfaceBoundaryKind::Farfield ||
    mixed.value().find(2)->kind != SurfaceBoundaryKind::Wall ||
    mixed.value().find(3)->kind != SurfaceBoundaryKind::Symmetry ||
    mixed.value().find(4)->kind != SurfaceBoundaryKind::Internal)
    return 21;
```

Also add `"Periodic:\n1\n"` to the invalid-section cases so unknown sections remain rejected.

- [ ] **Step 2: Run the focused test and verify failure**

Run: `cmake --build build --config Debug --target boundary_mesh_boundary_condition_map_test && ctest --test-dir build -C Debug -R boundary_mesh_boundary_condition_map_test --output-on-failure`

Expected: FAIL because `Symmetry:` or `Internal:` produces `InvalidSection`.

- [ ] **Step 3: Implement exact section mapping**

Replace the two-section branch with:

```cpp
const auto sectionKind = [](const std::string &name)
    -> std::optional<SurfaceBoundaryKind>
{
    if (name == "Far:") return SurfaceBoundaryKind::Farfield;
    if (name == "Wall:") return SurfaceBoundaryKind::Wall;
    if (name == "Symmetry:") return SurfaceBoundaryKind::Symmetry;
    if (name == "Internal:") return SurfaceBoundaryKind::Internal;
    return std::nullopt;
};

if (const auto kind = sectionKind(line))
{
    has_section = true;
    current_kind = *kind;
    continue;
}
```

Add `<optional>`. Keep all existing duplicate/missing-zone validation unchanged. Update README with the four-section example and remove the statement that Symmetry is unsupported.

- [ ] **Step 4: Run focused IO tests**

Run: `cmake --build build --config Debug --target boundary_mesh_boundary_condition_map_test boundary_mesh_cgns_surface_reader_test && ctest --test-dir build -C Debug -R "boundary_mesh_(boundary_condition_map|cgns_surface_reader)_test" --output-on-failure`

Expected: both tests PASS.

- [ ] **Step 5: Commit**

```bash
git add src/io/boundary_condition_map.cpp tests/unit/io/boundary_condition_map_test.cpp README.md
git commit -m "feat: parse symmetry and internal boundary zones"
```

---

### Task 2: Deterministic closest-point surface index

**Files:**
- Create: `include/boundary_mesh/spatial/triangle_surface_index.hpp`
- Create: `src/spatial/triangle_surface_index.cpp`
- Create: `tests/unit/spatial/triangle_surface_index_test.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Produces: `SurfaceTriangle`, `ClosestSurfacePoint`, `TriangleSurfaceIndex::build(...)`, and `TriangleSurfaceIndex::closestPoint(...)`.
- Consumes later: Task 3 `SlidingSurfaceBuilder`.

- [ ] **Step 1: Write closest-point tests**

Use this public interface in the new test:

```cpp
std::vector<SurfaceTriangle> triangles{{
    {Point3{0,0,0}, Point3{1,0,0}, Point3{0,1,0}},
    SurfaceFaceId{7}, 0}};
const auto index = TriangleSurfaceIndex::build(triangles);
if (!index.hasValue()) return 1;
const auto interior = index.value().closestPoint(Point3{0.2,0.3,2.0});
if (!interior.hasValue() ||
    (interior.value().point - Point3{0.2,0.3,0.0}).norm() > 1e-12 ||
    interior.value().source_face_id != SurfaceFaceId{7}) return 2;
const auto vertex = index.value().closestPoint(Point3{-1,-1,0});
if (!vertex.hasValue() || vertex.value().point.norm() > 1e-12) return 3;
```

Add a tie test with two equally distant triangles and assert the lower `(source_face_id, local_triangle_id)` wins. Add non-finite and degenerate build failures.

- [ ] **Step 2: Register and run the missing implementation test**

Add the source to `boundary_mesh_spatial`; add executable/test `boundary_mesh_triangle_surface_index_test` linked to `BoundaryMesh::Spatial`.

Run: `cmake -S . -B build -DBOUNDARY_MESH_ENABLE_CGNS_IO=ON -DBUILD_TESTING=ON && cmake --build build --config Debug --target boundary_mesh_triangle_surface_index_test`

Expected: compile FAIL because the header/types do not exist.

- [ ] **Step 3: Add the public types**

```cpp
struct SurfaceTriangle
{
    std::array<Point3, 3> points;
    SurfaceFaceId source_face_id{};
    std::uint32_t local_triangle_id{};
};

struct ClosestSurfacePoint
{
    Point3 point{Point3::Zero()};
    Vector3 unit_normal{Vector3::Zero()};
    Scalar squared_distance{};
    SurfaceFaceId source_face_id{};
    std::uint32_t local_triangle_id{};
};

class TriangleSurfaceIndex
{
public:
    static Result<TriangleSurfaceIndex, SpatialError>
    build(std::vector<SurfaceTriangle> triangles);
    Result<ClosestSurfacePoint, SpatialError>
    closestPoint(const Point3 &query) const;
private:
    std::vector<SurfaceTriangle> triangles_;
    BinaryAabbTree tree_;
};
```

- [ ] **Step 4: Implement closest point and deterministic search**

Implement the Ericson region tests for point-to-triangle closest point. Use a best-first or expanding-AABB query over `BinaryAabbTree`; if extending the tree with `queryNearestCandidates()` is smaller and deterministic, add that method and test it. Compare squared distance first, then `(source_face_id, local_triangle_id)` on exact ties. Reject non-finite queries and zero-area triangles with existing `SpatialError` values.

- [ ] **Step 5: Run the spatial test set**

Run: `cmake --build build --config Debug --target boundary_mesh_triangle_surface_index_test boundary_mesh_spatial_aabb_test boundary_mesh_spatial_binary_aabb_tree_test && ctest --test-dir build -C Debug -R "boundary_mesh_(triangle_surface_index|spatial_(aabb|binary_aabb_tree))_test" --output-on-failure`

Expected: selected tests PASS.

- [ ] **Step 6: Commit**

```bash
git add CMakeLists.txt tests/CMakeLists.txt include/boundary_mesh/spatial/triangle_surface_index.hpp src/spatial/triangle_surface_index.cpp tests/unit/spatial/triangle_surface_index_test.cpp include/boundary_mesh/spatial/binary_aabb_tree.hpp src/spatial/binary_aabb_tree.cpp
git commit -m "feat: add deterministic surface projection index"
```

---

### Task 3: Build axis and curved sliding surfaces

**Files:**
- Create: `include/boundary_mesh/growth/sliding_surface.hpp`
- Create: `include/boundary_mesh/growth/sliding_surface_builder.hpp`
- Create: `src/growth/sliding_surface_builder.cpp`
- Create: `tests/unit/growth/sliding_surface_builder_test.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: Task 2 `TriangleSurfaceIndex`.
- Produces: `SlidingSurfaceKind`, `SlidingSurface`, `SlidingSurfaceSet`, and `SlidingSurfaceBuilder::build(const SurfaceMesh&)`.

- [ ] **Step 1: Write axis/fallback/validation tests**

Test X, Y, and Z rectangular regions plus a non-axis triangle set. Assert:

```cpp
const auto surfaces = SlidingSurfaceBuilder{}.build(mesh);
if (!surfaces.hasValue()) return 1;
if (surfaces.value().find(10)->kind != SlidingSurfaceKind::AxisX) return 2;
if (surfaces.value().find(11)->kind != SlidingSurfaceKind::AxisY) return 3;
if (surfaces.value().find(12)->kind != SlidingSurfaceKind::AxisZ) return 4;
if (surfaces.value().find(13)->kind != SlidingSurfaceKind::Curved) return 5;
if (surfaces.value().find(13)->boundary_kind !=
    SurfaceBoundaryKind::Internal) return 6;
```

Add cases for duplicate region IDs used by both Symmetry and Internal, a degenerate face, and an invalid vertex reference; each must fail with `InvalidSlidingSurface` or `SlidingInputMismatch`.

- [ ] **Step 2: Run and verify compile failure**

Run: `cmake --build build --config Debug --target boundary_mesh_sliding_surface_builder_test`

Expected: compile FAIL because the builder is absent.

- [ ] **Step 3: Define the catalog**

```cpp
enum class SlidingSurfaceKind { AxisX, AxisY, AxisZ, Curved };

struct SlidingSurface
{
    std::uint32_t region_id{};
    SurfaceBoundaryKind boundary_kind{SurfaceBoundaryKind::Symmetry};
    SlidingSurfaceKind kind{SlidingSurfaceKind::Curved};
    Scalar axis_value{};
    Scalar reference_length{};
    Scalar projection_tolerance{};
    std::optional<TriangleSurfaceIndex> curved_index;
};

class SlidingSurfaceSet
{
public:
    const SlidingSurface *find(std::uint32_t region_id) const noexcept;
    const std::vector<SlidingSurface> &surfaces() const noexcept;
private:
    friend class SlidingSurfaceBuilder;
    std::vector<SlidingSurface> surfaces_;
};
```

- [ ] **Step 4: Implement BLMesh-compatible classification**

For each Symmetry/Internal region, gather unique edges and vertices, compute:

```cpp
const Scalar average_edge_length = edge_length_sum / unique_edges.size();
const Scalar reference_length = Scalar{0.02} * average_edge_length;
const Scalar axis_epsilon = Scalar{0.1} * reference_length;
```

Test X, then Y, then Z span. Use the stable mean for `axis_value`; otherwise triangulate and build `TriangleSurfaceIndex`. Sort `surfaces_` by region ID and reject one region carrying both boundary kinds.

- [ ] **Step 5: Run focused tests**

Run: `cmake --build build --config Debug --target boundary_mesh_sliding_surface_builder_test && ctest --test-dir build -C Debug -R boundary_mesh_sliding_surface_builder_test --output-on-failure`

Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add CMakeLists.txt tests/CMakeLists.txt include/boundary_mesh/growth/sliding_surface.hpp include/boundary_mesh/growth/sliding_surface_builder.hpp src/growth/sliding_surface_builder.cpp tests/unit/growth/sliding_surface_builder_test.cpp
git commit -m "feat: build planar and curved sliding surfaces"
```

---

### Task 4: Constrain smoothed directions and project final positions

**Files:**
- Modify: `include/boundary_mesh/growth/sliding_constraints.hpp`
- Modify: `include/boundary_mesh/growth/sliding_constraint_builder.hpp`
- Modify: `src/growth/sliding_constraint_builder.cpp`
- Modify: `include/boundary_mesh/growth/growth_direction_error.hpp`
- Modify: `tests/unit/growth/sliding_constraints_test.cpp`

**Interfaces:**
- Consumes: `SlidingSurfaceSet` from Task 3.
- Produces: `SlidingConstraints::constrainDirection(...)` and `SlidingConstraints::projectPosition(...)`.

- [ ] **Step 1: Rewrite tests against axis/curved catalog**

Retain existing one-plane, two-plane, redundant-plane, missing-region, and overconstraint cases. Add:

```cpp
const auto direction = constraints.value().constrainDirection(
    0, current_position, Vector3{1,2,3});
const auto position = constraints.value().projectPosition(
    0, current_position + Scalar{0.4} * direction.value());
if (!direction.hasValue() || !position.hasValue()) return 30;
if (std::abs(position.value().position.y() - expected_y) > 1e-12)
    return 31;
```

For a curved region, assert the result lies within `projection_tolerance`. For two surfaces, assert `iterations <= 20`, `position_change <= tolerance`, and `max_surface_residual <= tolerance`. Add a deliberately disjoint pair that returns `SlidingProjectionNotConverged`.

- [ ] **Step 2: Run and verify failure**

Run: `cmake --build build --config Debug --target boundary_mesh_sliding_constraints_test && ctest --test-dir build -C Debug -R boundary_mesh_sliding_constraints_test --output-on-failure`

Expected: compile/test FAIL because curved/final-position APIs are absent.

- [ ] **Step 3: Define result and errors**

```cpp
struct SlidingPositionProjection
{
    Point3 position{Point3::Zero()};
    std::uint32_t iterations{};
    Scalar position_change{};
    Scalar max_surface_residual{};
};

struct SlidingProjectionNotConverged
{
    std::size_t front_vertex_index{};
    VertexId source_vertex_id{};
    std::uint32_t layer{};
    std::vector<std::uint32_t> region_ids;
    std::uint32_t iterations{};
    Scalar position_change{};
    Scalar max_surface_residual{};
};
```

Add the new error to `GrowthDirectionError`. Change builder signature to:

```cpp
Result<SlidingConstraints, GrowthDirectionError> build(
    const SlidingSurfaceSet &surfaces,
    const GrowthFront &front,
    const FrontEvaluation &evaluation) const;
```

- [ ] **Step 4: Implement reference ordering**

Implement axis analytic projection and curved closest-point projection. `constrainDirection()` projects `current_position + raw_direction`, subtracts `current_position`, and normalizes. `projectPosition()` performs one projection for one surface or ascending-region alternating projection for at most 20 iterations, checking both position change and maximum re-projection residual.

- [ ] **Step 5: Run constraint tests**

Run: `cmake --build build --config Debug --target boundary_mesh_sliding_constraints_test && ctest --test-dir build -C Debug -R boundary_mesh_sliding_constraints_test --output-on-failure`

Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add include/boundary_mesh/growth/sliding_constraints.hpp include/boundary_mesh/growth/sliding_constraint_builder.hpp include/boundary_mesh/growth/growth_direction_error.hpp src/growth/sliding_constraint_builder.cpp tests/unit/growth/sliding_constraints_test.cpp
git commit -m "feat: project directions and positions onto sliding surfaces"
```

---

### Task 5: Integrate sliding projection into production stepping

**Files:**
- Modify: `include/boundary_mesh/growth/regular_layer_stepper.hpp`
- Modify: `include/boundary_mesh/growth/regular_layer_growth.hpp`
- Modify: `src/growth/regular_layer_stepper.cpp`
- Modify: `src/growth/regular_layer_generator.cpp`
- Modify: `tests/unit/growth/regular_layer_stepper_test.cpp`
- Modify: `tests/integration/regular_layer_growth_pipeline_test.cpp`

**Interfaces:**
- Consumes: `SlidingSurfaceSet`, Task 4 constraint/project APIs.
- Produces: projected `LayerStepResult::next_front` and local `FaceStopReason::SlidingProjectionFailure`.

- [ ] **Step 1: Add failing stepper tests**

Construct an X/Y/Z box and a curved sliding fixture. Enable nonzero direction smoothing, step one layer, and assert final coordinates are on the regions. Add a projection-failure fixture where only faces incident to the failing vertex stop with:

```cpp
FaceStopReason::SlidingProjectionFailure
```

while an unrelated face remains in `next_front`.

- [ ] **Step 2: Run and verify failure**

Run: `cmake --build build --config Debug --target boundary_mesh_regular_layer_stepper_test && ctest --test-dir build -C Debug -R boundary_mesh_regular_layer_stepper_test --output-on-failure`

Expected: FAIL because production stepping does not apply sliding constraints.

- [ ] **Step 3: Extend the stepper interface and stop reason**

```cpp
Result<LayerStepResult, RegularLayerGrowthError> step(
    const GrowthFront &current_front,
    const GrowthProfileTable &profiles,
    const FaceLayerConstraintTable &constraints,
    const SlidingSurfaceSet &sliding_surfaces,
    const RegularLayerGrowthOptions &options = {}) const;
```

Add `SlidingProjectionFailure` to `FaceStopReason` immediately before neighbor/isotropic reasons and update CLI stop-count printing to use the new enum count rather than a stale nine-element array.

- [ ] **Step 4: Apply constraints after field smoothing**

After `GrowthFieldSmoother{}.smooth(...)`, build constraints for `eligible.front`. For each vertex:

```cpp
const auto constrained = sliding.value().constrainDirection(
    vertex_index,
    eligible.front.vertices[vertex_index].position,
    field_result.value().directions[vertex_index]);
if (!constrained.hasValue()) vertex_projection_failed[vertex_index] = true;
else {
    candidate_vertex.direction = constrained.value();
    const Point3 raw = candidate_vertex.position +
        candidate_vertex.actual_height * candidate_vertex.direction;
    const auto projected = sliding.value().projectPosition(vertex_index, raw);
    if (!projected.hasValue()) vertex_projection_failed[vertex_index] = true;
    else candidate_vertex.position = projected.value().position;
}
```

Reject each face incident to a failed vertex locally before isotropic/quality evaluation. Do not turn a per-vertex nonconvergence into a global pipeline failure; catalog/build shape errors remain global.

- [ ] **Step 5: Build the catalog once in the generator**

At `generateRegularLayers()` entry, call `SlidingSurfaceBuilder{}.build(surface_mesh)`. Pass the resulting immutable set to trial/final `RegularLayerStepper::step()` calls. Update every direct stepper test call with an empty or fixture catalog.

- [ ] **Step 6: Run stepping and growth tests**

Run: `cmake --build build --config Debug --target boundary_mesh_regular_layer_stepper_test boundary_mesh_regular_layer_growth_pipeline_test && ctest --test-dir build -C Debug -R "boundary_mesh_(regular_layer_stepper_test|regular_layer_growth_pipeline_test)" --output-on-failure`

Expected: PASS; curved/axis projection tests prove final positions are used.

- [ ] **Step 7: Commit**

```bash
git add include/boundary_mesh/growth/regular_layer_stepper.hpp include/boundary_mesh/growth/regular_layer_growth.hpp src/growth/regular_layer_stepper.cpp src/growth/regular_layer_generator.cpp src/cli/boundary_mesh_command.cpp tests/unit/growth/regular_layer_stepper_test.cpp tests/integration/regular_layer_growth_pipeline_test.cpp
git commit -m "feat: enforce sliding surfaces during layer growth"
```

---

### Task 6: Exclude both sliding kinds from collision obstacles

**Files:**
- Modify: `src/spatial/collision_index.cpp:217-225`
- Modify: `tests/integration/collision_growth_pipeline_test.cpp`

**Interfaces:**
- Consumes: existing `buildOriginalSurfaceCollisionIndex()`.
- Produces: collision index containing neither original Symmetry nor Internal triangles.

- [ ] **Step 1: Add an Internal exclusion assertion**

Duplicate the existing Symmetry cube fixture with `SurfaceBoundaryKind::Internal`; assert its primitive count and accepted growth match the Symmetry case rather than the Farfield obstacle case.

- [ ] **Step 2: Run and verify failure**

Run: `cmake --build build --config Debug --target boundary_mesh_collision_growth_pipeline_test && ctest --test-dir build -C Debug -R boundary_mesh_collision_growth_pipeline_test --output-on-failure`

Expected: FAIL because Internal faces are currently indexed.

- [ ] **Step 3: Skip both kinds**

```cpp
const SurfaceBoundaryKind kind = mesh.face_tags[face_index].kind;
if (kind == SurfaceBoundaryKind::Symmetry ||
    kind == SurfaceBoundaryKind::Internal)
    continue;
```

- [ ] **Step 4: Run collision tests and commit**

Run: `cmake --build build --config Debug --target boundary_mesh_collision_growth_pipeline_test && ctest --test-dir build -C Debug -R "boundary_mesh_.*collision.*test" --output-on-failure`

Expected: all selected collision tests PASS.

```bash
git add src/spatial/collision_index.cpp tests/integration/collision_growth_pipeline_test.cpp
git commit -m "feat: exclude sliding surfaces from collision obstacles"
```

---

### Task 7: Generate and classify sliding side faces

**Files:**
- Modify: `include/boundary_mesh/growth/exposed_boundary.hpp`
- Modify: `src/growth/exposed_boundary.cpp`
- Modify: `src/growth/regular_layer_generator.cpp`
- Modify: `tests/unit/growth/exposed_boundary_test.cpp`
- Create: `tests/integration/sliding_side_boundary_pipeline_test.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: accepted lower/upper fronts and `SlidingSurfaceSet`.
- Produces: `BoundaryFace::boundary_kind` plus Symmetry/Internal side faces with smallest-shared-region ownership.

- [ ] **Step 1: Add side-face classification tests**

Extend `BoundaryFace` expectations so top faces are `BoundaryLayerInterface`. In the new integration test, build a Wall triangle bounded by one Symmetry and one Internal region, grow two layers, and count/tag side faces. Add an edge whose endpoints share `{7,9}` and assert exactly one side face is generated with region 7, not two coincident faces.

- [ ] **Step 2: Run and verify failure**

Run: `cmake --build build --config Debug --target boundary_mesh_exposed_boundary_test boundary_mesh_sliding_side_boundary_pipeline_test`

Expected: compile/test FAIL because boundary kind and sliding classification are absent.

- [ ] **Step 3: Carry explicit face kind**

```cpp
struct BoundaryFace
{
    std::vector<Point3> points;
    std::vector<CollisionVertexKey> vertex_keys;
    SurfaceFaceId source_face_id{};
    SurfaceBoundaryKind boundary_kind{
        SurfaceBoundaryKind::BoundaryLayerInterface};
    std::uint32_t region_id{};
};
```

Set bottom/top to `BoundaryLayerInterface`. For each candidate side edge, intersect the two lower vertices' sorted `sliding_region_ids`; if nonempty, select the smallest ID, look up the catalog, and assign its `boundary_kind` and `region_id`. Otherwise retain BoundaryLayerInterface and the source Wall region.

- [ ] **Step 4: Preserve cancellation and outward order**

Keep `BoundaryFaceKey` independent of kind so two copies of an internal shared face cancel even if constructed from opposite cells. Derive side point order from the accepted cell's oriented bottom/top faces. For partial stopping, let the existing exposed-face toggle operate on actual accepted cells; do not synthesize a Quad when either upper endpoint is absent.

- [ ] **Step 5: Keep dynamic sliding contacts legal**

When converting exposed faces to collision triangles, retain face kind/region metadata or filter triangles whose `boundary_kind` is Symmetry/Internal from the dynamic obstacle set. Add a two-layer test proving the second layer is not stopped by its own first-layer sliding side.

- [ ] **Step 6: Run side/exposed tests**

Run: `cmake --build build --config Debug --target boundary_mesh_exposed_boundary_test boundary_mesh_sliding_side_boundary_pipeline_test && ctest --test-dir build -C Debug -R "boundary_mesh_(exposed_boundary|sliding_side_boundary_pipeline)_test" --output-on-failure`

Expected: PASS with no duplicate side face and correct original tags.

- [ ] **Step 7: Commit**

```bash
git add tests/CMakeLists.txt include/boundary_mesh/growth/exposed_boundary.hpp src/growth/exposed_boundary.cpp src/growth/regular_layer_generator.cpp tests/unit/growth/exposed_boundary_test.cpp tests/integration/sliding_side_boundary_pipeline_test.cpp
git commit -m "feat: retain symmetry and internal layer side faces"
```

---

### Task 8: Preserve sliding sides in final boundary outputs

**Files:**
- Modify: `src/growth/farfield_boundary_builder.cpp`
- Modify: `tests/unit/growth/farfield_boundary_builder_test.cpp`
- Modify: `tests/integration/sliding_side_boundary_pipeline_test.cpp`

**Interfaces:**
- Consumes: `BoundaryFace::boundary_kind` and `region_id` from Task 7.
- Produces: complete `farfield_boundary` and interface-only `top_surface`.

- [ ] **Step 1: Add failing output assertions**

Build an exposed tracker containing one interface, one Symmetry region 30, and one Internal region 40 face. Assert `buildFarfieldBoundary()` returns all three with unchanged tags. Assert `extractBoundaryLayerTop()` returns exactly one face tagged BoundaryLayerInterface.

- [ ] **Step 2: Run and verify failure**

Run: `cmake --build build --config Debug --target boundary_mesh_farfield_boundary_builder_test && ctest --test-dir build -C Debug -R boundary_mesh_farfield_boundary_builder_test --output-on-failure`

Expected: FAIL because exposed faces are currently all relabeled BoundaryLayerInterface.

- [ ] **Step 3: Preserve explicit tags**

Change the exposed-face append from a hard-coded tag to:

```cpp
output.face_tags.push_back(
    {face.boundary_kind, face.region_id});
```

Leave `extractBoundaryLayerTop()` filtering exactly `BoundaryLayerInterface`.

- [ ] **Step 4: Run output tests and commit**

Run: `cmake --build build --config Debug --target boundary_mesh_farfield_boundary_builder_test boundary_mesh_sliding_side_boundary_pipeline_test && ctest --test-dir build -C Debug -R "boundary_mesh_(farfield_boundary_builder|sliding_side_boundary_pipeline)_test" --output-on-failure`

Expected: PASS.

```bash
git add src/growth/farfield_boundary_builder.cpp tests/unit/growth/farfield_boundary_builder_test.cpp tests/integration/sliding_side_boundary_pipeline_test.cpp
git commit -m "feat: preserve sliding tags in boundary output"
```

---

### Task 9: End-to-end verification and documentation

**Files:**
- Modify: `tests/integration/cgns_cli_pipeline_test.cpp`
- Modify: `README.md`
- Modify only if required by test registration: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: all preceding tasks.
- Produces: user-visible CLI proof and final documentation.

- [ ] **Step 1: Add a CGNS CLI fixture with all four boundary kinds**

Create numeric zones for Wall, Farfield, Symmetry, and Internal in the existing fixture style; write the four-section `.bc.txt`; run one or two layers. Assert exit code 0 and inspect output metadata/counts so Symmetry/Internal side faces are present while the top file contains only BoundaryLayerInterface faces.

- [ ] **Step 2: Run and verify the new integration case**

Run: `cmake --build build --config Debug --target boundary_mesh_cgns_cli_pipeline_test && ctest --test-dir build -C Debug -R boundary_mesh_cgns_cli_pipeline_test --output-on-failure`

Expected: PASS only when parsing, projection, collision exclusion, side generation, and output preservation are all connected.

- [ ] **Step 3: Update maintenance documentation**

Document:

```text
Symmetry/Internal zones constrain Wall-adjacent vertices.
Axis-aligned regions use analytic projection; other regions use triangulated closest-point projection.
Both kinds are excluded from collision obstacles and retained as generated side surfaces.
farfield_boundary is the complete boundary; boundary_layer_top contains only the outer BL interface.
```

Include the four-section input example and projection-failure behavior.

- [ ] **Step 4: Run formatting and the complete test suite**

Run:

```powershell
git diff --check
cmake --build build --config Debug --parallel
ctest --test-dir build -C Debug --output-on-failure
cmake --build build --config Release --parallel
ctest --test-dir build -C Release --output-on-failure
```

Expected: `git diff --check` emits no errors; Debug and Release suites report 100% tests passed.

- [ ] **Step 5: Review the final diff for scope and user changes**

Run:

```powershell
git status --short
git diff --stat HEAD~8..HEAD
git diff -- src/transition/reserved_layer_transition.cpp
```

Expected: feature commits contain only planned files; the user's pre-existing `reserved_layer_transition.cpp` change remains uncommitted and unmodified by this work.

- [ ] **Step 6: Commit**

```bash
git add tests/integration/cgns_cli_pipeline_test.cpp tests/CMakeLists.txt README.md
git commit -m "test: verify sliding surfaces end to end"
```

---

## Final Review Gate

- Confirm every Symmetry/Internal source region referenced by a Wall vertex has a catalog entry.
- Confirm direction projection occurs after field smoothing.
- Confirm final-position projection occurs before isotropic, quality, and collision evaluation.
- Confirm axis fallback to Curved is exercised by tests.
- Confirm both original sliding kinds are absent from collision primitives.
- Confirm a multi-region edge emits one side face owned by the smallest shared region.
- Confirm exposed internal duplicate faces cancel by topology key.
- Confirm full boundary and top-only output filters differ exactly as specified.
- Confirm no implementation commit includes the user's unrelated transition-file modification.
