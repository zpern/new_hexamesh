# Transition Sliding Contact Permissions Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Preserve legal `SideTransition` contact with its own Symmetry/Internal region while continuing to reject real crossing and contact with every other sliding region.

**Architecture:** Extend provisional transition boundary triangles with template-accurate point/edge/face permissions and a shared low/high column context. The transition boundary checker will use that context to apply the same region-local, upper-point-referenced same-side retry used by `blmesh`, then query again while ignoring only the proven legal region.

**Tech Stack:** C++17, Eigen geometry types, existing `SlidingIntersectionIndex`, CMake/CTest, legacy ASCII VTK regression outputs.

## Global Constraints

- Symmetry and Internal use identical geometry behavior and remain separated by `region_id`.
- Multi-normal processing continues to skip sliding-surface intersection checks.
- A complete-face exemption applies only to a real `SideTransition` parent face and only to its common region.
- Artificial triangulation diagonals never receive physical-edge permission.
- `TopCap` receives point and physical-edge metadata but no automatic complete-face exemption.
- Same-side correction examines every non-associated low/high column and takes its reference normal at an associated upper point, matching `blmesh::CheckSymmetryPrismFaces`.
- A legal own-region retry ignores only that region and must still detect all other sliding regions.
- Collision stops continue through the existing direct-stop and `max_layer_diff` propagation path.

---

### Task 1: Represent Complete Transition Column Context

**Files:**
- Modify: `include/boundary_mesh/transition/transition_boundary_checker.hpp`
- Test: `tests/unit/transition/transition_boundary_checker_test.cpp`

**Interfaces:**
- Consumes: `Point3`, per-vertex `std::vector<std::uint32_t>` sliding associations.
- Produces: `SlidingColumnState` and optional shared context on `OwnedBoundaryTriangle`.

- [ ] **Step 1: Write the failing type test**

Extend the existing metadata check with:

```cpp
auto context = std::make_shared<SlidingColumnContext>();
context->low_points = {{0,0,0},{1,0,0},{0,1,0}};
context->high_points = {{0,0,0},{1,0,1},{0,1,1}};
context->low_region_ids = {{{9},{},{}}};
context->high_region_ids = {{{9},{},{}}};
permission_metadata.sliding_columns = context;
if (permission_metadata.sliding_columns->low_points.size() != 3 ||
    permission_metadata.sliding_columns->high_region_ids[0] !=
        std::vector<std::uint32_t>{9})
    return 25;
```

- [ ] **Step 2: Build to verify RED**

Run:

```powershell
cmake --build build --config Release --target boundary_mesh_transition_boundary_checker_test -j 8
```

Expected: compilation fails because `SlidingColumnContext` and `sliding_columns` do not exist.

- [ ] **Step 3: Add the minimal production types**

In `transition_boundary_checker.hpp`, include `<memory>` and add:

```cpp
struct SlidingColumnContext
{
    std::vector<Point3> low_points;
    std::vector<Point3> high_points;
    std::vector<std::vector<std::uint32_t>> low_region_ids;
    std::vector<std::vector<std::uint32_t>> high_region_ids;
};
```

Add to `OwnedBoundaryTriangle`:

```cpp
std::shared_ptr<const SlidingColumnContext> sliding_columns;
```

- [ ] **Step 4: Build and run to verify GREEN**

```powershell
cmake --build build --config Release --target boundary_mesh_transition_boundary_checker_test -j 8
& .\build\tests\Release\boundary_mesh_transition_boundary_checker_test.exe
```

Expected: build succeeds and executable exits `0`.

- [ ] **Step 5: Commit**

```powershell
git add include/boundary_mesh/transition/transition_boundary_checker.hpp tests/unit/transition/transition_boundary_checker_test.cpp
git commit -m "refactor: represent transition sliding columns"
```

---

### Task 2: Propagate Region and Physical-Edge Metadata from Templates

**Files:**
- Modify: `src/transition/provisional_transition_builder.cpp`
- Test: `tests/integration/incremental_layer_transition_pipeline_test.cpp`

**Interfaces:**
- Consumes: `GrowthFrontVertex::boundary.sliding_region_ids`, transition template triangles, `SlidingColumnContext` from Task 1.
- Produces: populated `vertex_sliding_region_ids`, `physical_edge_mask`, `complete_face_exemption_regions`, and `sliding_columns` for every provisional triangle.

- [ ] **Step 1: Write failing provisional-boundary assertions**

Add a focused fixture with a stopped quad beside one retained quad. Mark the two shared low/high columns with region `9`, build the provisional transition, locate a `BoundaryOwnerRole::SideTransition` triangle, and assert:

```cpp
assert(side.sliding_columns != nullptr);
assert(side.sliding_columns->low_points.size() == 4);
assert(side.sliding_columns->high_points.size() == 4);
assert(std::count_if(
    side.vertex_sliding_region_ids.begin(),
    side.vertex_sliding_region_ids.end(),
    [](const auto &ids) { return contains(ids, 9); }) >= 2);
assert(side.physical_edge_mask != 0);
```

Add a complete attached-side variant and assert both split triangles contain region `9` in `complete_face_exemption_regions`. Assert the shared artificial diagonal bit is absent from both triangles.

- [ ] **Step 2: Run the focused test to verify RED**

```powershell
cmake --build build --config Release --target boundary_mesh_incremental_layer_transition_pipeline_test -j 8
& .\build\tests\Release\boundary_mesh_incremental_layer_transition_pipeline_test.exe
```

Expected: assertion failure because provisional side triangles currently contain empty permission metadata.

- [ ] **Step 3: Build low/high column contexts**

In each triangle and quad transition branch, create one shared context from the parent current face:

```cpp
auto columns = std::make_shared<SlidingColumnContext>();
for (std::size_t local = 0; local < low_source_ids.size(); ++local)
{
    const auto &low_vertex = current.vertices[low_source_ids[local]];
    columns->low_points.push_back(low_vertex.position);
    columns->low_region_ids.push_back(
        low_vertex.boundary.sliding_region_ids);
    const auto candidate_vertex = candidate_vertices.find(
        cellKey(low_vertex.source_vertex_id, low_vertex.branch_id));
    if (candidate_vertex == candidate_vertices.end())
    {
        columns->high_points.push_back(low_vertex.position);
        columns->high_region_ids.push_back(
            low_vertex.boundary.sliding_region_ids);
    }
    else
    {
        const auto &high_vertex = candidate.vertices[candidate_vertex->second];
        columns->high_points.push_back(high_vertex.position);
        columns->high_region_ids.push_back(
            high_vertex.boundary.sliding_region_ids);
    }
}
```

- [ ] **Step 4: Compute permissions from the parent face topology**

Add focused helpers in the anonymous namespace:

```cpp
std::vector<std::uint32_t> commonRegions(
    const std::vector<std::vector<std::uint32_t>> &associations);

std::uint8_t physicalEdges(
    const Triangle &triangle,
    const std::vector<std::uint64_t> &parent_edge_keys);
```

`commonRegions` sorts/deduplicates each list and intersects every list. `physicalEdges` sets a local triangle edge bit only when its unordered pair of template vertex IDs occurs in the parent face's real-edge set. Pass per-point regions, this mask, complete regions only for `SideTransition`, and the shared column context through `appendOwnedTriangle`.

For `TopCap`, pass point regions and its real outer-edge mask, but pass an empty complete-region vector.

- [ ] **Step 5: Run the focused test to verify GREEN**

```powershell
cmake --build build --config Release --target boundary_mesh_incremental_layer_transition_pipeline_test -j 8
& .\build\tests\Release\boundary_mesh_incremental_layer_transition_pipeline_test.exe
```

Expected: executable exits `0` and confirms artificial diagonals remain unauthorized.

- [ ] **Step 6: Commit**

```powershell
git add src/transition/provisional_transition_builder.cpp tests/integration/incremental_layer_transition_pipeline_test.cpp
git commit -m "feat: propagate transition sliding permissions"
```

---

### Task 3: Apply blmesh-Style Low/High Same-Side Retry

**Files:**
- Modify: `src/transition/transition_boundary_checker.cpp`
- Test: `tests/unit/transition/transition_boundary_checker_test.cpp`

**Interfaces:**
- Consumes: `OwnedBoundaryTriangle::sliding_columns`, `SlidingIntersectionIndex::faceNormalAtPoint`, `signedSideToRegion`, and region-specific ignored sets.
- Produces: legal own-region retry without weakening unrelated-region detection.

- [ ] **Step 1: Add the failing same-side tests**

Create a planar Internal region at `z=0`. Add a candidate side triangle that initially touches it and a column context where column 0 is associated on both low/high points.

Case A:

```text
column 0: low/high z=0, associated with region 90
column 1: low z=0.2, high z=0.4
column 2: low z=0.3, high z=0.5
```

Assert no rollback after the own-region retry.

Case B changes only column 2 high to `z=-0.5`; assert rollback. This proves the upper point participates even when all tested lower points are on one side.

Case C adds a second Internal triangle in region `91` intersecting the candidate; assert rollback after region `90` is ignored.

- [ ] **Step 2: Run to verify RED**

```powershell
cmake --build build --config Release --target boundary_mesh_transition_boundary_checker_test -j 8
& .\build\tests\Release\boundary_mesh_transition_boundary_checker_test.exe
```

Expected: Case A rolls back because no transition same-side retry exists.

- [ ] **Step 3: Implement the minimal region retry helper**

Add helpers in `transition_boundary_checker.cpp`:

```cpp
bool containsRegion(
    const std::vector<std::uint32_t> &ids,
    std::uint32_t region);

Result<bool, SpatialError> canIgnoreOwnSlidingRegion(
    const OwnedBoundaryTriangle &owned,
    const SlidingIntersectionIndex &index,
    std::uint32_t region)
{
    if (!owned.sliding_columns) return Result<bool, SpatialError>::success(false);
    const auto &columns = *owned.sliding_columns;
    if (columns.low_points.size() != columns.high_points.size() ||
        columns.low_points.size() != columns.low_region_ids.size() ||
        columns.low_points.size() != columns.high_region_ids.size())
        return Result<bool, SpatialError>::failure(
            SpatialError::InvalidTopologyReference);

    std::vector<std::size_t> associated;
    for (std::size_t i = 0; i < columns.low_points.size(); ++i)
        if (containsRegion(columns.low_region_ids[i], region) &&
            containsRegion(columns.high_region_ids[i], region))
            associated.push_back(i);
    if (associated.empty() || associated.size() == columns.low_points.size())
        return Result<bool, SpatialError>::success(false);

    const auto reference = index.faceNormalAtPoint(
        region, columns.high_points[associated.front()]);
    if (!reference.hasValue())
        return Result<bool, SpatialError>::failure(reference.error());

    std::vector<Scalar> sides;
    for (std::size_t i = 0; i < columns.low_points.size(); ++i)
    {
        if (std::find(associated.begin(), associated.end(), i) !=
            associated.end())
            continue;
        for (const Point3 *point : {&columns.low_points[i],
                                    &columns.high_points[i]})
        {
            const auto side = index.signedSideToRegion(
                region, *point, reference.value());
            if (!side.hasValue())
                return Result<bool, SpatialError>::failure(side.error());
            sides.push_back(side.value());
        }
    }
    return Result<bool, SpatialError>::success(
        slidingSideValuesStayOnOneSide(sides, Scalar{1e-10}));
}
```

- [ ] **Step 4: Query again while ignoring only the legal region**

Replace the one-shot sliding query with a loop:

```cpp
std::set<std::uint32_t> ignored;
while (true)
{
    const auto sliding_hit = input.sliding_surface->query(
        owned[index].points, permissions, ignored);
    if (!sliding_hit.hasValue())
        return RollbackResult::failure(
            TransitionBoundaryError{sliding_hit.error()});
    if (!sliding_hit.value().intersected) break;
    const auto legal = canIgnoreOwnSlidingRegion(
        owned[index], *input.sliding_surface,
        sliding_hit.value().region_id);
    if (!legal.hasValue())
        return RollbackResult::failure(
            TransitionBoundaryError{legal.error()});
    if (!legal.value())
    {
        hit = true;
        break;
    }
    ignored.insert(sliding_hit.value().region_id);
}
```

Complete-face permissions remain handled inside `query`; they do not enter this retry loop as hits.

- [ ] **Step 5: Run unit tests to verify GREEN**

```powershell
cmake --build build --config Release --target boundary_mesh_transition_boundary_checker_test boundary_mesh_sliding_intersection_test -j 8
& .\build\tests\Release\boundary_mesh_transition_boundary_checker_test.exe
& .\build\tests\Release\boundary_mesh_sliding_intersection_test.exe
```

Expected: both executables exit `0`.

- [ ] **Step 6: Commit**

```powershell
git add src/transition/transition_boundary_checker.cpp tests/unit/transition/transition_boundary_checker_test.cpp
git commit -m "fix: preserve legal transition sliding contact"
```

---

### Task 4: Lock the Incremental Pipeline Behavior

**Files:**
- Modify: `tests/integration/sliding_surface_intersection_regression_test.cpp`
- Modify: `tests/integration/incremental_layer_transition_pipeline_test.cpp`

**Interfaces:**
- Consumes: completed permissions and same-side retry from Tasks 2–3.
- Produces: end-to-end regression coverage for legal attachment and real crossing.

- [ ] **Step 1: Add a failing attached-transition pipeline case**

Construct an incremental transition where one face stops, its neighbor continues, and the generated `SideTransition` lies on an Internal plane. Use `max_layer_diff=1`; assert the neighbor remains retained and a transition cell is generated.

- [ ] **Step 2: Add a failing upper-point crossing case**

Use the same fixture but move one non-associated high point across the Internal plane. Assert the dependent high face is rejected with a transition collision stop.

- [ ] **Step 3: Run both integration targets**

```powershell
cmake --build build --config Release --target boundary_mesh_incremental_layer_transition_pipeline_test boundary_mesh_sliding_surface_intersection_regression_test -j 8
& .\build\tests\Release\boundary_mesh_incremental_layer_transition_pipeline_test.exe
& .\build\tests\Release\boundary_mesh_sliding_surface_intersection_regression_test.exe
```

Expected: before Tasks 2–3 the attached case fails; with Tasks 2–3 implemented both executables exit `0`.

- [ ] **Step 4: Commit**

```powershell
git add tests/integration/incremental_layer_transition_pipeline_test.cpp tests/integration/sliding_surface_intersection_regression_test.cpp
git commit -m "test: cover transition sliding attachment"
```

---

### Task 5: Verify Real Plane and Curve Meshes

**Files:**
- Generate without overwriting existing files: `C:/Users/zpern/Desktop/todo/jiuyuan-quailty/internal/test_case/plane/plane_transition_permission_fixed_*`
- Generate without overwriting existing files: `C:/Users/zpern/Desktop/todo/jiuyuan-quailty/internal/test_case/curve/curve_transition_permission_fixed_*`

**Interfaces:**
- Consumes: Release CLI and original `plane.cgns`/`curve.cgns` inputs.
- Produces: VTKs and logs for quantitative and ParaView inspection.

- [ ] **Step 1: Run the focused CTest set**

```powershell
ctest --test-dir build -C Release --output-on-failure -R "(sliding|transition|incremental)"
```

Expected: `100% tests passed`.

- [ ] **Step 2: Run the complete suite**

```powershell
ctest --test-dir build -C Release --output-on-failure
```

Expected: `100% tests passed`.

- [ ] **Step 3: Generate Plane with the original parameters**

```powershell
& .\build\Release\boundary_mesh_cli.exe `
  --input 'C:\Users\zpern\Desktop\todo\jiuyuan-quailty\internal\test_case\plane\plane.cgns' `
  --first-height 0.1 --growth-ratio 1.2 --layer-count 20 `
  --maximum-skewness 1 --max-layer-diff 1 --isotropic-height 1 `
  --multi-normal false `
  --output-prefix 'C:\Users\zpern\Desktop\todo\jiuyuan-quailty\internal\test_case\plane\plane_transition_permission_fixed'
```

Expected: successful generation; high-x growth no longer collapses due to legal Internal-attached `SideTransition` contact.

- [ ] **Step 4: Generate Curve with the same parameters**

```powershell
& .\build\Release\boundary_mesh_cli.exe `
  --input 'C:\Users\zpern\Desktop\todo\jiuyuan-quailty\internal\test_case\curve\curve.cgns' `
  --first-height 0.1 --growth-ratio 1.2 --layer-count 20 `
  --maximum-skewness 1 --max-layer-diff 1 --isotropic-height 1 `
  --multi-normal false `
  --output-prefix 'C:\Users\zpern\Desktop\todo\jiuyuan-quailty\internal\test_case\curve\curve_transition_permission_fixed'
```

Expected: successful generation without unauthorized sliding-surface crossing.

- [ ] **Step 5: Audit accepted cells and compare both Plane sides**

Parse the ASCII VTK `source_face_id` and `layer` cell arrays, associate every source face with its first-layer cell-center x coordinate, split at `x=0.127`, and record source-face count plus mean and median maximum layer on each side. Run `boundary_mesh_sliding_surface_intersection_regression_test.exe` as the independent accepted-cell intersection audit and retain its generated Symmetry/Internal VTK artifacts.

Expected: no unauthorized intersections; the high-x side is not prematurely truncated by missing transition permissions.

- [ ] **Step 6: Review the final diff and commit any test-only report updates**

```powershell
git diff --check
git status --short --ignore-submodules=all
```

Expected: no whitespace errors and only intentional source/test changes. Do not delete or overwrite user-provided VTK files.
