# Zero-Layer Interface Fallback Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Ensure every Wall source face with zero accepted boundary-layer cells is emitted as its original Triangle or Quad in the final boundary-layer interface.

**Architecture:** Keep `ExposedBoundaryTracker` unchanged so collision behavior is unaffected. Extend `buildFarfieldBoundary` with an explicit list of zero-layer source face IDs; the builder appends those original Wall faces with reversed winding and `BoundaryLayerInterface` tags. `RegularLayerGenerator` derives the list from final `FaceGrowthRecord` values.

**Tech Stack:** C++17, Eigen point types, CMake/MSBuild, CTest, existing `Result` and `SpatialError` error model.

## Global Constraints

- Apply the fallback to both Triangle and Quad Wall faces.
- A Wall face with `accepted_layer_count == 0` uses its original geometry.
- A Wall face with one or more accepted layers continues to use the tracked outer top and exposed sides.
- Preserve the source Wall `region_id` and change the output kind to `BoundaryLayerInterface`.
- Reverse fallback face winding toward the farfield region.
- Do not add initial Wall faces to collision history and do not change growth decisions.
- Reject out-of-range, duplicate, or non-Wall fallback IDs with `SpatialError::InvalidTopologyReference`.

---

### Task 1: Add explicit zero-layer faces to the farfield boundary builder

**Files:**
- Modify: `include/boundary_mesh/growth/farfield_boundary_builder.hpp`
- Modify: `src/growth/farfield_boundary_builder.cpp`
- Test: `tests/unit/growth/farfield_boundary_builder_test.cpp`
- Modify: `src/growth/regular_layer_generator.cpp` only to pass an empty third argument until Task 2

**Interfaces:**
- Consumes: original `SurfaceMesh`, current `ExposedBoundaryTracker`, and stable `SurfaceFaceId` values.
- Produces: `buildFarfieldBoundary(const SurfaceMesh &, const ExposedBoundaryTracker &, const std::vector<SurfaceFaceId> &)`.

- [ ] **Step 1: Write the failing unit test**

Extend `farfield_boundary_builder_test.cpp` with an empty tracker and an original surface containing one Farfield face, one Wall Triangle, and one Wall Quad. Call the wished-for API:

```cpp
const auto zero_layer = buildFarfieldBoundary(
    original,
    ExposedBoundaryTracker{},
    {SurfaceFaceId{1}, SurfaceFaceId{2}});
assert(zero_layer.hasValue());
assert(zero_layer.value().faces.size() == 3);
assert(zero_layer.value().face_tags[1].kind ==
       SurfaceBoundaryKind::BoundaryLayerInterface);
assert(zero_layer.value().face_tags[1].region_id == 9);
assert(zero_layer.value().face_tags[2].kind ==
       SurfaceBoundaryKind::BoundaryLayerInterface);
assert(zero_layer.value().face_tags[2].region_id == 10);
assert(std::get_if<Triangle>(&zero_layer.value().faces[1]) != nullptr);
assert(std::get_if<Quad>(&zero_layer.value().faces[2]) != nullptr);
```

For each fallback face, calculate its first two edges from output coordinates and assert its normal has the opposite dot-product sign from the corresponding input face normal. Add failure assertions for duplicate ID, out-of-range ID, and a Farfield ID passed as a fallback.

- [ ] **Step 2: Run the unit target and verify RED**

Run:

```powershell
cmake --build build --config Debug --target boundary_mesh_farfield_boundary_builder_test
```

Expected: compilation fails because `buildFarfieldBoundary` does not accept the third argument.

- [ ] **Step 3: Extend the public builder signature**

In the header, include `<vector>` and declare:

```cpp
Result<SurfaceMesh, SpatialError> buildFarfieldBoundary(
    const SurfaceMesh &original_surface,
    const ExposedBoundaryTracker &exposed_boundary,
    const std::vector<SurfaceFaceId> &zero_layer_source_face_ids);
```

Update the existing generator call temporarily to pass an empty vector:

```cpp
buildFarfieldBoundary(surface_mesh, exposed_boundary, {});
```

- [ ] **Step 4: Implement validation and reversed fallback copying**

After copying original Farfield faces and before appending tracked faces, validate each ID with a `std::vector<bool>` and append it using reverse local-corner order:

```cpp
std::vector<bool> selected(original_surface.faces.size(), false);
for (const SurfaceFaceId face_id : zero_layer_source_face_ids)
{
    const std::size_t face_index = static_cast<std::size_t>(face_id);
    if (face_index >= original_surface.faces.size() ||
        selected[face_index] ||
        original_surface.face_tags[face_index].kind !=
            SurfaceBoundaryKind::Wall)
    {
        return Result<SurfaceMesh, SpatialError>::failure(
            SpatialError::InvalidTopologyReference);
    }
    selected[face_index] = true;

    const auto remapped = std::visit(
        [&](const auto &face) -> Result<SurfaceFace, SpatialError>
        {
            using Face = std::decay_t<decltype(face)>;
            Face output_face;
            for (std::size_t corner = 0;
                 corner < face.vertex_ids.size();
                 ++corner)
            {
                const VertexId source_id =
                    face.vertex_ids[face.vertex_ids.size() - 1 - corner];
                const auto output_id = appendVertex(
                    output,
                    output_keys,
                    {source_id, 0},
                    original_surface.vertices[static_cast<std::size_t>(source_id)]);
                if (!output_id.hasValue())
                    return Result<SurfaceFace, SpatialError>::failure(
                        output_id.error());
                output_face.vertex_ids[corner] = output_id.value();
            }
            return Result<SurfaceFace, SpatialError>::success(output_face);
        },
        original_surface.faces[face_index]);
    if (!remapped.hasValue())
        return Result<SurfaceMesh, SpatialError>::failure(remapped.error());
    output.faces.push_back(remapped.value());
    output.face_tags.push_back(
        {SurfaceBoundaryKind::BoundaryLayerInterface,
         original_surface.face_tags[face_index].region_id});
}
```

Check `source_id` against `original_surface.vertices.size()` before indexing, matching the existing Farfield path.

- [ ] **Step 5: Run the unit target and verify GREEN**

Run:

```powershell
cmake --build build --config Debug --target boundary_mesh_farfield_boundary_builder_test
ctest --test-dir build -C Debug -R "^boundary_mesh_farfield_boundary_builder_test$" --output-on-failure
```

Expected: `1/1` test passes.

- [ ] **Step 6: Commit Task 1**

```powershell
git add include/boundary_mesh/growth/farfield_boundary_builder.hpp `
        src/growth/farfield_boundary_builder.cpp `
        src/growth/regular_layer_generator.cpp `
        tests/unit/growth/farfield_boundary_builder_test.cpp
git commit -m "feat: support zero-layer interface faces"
```

### Task 2: Feed final zero-layer records from the generator

**Files:**
- Modify: `src/growth/regular_layer_generator.cpp`
- Test: `tests/integration/regular_layer_growth_pipeline_test.cpp`

**Interfaces:**
- Consumes: final `RegularLayerGrowthResult::faces` records after all growth layers finish.
- Produces: a deterministic ascending list of source IDs whose `accepted_layer_count` is zero.

- [ ] **Step 1: Write the failing integration test**

Create a copy of `makeMixedMesh()`, distort the hexa Wall face by moving its local vertex 6 from `y=1` to `y=0.2`, and use `maximum_skewness = 0.1`. The existing right-triangle Wall and distorted Quad Wall must both reject their first candidate. Assert:

```cpp
assert(zero_growth.hasValue());
assert(zero_growth.value().mesh.cells.empty());
assert(zero_growth.value().faces.size() == 2);
assert(zero_growth.value().faces[0].accepted_layer_count == 0);
assert(zero_growth.value().faces[1].accepted_layer_count == 0);

std::size_t interface_count = 0;
bool triangle_region_found = false;
bool quad_region_found = false;
for (const SurfaceBoundaryTag &tag :
     zero_growth.value().farfield_boundary.face_tags)
{
    if (tag.kind != SurfaceBoundaryKind::BoundaryLayerInterface)
        continue;
    ++interface_count;
    triangle_region_found = triangle_region_found || tag.region_id == 10;
    quad_region_found = quad_region_found || tag.region_id == 11;
}
assert(interface_count == 2);
assert(triangle_region_found);
assert(quad_region_found);
```

- [ ] **Step 2: Run the integration target and verify RED**

Run:

```powershell
cmake --build build --config Debug --target boundary_mesh_regular_layer_growth_pipeline_test
ctest --test-dir build -C Debug -R "^boundary_mesh_regular_layer_growth_pipeline_test$" --output-on-failure
```

Expected: executable returns nonzero because the final interface contains zero fallback faces.

- [ ] **Step 3: Pass zero-layer source IDs to the builder**

Immediately before `buildFarfieldBoundary`, derive the final list:

```cpp
std::vector<SurfaceFaceId> zero_layer_source_face_ids;
for (const FaceGrowthRecord &record : result.faces)
{
    if (record.accepted_layer_count == 0)
        zero_layer_source_face_ids.push_back(record.source_face_id);
}
const auto farfield_boundary = buildFarfieldBoundary(
    surface_mesh,
    exposed_boundary,
    zero_layer_source_face_ids);
```

Do not modify `ExposedBoundaryTracker` initialization or collision inputs.

- [ ] **Step 4: Run focused tests and verify GREEN**

Run:

```powershell
cmake --build build --config Debug --target `
    boundary_mesh_farfield_boundary_builder_test `
    boundary_mesh_regular_layer_growth_pipeline_test
ctest --test-dir build -C Debug `
    -R "boundary_mesh_(farfield_boundary_builder|regular_layer_growth_pipeline)_test" `
    --output-on-failure
```

Expected: `2/2` tests pass.

- [ ] **Step 5: Commit Task 2**

```powershell
git add src/growth/regular_layer_generator.cpp `
        tests/integration/regular_layer_growth_pipeline_test.cpp
git commit -m "fix: preserve zero-layer wall interfaces"
```

### Task 3: Full verification

**Files:**
- Verify only; no production changes expected.

**Interfaces:**
- Consumes: completed Task 1 and Task 2 commits.
- Produces: fresh build, regression, and clean-diff evidence.

- [ ] **Step 1: Run the full Debug build**

```powershell
cmake --build build --config Debug
```

Expected: exit code `0`.

- [ ] **Step 2: Run all tests**

```powershell
ctest --test-dir build -C Debug --output-on-failure
```

Expected: all registered tests pass with zero failures.

- [ ] **Step 3: Check repository state**

```powershell
git diff --check
git status --short --branch
git log -4 --oneline --decorate
```

Expected: no tracked modifications remain; the pre-existing untracked `.superpowers/` directory remains untouched.
