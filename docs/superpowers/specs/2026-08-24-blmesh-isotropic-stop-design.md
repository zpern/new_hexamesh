# BLMesh-Style Isotropic Stop Design

## Goal

Replace the current face-area isotropic-stop criterion with a BLMesh-style,
node-centered criterion that treats triangular prism and quadrilateral hexa
growth fronts consistently while preserving the existing face-based regular
layer mesh architecture.

## Current Problem

The current implementation accepts a candidate cell and stops its source face
when

```text
average candidate side length / sqrt(current base area) >= isotropic_height
```

`sqrt(area)` has a different relationship to edge scale for triangles and
quadrilaterals. For an equilateral triangle of edge length `a`, it is about
`0.658a`; for a square it is `a`. The same nominal threshold therefore stops
regular triangular prisms substantially earlier than regular hexahedra.

## BLMesh Reference Semantics

BLMesh computes three edge-based scales for each triangular front:

```text
local_average   = arithmetic mean of edge lengths
local_geometric = geometric mean of edge lengths
local_minimum   = minimum edge length
```

It combines the local average with a global front average for the ordinary
continuation decision and uses geometric and minimum scales as hard-stop
guards. It then requires neighborhood agreement before stopping a node.

This design preserves those semantics but does not copy two implementation
artifacts from BLMesh:

- The global average is recomputed for every active layer; it is never reset to
  zero without being rebuilt.
- All indexing is expressed through the current front's explicit face-to-node
  adjacency, so the triangular-only BLMesh indexing is not reused for quads.

## Face Scale Model

For every active front face, compute edge lengths from its current-layer
geometry. A triangle contributes three perimeter edges and a quadrilateral
contributes four perimeter edges. Diagonals are not included because BLMesh's
reference quantities describe front boundary scale rather than a chosen
triangulation.

For each face, store:

```text
Lavg = arithmetic mean of perimeter edge lengths
Lgeo = geometric mean of perimeter edge lengths
Lmin = minimum perimeter edge length
```

All values must be finite and strictly positive. Existing front evaluation
already rejects degenerate faces before the isotropic decision; an invalid
scale is therefore treated as an invalid front-evaluation state rather than as
an isotropic stop.

The layer-global scale `G` is the unweighted arithmetic mean of `Lavg` across
all eligible active faces in that step. This matches BLMesh's front-wise
averaging and avoids allowing large faces to dominate through area weighting.

## Node Candidate Decision

The decision uses the candidate layer's actual side length at each vertex:

```text
h = norm(candidate_position - current_position)
H = h / isotropic_height
```

For every incident eligible face, compute:

```text
Lblend = max(0.1 * G + 0.9 * Lavg,
             0.3 * G + 0.7 * Lavg)
```

A node starts as a candidate for isotropic stop. Each incident face updates the
decision in BLMesh order:

1. If `Lblend > 0.95 * H`, the node is not yet a stop candidate.
2. If `H > 1.3 * Lgeo`, the node is a forced stop candidate.
3. If `H > 1.8 * Lmin`, the node is a forced stop candidate.

Either hard-stop guard overrides the ordinary continuation result. A vertex
with no incident eligible face is not considered by the isotropic evaluator.

## Neighborhood Consensus

Build vertex adjacency from the active face perimeter edges. A candidate node
becomes a confirmed isotropic-stop node only when every directly adjacent
active node is also an isotropic-stop candidate. This reproduces BLMesh's
neighbor-consensus intent and prevents isolated termination spikes.

The consensus is evaluated from one immutable candidate-state snapshot; it is
not updated in-place while walking vertices. The result is therefore
independent of vertex iteration order.

## Mapping Node Stops to Face Stops

The current generator produces complete prisms and hexahedra and does not
produce BLMesh-style pyramid transition cells. A source face cannot continue
if any of its vertices is confirmed stopped, because its next complete volume
cell would require every vertex.

Consequently, after a candidate cell passes quality evaluation:

- accept the current cell;
- if any vertex of its top face is a confirmed isotropic-stop node, add an
  `IsotropicHeightReached` accepted-stop event for the source face;
- otherwise retain the face in the continuing front.

Existing termination propagation then enforces
`max_neighbor_layer_difference`. Collision filtering remains later in the
existing pipeline and may remove additional accepted candidates without
changing the isotropic scale calculation.

## API and Compatibility

The public `RegularLayerGrowthOptions::isotropic_height` option remains a
finite positive scalar with default value `1`. The CLI option
`--isotropic-height` and the `FaceStopReason::IsotropicHeightReached` value are
unchanged.

No new runtime option is introduced for BLMesh's constants. The constants
`0.95`, `1.3`, `1.8`, and the two global/local blends are fixed algorithm
semantics in this change. Making them configurable is outside scope.

## Code Structure

Introduce a focused internal isotropic evaluator in the growth module rather
than expanding `RegularLayerStepper` further. It consumes the current eligible
front, candidate front, and front adjacency, and produces a boolean stop flag
for each eligible face. `RegularLayerStepper` remains responsible for turning
those flags into accepted-stop events.

Expected files:

- `include/boundary_mesh/growth/isotropic_stop_evaluator.hpp`
- `src/growth/isotropic_stop_evaluator.cpp`
- `src/growth/regular_layer_stepper.cpp`
- `CMakeLists.txt`
- `tests/unit/growth/isotropic_stop_evaluator_test.cpp`
- `tests/unit/growth/regular_layer_stepper_test.cpp`

The evaluator will return the existing growth error family when input mapping
or scale data is invalid; it will not silently classify invalid geometry as a
stop.

## Test Design

Tests are written before implementation and must first fail for the missing
behavior.

Evaluator unit tests cover:

- a regular triangular front and a regular quadrilateral front reaching the
  same normalized stopping height;
- ordinary continuation below the blended local/global scale;
- geometric-mean hard stop on an anisotropic face;
- minimum-edge hard stop on a slender triangle;
- minimum-edge hard stop on a slender quadrilateral;
- an isolated candidate node rejected by neighborhood consensus;
- a complete candidate neighborhood confirmed independently of vertex order;
- non-finite or non-positive scale input returning an error.

Stepper integration tests cover:

- the current cell remains accepted when isotropic stopping is triggered;
- the face is omitted from the next continuing layer through the existing
  accepted-stop path;
- both prism and hexa faces report `IsotropicHeightReached`;
- existing quality rejection, collision filtering, layer limits, and neighbor
  propagation tests remain unchanged and passing.

## Acceptance Criteria

- No isotropic decision uses `sqrt(face_area)`.
- Triangular and quadrilateral regular fronts have the same normalized
  stopping interpretation.
- Slender faces are protected by geometric-mean and minimum-edge guards.
- Isolated node candidates do not stop a face until direct-neighbor consensus
  is reached.
- Current-layer cells are accepted before isotropic termination is recorded.
- Public CLI/API compatibility is preserved.
- The complete configured test suite passes.
