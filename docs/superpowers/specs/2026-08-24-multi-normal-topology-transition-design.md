# Multi-Normal Topology Transition Design

## Goal

Port BLMesh's multi-normal vertex splitting, split-edge stitching, and local
transition-volume construction into `new_boundaryMesh`, while preserving the
existing mixed Triangle/Quad growth-front representation and regular-layer
pipeline.

The transition is a topology pre-step. It moves only split vertex copies,
constructs the local transition cells around them, and directly produces the
new surface used by subsequent regular layer growth. It is not a regular
boundary-layer step.

## Reference Semantics

The design preserves these BLMesh behaviors:

- A single-normal direction first maximizes the minimum dot product against
  all incident face normals.
- Only poor-quality vertices are considered for splitting.
- Candidate splitter edges partition an ordered incident-face fan into
  contiguous groups.
- Each group receives its own optimized normal.
- A split vertex is represented by coincident copies with a shared source
  vertex identity and different branches.
- Incident faces are reconnected to their assigned copies.
- A split edge is expanded into a triangle, or into an ordered triangle strip
  when both endpoints have multiple branches.
- Split copies move during the transition; ordinary vertices have zero
  transition height.
- Bottom-layer coincident copies are merged by their original geometry.
- Cell diagonals and decomposition order follow BLMesh's point-ID ordering,
  not geometric shortest-diagonal heuristics.
- The post-transition surface is written directly from the topology produced
  by the equivalent of `BuildTopo()`; it is not extracted from the boundary
  faces of the generated transition cells.

## Pipeline

```text
input GrowthFront
    -> ordered mixed-face fans
    -> complex-vertex detection
    -> splitter-edge strategy enumeration
    -> per-group normal optimization
    -> vertex-copy and face-remapping topology
    -> split-edge triangle-strip stitching
    -> split-only transition displacement
    -> Triangle transition-cell construction
    -> reserved Quad transition-cell construction boundary
    -> transition VolumeMesh + directly written transformed GrowthFront
```

Geometric failures in implemented Triangle transition construction are atomic
per accepted transition plan. Quad-based transition regions are an explicit
temporary exception: their volume cells may be omitted while their accepted
split topology and displaced output surface are retained for the user's later
manual volume decomposition.

## Ordered Mixed-Face Fans

For every front vertex, build a cyclic or boundary-terminated ordered fan of
incident faces. Each sector records the incident face, its predecessor and
successor vertices around the center, and the face unit normal.

Triangle and Quad sectors have identical fan semantics:

- each face contributes one face normal;
- only real perimeter edges participate;
- a Quad diagonal is never introduced for fan ordering or splitter-edge
  detection;
- an incident face is an indivisible member of one normal group.

Non-manifold, branched, disconnected, or inconsistently oriented fans return
a structured error before topology modification.

## Complex Vertices and Split Plans

For a vertex with incident normals `f_i`, compute the single-normal candidate

```text
n* = arg max_n min_i dot(n, f_i)
```

and its visibility. BLMesh-compatible thresholds select vertices whose
single-normal quality warrants splitter-edge enumeration. Candidate splitter
sets partition the ordered fan into contiguous groups. Each group obtains its
own maximum-minimum-visibility direction.

The result is an immutable plan containing:

- the source vertex;
- ordered branches and their directions;
- each branch's contiguous incident-face group;
- the selected splitter edges;
- the quality of the original and selected solutions.

Plans that do not improve the BLMesh-compatible quality criterion are
discarded without changing the front.

## Surface Topology Construction

Each accepted branch creates one front vertex copy. Copies initially share the
source position and `source_vertex_id`, while a transition-local branch index
distinguishes them. Every original Triangle or Quad replaces a split corner
with the copy assigned to that face's group; its face type otherwise remains
unchanged.

If only endpoint `A` of original edge `(A, B)` is split into adjacent branches
`A0` and `A1`, add the stitched face:

```text
Triangle{A0, A1, B}
```

If both endpoints split, build the ordered branch chains on both ends and use
the BLMesh smoothest-interleaving dynamic program to produce a consistently
oriented triangle strip. The cost follows BLMesh normal-continuity semantics;
point-ID order is the deterministic tie breaker.

The output of this stage is the authoritative post-split surface connectivity.
Transition-volume construction consumes it but never reconstructs or replaces
it.

## Transition Displacement

Build bottom vertices by merging topology vertices that have the same original
surface position. Mark topology vertices that occur in a connector with two
or more coincident bottom IDs as split copies, matching
`BuildPreGenContext()`'s `duplicate_lower_ids` role.

Transition height is assigned as follows:

```text
split copy      -> configured local transition height
ordinary point -> exactly zero
```

The moved position is the topology position plus transition height times the
branch direction. Length shrinking and retry behavior may reduce split-copy
heights when collision or cell-validity checks fail, but must never give an
ordinary point a nonzero transition height.

## Triangle Transition Cells

Triangle transition construction follows BLMesh's
`BuildCandidateVolume()` semantics. It compares the three merged bottom IDs,
the three topology vertices, and membership in the split-copy set. Resulting
regions are emitted as consistently oriented Tetra cells using BLMesh's
point-ID ordering.

The important bottom-ID cases are:

- all three bottom IDs equal: one Tetra from the common bottom point and the
  three upper topology vertices;
- exactly two bottom IDs equal: use the same ID-ordered one-or-two-Tetra
  branches as BLMesh, including the additional Tetra when the third topology
  vertex is also a moving multi-normal point;
- all three bottom IDs distinct: reproduce BLMesh's ID-ordered decomposition,
  omit the region when none of the three topology vertices moves, and reject
  zero-volume Tetra candidates.

The port retains the reference algorithm's Tetra construction rather than
replacing five-vertex regions with a Pyramid abstraction.

## Quad Transition-Cell Extension Boundary

Concrete Quad transition-cell decomposition is intentionally excluded from
this implementation scope and will be supplied by the user. The port must
reserve a focused function boundary that receives all information needed by
that implementation without requiring changes to fan construction, topology
splitting, displacement, or result assembly.

The reserved operation consumes:

- the ordered Quad topology vertex IDs;
- the corresponding merged bottom IDs;
- bottom and displaced upper coordinates;
- split-copy membership for each corner;
- BLMesh-compatible stable point IDs used for diagonal decisions;
- the source face ID and transition layer metadata.

It produces zero or more standard `VolumeCell` values with matching metadata.
Until the user supplies the implementation, every Quad-based transition
region produces no transition cell. This omission does not discard the split
vertices, remapped original faces, stitching triangles, accepted displacement,
or directly written transformed front. The system must not silently
triangulate the Quad or guess a diagonal; the intentionally omitted region is
reported in transition diagnostics so the user can locate and manually
decompose it later.

## Direct Post-Transition Surface

The transformed front mirrors BLMesh's `WriteSurMesh()` behavior. It uses the
surface connectivity already produced by the topology stage and updates only
the split-copy coordinates using the accepted transition heights and
directions.

```text
transformed_front.faces    = split topology faces
split vertex position      = source position + height * branch direction
ordinary vertex position   = source position
```

Original Triangle and Quad faces retain their source face IDs. Stitching
triangles carry deterministic transition-source metadata derived from the
split edge and its incident source faces. They must participate in subsequent
regular front evaluation and growth without being misreported as an original
wall face.

No volume-boundary enumeration, internal-face cancellation, or surface
extraction is performed.

## Result Model

The transition operation returns:

```cpp
struct MultiNormalTransitionResult
{
    VolumeMesh transition_cells;
    GrowthFront transformed_front;
    std::vector<SplitVertexMapping> vertex_mapping;
    std::vector<TransitionFaceOrigin> transition_face_origins;
    std::vector<OmittedQuadTransition> omitted_quad_transitions;
};
```

`vertex_mapping` preserves the source vertex and branch identity for every
transformed-front vertex. `transition_face_origins` records the split edge or
complex vertex and all contributing source faces for generated stitching
faces. Each `OmittedQuadTransition` records the source face ID, ordered
topology vertex IDs, merged bottom IDs, and moved-corner mask needed to locate
and later implement or manually replace the missing Quad-based transition
volume. The exact storage integration with the existing growth result is fixed
in the implementation plan, but no origin or omission information may be
discarded.

## Validation and Atomic Fallback

Before committing a transition plan, validate:

- finite directions, heights, and displaced coordinates;
- valid mixed-face references and consistent face winding;
- manifold stitched surface topology;
- no duplicate or zero-area new surface face;
- finite, nonzero, correctly oriented Tetra volumes;
- no illegal transition-cell overlap or original-surface collision;
- a complete mapping from transformed vertices and faces to their origins.

On recoverable geometric failure in implemented Triangle transition regions,
reduce only the involved split-copy heights and retry within a fixed limit. If
the local plan remains invalid, discard the entire plan and retain its
original vertices, faces, and single-normal behavior. Omitted Quad transition
regions are not treated as geometric failures: retain their topology and new
surface, emit no corresponding volume cells, and record their source face and
topology vertex IDs in diagnostics.

## Code Boundaries

Implementation should keep these responsibilities separate:

- mixed-face fan construction and validation;
- complex-vertex detection and split-plan optimization;
- surface vertex copying, face remapping, and split-edge stitching;
- bottom-ID merging and split-only displacement;
- Triangle transition-cell construction;
- reserved Quad transition-cell construction;
- transition validation and atomic commit;
- transformed-front/result integration.

No legacy `BLVector`, mutable global cache, or preprocessor threshold macro is
introduced. Reference constants live in a named multi-normal configuration,
with BLMesh-compatible defaults.

## Test Design

Tests are written before implementation and first demonstrate the missing
behavior.

Required coverage includes:

- ordered Triangle-only, Quad-only, and mixed Triangle/Quad fans;
- rejection of branched, disconnected, and inconsistently wound fans;
- unchanged simple vertices and faces;
- a single split endpoint producing one stitching triangle;
- two split endpoints producing an ordered triangle strip;
- permutation-stable stitching under BLMesh point-ID ordering;
- all Triangle bottom-ID equality cases from `BuildCandidateVolume()`;
- ordinary vertices retaining exactly zero transition displacement;
- direct transformed-front connectivity matching the topology-stage output;
- absence of any volume-derived surface extraction;
- atomic rollback after invalid Triangle-derived Tetra or collision failure;
- unchanged Quads producing no transition cell;
- affected Quads producing no transition cell while retaining their split,
  displaced transformed-front faces and an omission diagnostic;
- subsequent regular Triangle-to-Prism and Quad-to-Hexa growth from a valid
  transformed front.

## Acceptance Criteria

- Complex vertices are split into ordered, source-mapped branches using
  BLMesh-compatible quality and topology semantics.
- Split edges are closed by deterministic, manifold stitching triangles.
- Only split copies move during the transition.
- Triangle transition regions reproduce `BuildCandidateVolume()`'s
  point-ID-ordered Tetra behavior.
- Quad transition decomposition is isolated behind the documented extension
  boundary and contains no guessed implementation.
- Omitted Quad transition regions do not suppress or roll back the directly
  constructed post-transition surface and are identifiable in diagnostics.
- The transformed front is written directly from split surface topology and
  accepted moved coordinates.
- No transition surface is extracted from volume-cell boundary faces.
- Implemented Triangle-region failure never commits partial local topology or
  partial Triangle transition cells; Quad-region volume omission is the sole
  documented exception.
- Existing regular Triangle/Quad growth remains compatible with the resulting
  transformed front.
