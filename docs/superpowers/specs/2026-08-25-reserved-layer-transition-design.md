# Reserved-Layer Transition Design

## Purpose

Generate a conforming boundary-layer top surface when adjacent source faces stop
at different heights. The algorithm limits every low source face to at most one
high edge, reserves the outer two trial layers, and replaces selected reserved
Prism/Hexa cells with Pyramid/Tetra transition templates.

This design specifies topology and deterministic surface output. Pyramid/Tetra
quality evaluation, transition-cell intersection checks, and failure recovery
are intentionally deferred.

## Terms and Layer Counts

For each source face, record:

```cpp
struct FaceLayerState
{
    int trial_layers;
    int occupied_layers;
    int regular_layers;
};
```

- `trial_layers` is the number of candidate layers successfully produced by
  growth.
- `regular_layers` is the number of unchanged Prism/Hexa layers committed
  directly.
- `occupied_layers` is the final height after the penultimate candidate layer
  has either been retained or decomposed.

The counts are:

```text
regular_layers  = max(0, trial_layers - 2)
occupied_layers = max(0, trial_layers - 1)
```

When the user requests `N` layers, growth attempts up to `N + 2` layers. For a
face that successfully produces seven trial layers, layers 1 through 5 are
ordinary cells, layer 6 is the transition base, and layer 7 supplies geometry
for the outer transition but is not committed as an ordinary cell.

The outer two candidate layers are therefore *reserved for transition
processing*, not both discarded.

## Coordination Invariants

Only faces sharing a complete source edge are edge neighbors. Faces sharing
only a vertex are not directly compared.

After stop propagation, every edge-neighbor pair `f`, `g` must satisfy:

```text
abs(occupied_layers[f] - occupied_layers[g]) <= 1
```

For a low face `f`, an edge is a high edge when its neighbor `g` satisfies:

```text
occupied_layers[g] == occupied_layers[f] + 1
```

Every Triangle and Quad must have at most one high edge. If a face has two or
more high edges, stop propagation lowers affected neighboring layer limits and
rechecks the local neighborhood. This follows the propagation intent of the old
HexaMesh algorithm and only decreases layer limits, so the process converges.

The zero/one-layer pair is special:

```text
trial layers:    0 <-> 1
occupied layers: 0 <-> 0
```

The sole layer of the one-layer face is discarded, so this pair has no final
geometric step and creates no transition volume.

## Reserved-Cell Actions

The transition stage classifies candidate cells as:

```cpp
enum class ReservedCellAction
{
    CommitRegular,
    KeepTransitionBase,
    SplitTransitionBase,
    UseAsGeometryOnly,
    Discard
};
```

For a face with at least two trial layers:

| Source face | Candidate position | Action |
|---|---|---|
| Triangle | Layers 1 through `L-2` | Commit ordinary Prisms |
| Triangle | Penultimate layer `L-1` | Keep the Prism unchanged |
| Triangle | Outermost layer `L` | Geometry only |
| Quad | Layers 1 through `L-2` | Commit ordinary Hexas |
| Quad | Penultimate layer `L-1` | Always decompose the Hexa |
| Quad | Outermost layer `L` | Geometry only |

The penultimate Quad Hexa is decomposed regardless of whether the source face
has a high edge. The penultimate Triangle Prism is always retained unchanged.

## Transition Decision Table

| Source | Trial layers | High edges | Result |
|---|---:|---:|---|
| Triangle | 0 | 0 | Keep the initial Triangle; no volume |
| Triangle | 1 | 0 | Discard the Prism and keep the initial Triangle |
| Triangle | 1 | 1 | Discard the Prism and create one side Pyramid |
| Triangle | 2 or more | 0 | Retain through the penultimate Prism |
| Triangle | 2 or more | 1 | Retain the penultimate Prism and create one side Pyramid |
| Quad | 0 | 0 | Split the initial Quad into two Triangles; no volume |
| Quad | 1 | 0 | Discard the Hexa and split the initial Quad; no volume |
| Quad | 1 | 1 | Discard the Hexa and create one Pyramid plus one Tetra |
| Quad | 2 or more | 0 | Decompose the penultimate Hexa |
| Quad | 2 or more | 1 | Decompose the penultimate Hexa and add the side transition |

A zero-trial-layer face cannot have a real high edge: its neighbor has at most
one trial layer, which also occupies zero final layers.

## Quad Diagonal Rule

Whenever a Quad surface must be triangulated, compare its two diagonals. For
each diagonal, calculate the surface skewness of its two candidate Triangles.
Select the diagonal whose worse Triangle has the lower skewness:

```text
selected = arg min diagonal max(skewness(triangle_0), skewness(triangle_1))
```

A deterministic vertex-ID tie break resolves equal scores. The selected
diagonal belongs to that source face and is reused by its outer transition
template. It is not negotiated with another source template.

## Triangle Side-Pyramid Template

Let the only high edge of the low Triangle be `(v0, v1)`, and let `v2` be the
corner not touching that edge. Let the low and high copies of the edge be:

```text
v0_low, v1_low
v0_high, v1_high
```

Create:

```text
Pyramid(v0_low, v1_low, v1_high, v0_high, v2_low)
```

The four edge copies form the step face and `v2_low` is the apex. Other edge
directions use a cyclic permutation of this standard local numbering.

## Penultimate Quad-Hexa Decomposition

Number the penultimate Hexa as:

```text
bottom: b0, b1, b2, b3
top:    t0, t1, t2, t3
center: c
```

Create a center point `c` and five Pyramids from the bottom and four side faces:

```text
Pyramid(b0, b1, b2, b3, c)
Pyramid(b0, t0, t1, b1, c)
Pyramid(b1, t1, t2, b2, c)
Pyramid(b2, t2, t3, b3, c)
Pyramid(b3, t3, t0, b0, c)
```

Use the Quad diagonal rule on `(t0, t1, t2, t3)`. For diagonal `t0-t2`, create:

```text
Tetra(t0, t1, t2, c)
Tetra(t0, t2, t3, c)
```

For diagonal `t1-t3`, create:

```text
Tetra(t1, t2, t3, c)
Tetra(t1, t3, t0, c)
```

Thus every penultimate Quad Hexa becomes five Pyramids and two Tetrahedra. The
chosen top diagonal is retained for the outer side-transition template.

## Quad Single-High-Edge Template

Use the following canonical orientation for a low Quad whose only high edge is
`a-b`:

```text
low triangulated top:       step:

a ----- b                   e ----- f
|     / |                   |       |
|   /   |                   a ----- b
c ----- d

selected diagonal: a-d
```

The selected diagonal divides the low top into `a-b-d` and `a-c-d`. Create:

```text
Pyramid(e, f, a, b, d)
Tetra(a, c, d, e)
```

The Pyramid consumes triangle `a-b-d`; the Tetra consumes triangle `a-c-d`.
Their shared triangle `a-d-e` is internal. After removing attachment and shared
faces, this source directly emits four outer top Triangles:

```text
Triangle(a, e, c)
Triangle(e, d, c)
Triangle(e, f, d)
Triangle(f, b, d)
```

The alternate diagonal and the other three high-edge directions use consistent
rotations/reflections of this canonical template. Cell vertex order and output
Triangle orientation must be normalized to the project's outward-orientation
convention.

## Zero- and One-Layer Cases

### Zero trial layers

- A Quad is split into two initial-surface Triangles by the Quad diagonal rule.
- A Triangle remains the original Triangle.
- No transition volume is created, including next to a one-trial-layer face.

### One trial layer

The sole Prism or Hexa is never committed.

- A Triangle without a real high edge retains its initial Triangle and creates
  no volume.
- A Triangle with one real high edge creates the Triangle side Pyramid.
- A Quad without a real high edge emits the two triangulated initial-surface
  Triangles and creates no volume.
- A Quad with one real high edge applies the one-Pyramid/one-Tetra side template
  using its selected initial-surface diagonal.

A real high edge for a one-layer face can only arise next to a face with two
trial layers, because their occupied heights are zero and one respectively.

## Direct Boundary-Layer Top Output

Each source face directly returns the final top faces implied by its selected
template. The boundary-layer top is not reconstructed by scanning all volume
cell boundary faces.

```cpp
struct SourceTransitionResult
{
    std::vector<VolumeCell> volume_cells;
    std::vector<Triangle> top_faces;
};
```

Examples include:

- one Triangle for an unchanged triangular source;
- two Triangles for a triangulated Quad with no volume;
- the exposed faces of a Triangle side Pyramid;
- the four canonical Triangles `aec`, `edc`, `efd`, `fbd` for the Quad
  single-high-edge template.

The final boundary-layer top is the deterministic concatenation of all source
results in source-face order. Each output Triangle retains its source face ID,
layer metadata, and transition-template identity.

## Processing Pipeline

```text
request N layers
    -> trial growth up to N + 2 layers
    -> record per-source trial counts and candidate geometry
    -> coordinate stops and enforce occupied-height difference <= 1
    -> enforce at most one high edge per low source
    -> commit ordinary layers 1 through L-2
    -> retain penultimate Triangle Prisms
    -> decompose every penultimate Quad Hexa
    -> construct single-high-edge side transitions
    -> directly collect each source's final top Triangles
    -> output transition VolumeMesh and boundary-layer top surface
```

Construction should be planned per source and committed only after the required
template is complete, so an unsupported topology does not leave a partially
written source result.

## Deferred Work

The first implementation deliberately excludes:

- Pyramid/Tetra quality thresholds;
- transition-cell intersection checks;
- center-point optimization;
- quality-driven retry or growth rollback;
- templates for more than one high edge;
- transitions spanning more than one occupied layer;
- special coordination across vertex-only adjacency;
- global optimization across source templates.

These exclusions do not change the required topology invariants or deterministic
template selection described above.

## Acceptance Criteria

- Requested `N` layers trigger at most `N + 2` trial layers.
- `regular_layers` and `occupied_layers` follow the specified formulas.
- A zero/one trial-layer pair produces no step volume.
- Every low source has at most one high edge before transition construction.
- Every penultimate Quad Hexa is replaced by five Pyramids and two Tetrahedra.
- Every penultimate Triangle Prism is retained unchanged.
- A canonical Quad high-edge case with diagonal `a-d` creates
  `Pyramid(e,f,a,b,d)` and `Tetra(a,c,d,e)`.
- That case emits exactly `aec`, `edc`, `efd`, and `fbd`, modulo normalized
  outward orientation.
- Zero-layer Quads emit two Triangles; zero-layer Triangles remain unchanged.
- One-layer ordinary Prism/Hexa cells are never committed.
- The final boundary-layer top is produced directly from per-source results.
