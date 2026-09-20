# Surface Orientation Unification Design

## Goal

Add a small reusable helper that makes the faces in each connected component of a `SurfaceMesh` use a consistent local winding. The helper does not determine whether a closed component points inward or outward. Existing topology validation remains responsible for complete validation after the helper runs.

## Public API

Add `unifySurfaceOrientation(SurfaceMesh &mesh)` to the existing `mesh_surface_orientation` module. It returns `Result<std::size_t, SurfaceOrientationError>` where the success value is the number of faces reversed.

`SurfaceOrientationError` distinguishes the input conditions that prevent safe orientation propagation:

- a face repeats a vertex;
- an edge has more than two incident faces;
- the propagated orientation constraints contradict one another, meaning the component cannot be oriented consistently.

Boundary edges are permitted.

## Algorithm

Build an undirected edge-incidence map from triangle and quad edges. Each incidence records the face and its direction relative to the canonical ordered edge.

Process faces in ascending `SurfaceFaceId` order. For every unvisited face, use it as the unchanged seed of a new connected component and run breadth-first propagation across shared edges. Two adjacent faces must traverse their common edge in opposite directions after applying their assigned flip states. Record the required flip state without modifying the mesh.

If validation or propagation fails, return an error before changing any face. If it succeeds, reverse every face marked for flipping using the same winding reversal convention as `reverseSurfaceOrientation`, and return the number changed.

## Determinism and Mutation

Face order and canonical edge order make the result deterministic. The function provides transactional behavior: errors leave vertices, faces, and boundary tags unchanged. Successful calls preserve vertices, face order, face types, and boundary tags; only vertex winding changes. Calling it again returns zero changes.

## Tests

Extend `surface_orientation_test` to cover:

- an already consistent component;
- a mixed triangle/quad component requiring a flip;
- disconnected components;
- open surfaces with boundary edges;
- repeated-vertex, non-manifold, and contradictory constraints;
- no mutation on error;
- idempotence and preservation of tags and vertices.

