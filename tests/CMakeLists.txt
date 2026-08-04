# 1
add_executable(boundary_mesh_core_types_test unit/core/core_types_test.cpp)
target_link_libraries(boundary_mesh_core_types_test PRIVATE BoundaryMesh::Core)
add_test(NAME boundary_mesh_core_types_test COMMAND boundary_mesh_core_types_test)

#2
add_executable(boundary_mesh_surface_mesh_test unit/mesh/surface_mesh_test.cpp)
target_link_libraries(boundary_mesh_surface_mesh_test PRIVATE BoundaryMesh::Core)
add_test(NAME boundary_mesh_surface_mesh_test COMMAND boundary_mesh_surface_mesh_test)

#3
add_executable(boundary_mesh_volume_mesh_test unit/mesh/volume_mesh_test.cpp)
target_link_libraries(boundary_mesh_volume_mesh_test PRIVATE BoundaryMesh::Core)
add_test(NAME boundary_mesh_volume_mesh_test COMMAND boundary_mesh_volume_mesh_test)

# 4
add_executable(boundary_mesh_result_test unit/core/result_test.cpp)
target_link_libraries(boundary_mesh_result_test PRIVATE BoundaryMesh::Core)
add_test(NAME boundary_mesh_result_test COMMAND boundary_mesh_result_test)

# 5
add_executable(boundary_mesh_surface_topology_types_test unit/mesh/surface_topology_types_test.cpp)
target_link_libraries(boundary_mesh_surface_topology_types_test PRIVATE BoundaryMesh::Core)
add_test(NAME boundary_mesh_surface_topology_types_test COMMAND boundary_mesh_surface_topology_types_test)

# 6
add_executable(boundary_mesh_surface_topology_validation_test unit/mesh/surface_topology_validation_test.cpp)
target_link_libraries(boundary_mesh_surface_topology_validation_test PRIVATE BoundaryMesh::Core)
add_test(NAME boundary_mesh_surface_topology_validation_test COMMAND boundary_mesh_surface_topology_validation_test)
# 7
add_executable(
    boundary_mesh_surface_topology_builder_test
    unit/mesh/surface_topology_builder_test.cpp
)

target_link_libraries(
    boundary_mesh_surface_topology_builder_test
    PRIVATE
        BoundaryMesh::Core
)

add_test(
    NAME boundary_mesh_surface_topology_builder_test
    COMMAND boundary_mesh_surface_topology_builder_test
)

# 8
add_executable(
    boundary_mesh_surface_topology_edge_error_test
    unit/mesh/surface_topology_edge_error_test.cpp
)

target_link_libraries(
    boundary_mesh_surface_topology_edge_error_test
    PRIVATE
        BoundaryMesh::Core
)

add_test(
    NAME boundary_mesh_surface_topology_edge_error_test
    COMMAND boundary_mesh_surface_topology_edge_error_test
)

# 9
add_executable(
    boundary_mesh_surface_face_evaluation_test
    unit/surface/face_evaluation_test.cpp
)

target_link_libraries(
    boundary_mesh_surface_face_evaluation_test
    PRIVATE
        BoundaryMesh::Surface
)

add_test(
    NAME boundary_mesh_surface_face_evaluation_test
    COMMAND boundary_mesh_surface_face_evaluation_test
)
