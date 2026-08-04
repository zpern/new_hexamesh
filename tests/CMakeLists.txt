add_executable(
    boundary_mesh_core_types_test
    unit/core_types_test.cpp
)

target_link_libraries(
    boundary_mesh_core_types_test
    PRIVATE
        BoundaryMesh::Core
)

add_test(
    NAME boundary_mesh_core_types_test
    COMMAND boundary_mesh_core_types_test
)