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

# 10
add_executable(
    boundary_mesh_growth_patch_test
    unit/growth/growth_patch_test.cpp
)

target_link_libraries(
    boundary_mesh_growth_patch_test
    PRIVATE
        BoundaryMesh::BoundaryLayer
)

add_test(
    NAME boundary_mesh_growth_patch_test
    COMMAND boundary_mesh_growth_patch_test
)

# 11
add_executable(
    boundary_mesh_growth_front_test
    unit/growth/growth_front_test.cpp
)
target_link_libraries(
    boundary_mesh_growth_front_test
    PRIVATE BoundaryMesh::BoundaryLayer
)
add_test(
    NAME boundary_mesh_growth_front_test
    COMMAND boundary_mesh_growth_front_test
)

# 12
add_executable(
    boundary_mesh_front_evaluator_test
    unit/growth/front_evaluator_test.cpp
)
target_link_libraries(
    boundary_mesh_front_evaluator_test
    PRIVATE BoundaryMesh::BoundaryLayer
)
add_test(
    NAME boundary_mesh_front_evaluator_test
    COMMAND boundary_mesh_front_evaluator_test
)

# 13
add_executable(
    boundary_mesh_growth_direction_test
    unit/growth/growth_direction_test.cpp
)
target_link_libraries(
    boundary_mesh_growth_direction_test
    PRIVATE BoundaryMesh::BoundaryLayer
)
add_test(
    NAME boundary_mesh_growth_direction_test
    COMMAND boundary_mesh_growth_direction_test
)

# 14
add_executable(
    boundary_mesh_symmetry_constraints_test
    unit/growth/symmetry_constraints_test.cpp
)
target_link_libraries(
    boundary_mesh_symmetry_constraints_test
    PRIVATE BoundaryMesh::BoundaryLayer
)
add_test(
    NAME boundary_mesh_symmetry_constraints_test
    COMMAND boundary_mesh_symmetry_constraints_test
)

# 15：阶段 03 的公开接口流水线集成测试
add_executable(
    boundary_mesh_growth_front_pipeline_test
    integration/growth_front_pipeline_test.cpp
)
target_link_libraries(
    boundary_mesh_growth_front_pipeline_test
    PRIVATE BoundaryMesh::BoundaryLayer
)
add_test(
    NAME boundary_mesh_growth_front_pipeline_test
    COMMAND boundary_mesh_growth_front_pipeline_test
)

# 16：三角形和四边形 equiangular skewness
add_executable(
    boundary_mesh_surface_face_skewness_test
    unit/surface/face_skewness_test.cpp
)
target_link_libraries(
    boundary_mesh_surface_face_skewness_test
    PRIVATE BoundaryMesh::Surface
)
add_test(
    NAME boundary_mesh_surface_face_skewness_test
    COMMAND boundary_mesh_surface_face_skewness_test
)

# 17: Prism/Hexa 体单元质量公共类型与顶点顺序
add_executable(
    boundary_mesh_volume_cell_types_test
    unit/quality/volume_cell_types_test.cpp
)
target_link_libraries(
    boundary_mesh_volume_cell_types_test
    PRIVATE BoundaryMesh::Quality
)
add_test(
    NAME boundary_mesh_volume_cell_types_test
    COMMAND boundary_mesh_volume_cell_types_test
)

# 18: Prism 候选单元质量评价
add_executable(
    boundary_mesh_prism_evaluator_test
    unit/quality/prism_evaluator_test.cpp
)
target_link_libraries(
    boundary_mesh_prism_evaluator_test
    PRIVATE BoundaryMesh::Quality
)
add_test(
    NAME boundary_mesh_prism_evaluator_test
    COMMAND boundary_mesh_prism_evaluator_test
)
