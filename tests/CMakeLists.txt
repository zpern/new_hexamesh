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

# 19: Hexa 候选单元质量评价
add_executable(
    boundary_mesh_hexa_evaluator_test
    unit/quality/hexa_evaluator_test.cpp
)
target_link_libraries(
    boundary_mesh_hexa_evaluator_test
    PRIVATE BoundaryMesh::Quality
)
add_test(
    NAME boundary_mesh_hexa_evaluator_test
    COMMAND boundary_mesh_hexa_evaluator_test
)

# 20: 体单元质量错误模型、尺度稳定性与确定性诊断
add_executable(
    boundary_mesh_volume_cell_error_test
    unit/quality/volume_cell_error_test.cpp
)
target_link_libraries(
    boundary_mesh_volume_cell_error_test
    PRIVATE BoundaryMesh::Quality
)
add_test(
    NAME boundary_mesh_volume_cell_error_test
    COMMAND boundary_mesh_volume_cell_error_test
)

# 21：固定子四面体有向体积分类与确定性诊断
add_executable(
    boundary_mesh_volume_cell_validity_test
    unit/quality/volume_cell_validity_test.cpp
)
target_link_libraries(
    boundary_mesh_volume_cell_validity_test
    PRIVATE BoundaryMesh::Quality
)
add_test(
    NAME boundary_mesh_volume_cell_validity_test
    COMMAND boundary_mesh_volume_cell_validity_test
)

# 22：阶段 05 预推出候选接受与停止决策模拟
add_executable(
    boundary_mesh_volume_cell_quality_pipeline_test
    integration/volume_cell_quality_pipeline_test.cpp
)
target_link_libraries(
    boundary_mesh_volume_cell_quality_pipeline_test
    PRIVATE BoundaryMesh::Quality
)
add_test(
    NAME boundary_mesh_volume_cell_quality_pipeline_test
    COMMAND boundary_mesh_volume_cell_quality_pipeline_test
)

# Release 性能程序不注册为 CTest，避免机器相关时间阈值。
add_executable(
    boundary_mesh_volume_cell_quality_benchmark
    ../benchmarks/volume_cell_quality_benchmark.cpp
)
target_link_libraries(
    boundary_mesh_volume_cell_quality_benchmark
    PRIVATE BoundaryMesh::Quality
)

# 23：逐源顶点生长参数验证和层步长计算
add_executable(
    boundary_mesh_growth_profile_test
    unit/growth/growth_profile_test.cpp
)
target_link_libraries(
    boundary_mesh_growth_profile_test
    PRIVATE BoundaryMesh::BoundaryLayer
)
add_test(
    NAME boundary_mesh_growth_profile_test
    COMMAND boundary_mesh_growth_profile_test
)

# 24：规则层生长公共状态、输出与错误类型
add_executable(
    boundary_mesh_regular_layer_growth_types_test
    unit/growth/regular_layer_growth_types_test.cpp
)
target_link_libraries(
    boundary_mesh_regular_layer_growth_types_test
    PRIVATE BoundaryMesh::BoundaryLayer
)
add_test(
    NAME boundary_mesh_regular_layer_growth_types_test
    COMMAND boundary_mesh_regular_layer_growth_types_test
)

# 25：单层规则前沿预推出、质量检查与紧凑映射
add_executable(
    boundary_mesh_regular_layer_stepper_test
    unit/growth/regular_layer_stepper_test.cpp
)
target_link_libraries(
    boundary_mesh_regular_layer_stepper_test
    PRIVATE
        BoundaryMesh::BoundaryLayer
        BoundaryMesh::Quality
)
add_test(
    NAME boundary_mesh_regular_layer_stepper_test
    COMMAND boundary_mesh_regular_layer_stepper_test
)

# 26：混合 Triangle/Quad 规则边界层完整生成流水线
add_executable(
    boundary_mesh_regular_layer_growth_pipeline_test
    integration/regular_layer_growth_pipeline_test.cpp
)
target_link_libraries(
    boundary_mesh_regular_layer_growth_pipeline_test
    PRIVATE BoundaryMesh::BoundaryLayer
)
add_test(
    NAME boundary_mesh_regular_layer_growth_pipeline_test
    COMMAND boundary_mesh_regular_layer_growth_pipeline_test
)

# 27：规则层程序级错误传播和输入事务保护
add_executable(
    boundary_mesh_regular_layer_growth_failure_test
    integration/regular_layer_growth_failure_test.cpp
)
target_link_libraries(
    boundary_mesh_regular_layer_growth_failure_test
    PRIVATE BoundaryMesh::BoundaryLayer
)
add_test(
    NAME boundary_mesh_regular_layer_growth_failure_test
    COMMAND boundary_mesh_regular_layer_growth_failure_test
)

# 28：闭区间 AABB 分离、重叠及面/边/点接触语义
add_executable(
    boundary_mesh_spatial_aabb_test
    unit/spatial/aabb_test.cpp
)
target_link_libraries(
    boundary_mesh_spatial_aabb_test
    PRIVATE BoundaryMesh::Spatial
)
add_test(
    NAME boundary_mesh_spatial_aabb_test
    COMMAND boundary_mesh_spatial_aabb_test
)

# 29：共享只读二叉 AABB 树的批量构建和确定性查询
add_executable(
    boundary_mesh_spatial_binary_aabb_tree_test
    unit/spatial/binary_aabb_tree_test.cpp
)
target_link_libraries(
    boundary_mesh_spatial_binary_aabb_tree_test
    PRIVATE BoundaryMesh::Spatial
)
add_test(
    NAME boundary_mesh_spatial_binary_aabb_tree_test
    COMMAND boundary_mesh_spatial_binary_aabb_tree_test
)

# 30：三角形接触分类及合法共享拓扑过滤
add_executable(
    boundary_mesh_spatial_triangle_contact_test
    unit/spatial/triangle_contact_test.cpp
)
target_link_libraries(
    boundary_mesh_spatial_triangle_contact_test
    PRIVATE BoundaryMesh::Spatial
)
add_test(
    NAME boundary_mesh_spatial_triangle_contact_test
    COMMAND boundary_mesh_spatial_triangle_contact_test
)
