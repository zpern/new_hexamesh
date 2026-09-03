cmake_minimum_required(VERSION 3.20)

project(BoundaryMesh VERSION 1.0.0 LANGUAGES C CXX)

option(
    BOUNDARY_MESH_ENABLE_CGNS_IO
    "Build CGNS IO, CLI, and CGNS tests"
    ON
)

message(STATUS "##############################")
MESSAGE(STATUS "CMake config files are writtern by zpern")
MESSAGE(STATUS "config bug report to: zpern@outlook.com")

set(TIGER_ROOT_DIR "${CMAKE_SOURCE_DIR}/")
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/CompileOptions.cmake")
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/Dependencies.cmake")


add_library(
    boundary_mesh_core
    STATIC
        src/mesh/surface_topology_builder.cpp
        src/mesh/surface_orientation.cpp
)
add_library(BoundaryMesh::Core ALIAS boundary_mesh_core)

target_compile_features(boundary_mesh_core PUBLIC cxx_std_17)

target_include_directories(
    boundary_mesh_core
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(
    boundary_mesh_core
    PUBLIC
        Eigen3::Eigen
        BoundaryMesh::CompileOptions
)

add_library(
    boundary_mesh_surface
    STATIC
        src/surface/face_evaluation.cpp
        src/surface/face_skewness.cpp
)
add_library(
    BoundaryMesh::Surface
    ALIAS boundary_mesh_surface
)

target_compile_features(
    boundary_mesh_surface
    PUBLIC
        cxx_std_17
)

target_include_directories(
    boundary_mesh_surface
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(
    boundary_mesh_surface
    PUBLIC
        BoundaryMesh::Core
)

add_library(
    boundary_mesh_quality
    STATIC
        src/quality/volume_cell_evaluator_common.cpp
        src/quality/prism_evaluator.cpp
        src/quality/hexa_evaluator.cpp
)
add_library(BoundaryMesh::Quality ALIAS boundary_mesh_quality)

target_compile_features(boundary_mesh_quality PUBLIC cxx_std_17)

target_include_directories(
    boundary_mesh_quality
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(
    boundary_mesh_quality
    PUBLIC
        BoundaryMesh::Core
        BoundaryMesh::Surface
)

add_library(
    boundary_mesh_spatial
    STATIC
        src/spatial/aabb.cpp
        src/spatial/binary_aabb_tree.cpp
        src/spatial/triangle_surface_index.cpp
        src/spatial/triangle_contact.cpp
        src/spatial/collision_index.cpp
)
add_library(BoundaryMesh::Spatial ALIAS boundary_mesh_spatial)

target_compile_features(boundary_mesh_spatial PUBLIC cxx_std_17)

target_include_directories(
    boundary_mesh_spatial
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        "${TIGER_DEPENDENCIES_DIR}/geom"
)

target_link_libraries(
    boundary_mesh_spatial
    PUBLIC
        BoundaryMesh::Core
    PRIVATE
        tiger_geom
)

add_library(
    boundary_mesh_boundary_layer
    STATIC
        src/growth/growth_patch_builder.cpp
        src/growth/growth_front_builder.cpp
        src/growth/front_adjacency.cpp
        src/growth/isotropic_stop_evaluator.cpp
        src/growth/front_evaluator.cpp
        src/growth/growth_direction.cpp
        src/growth/growth_field_smoother.cpp
        src/growth/skewness_direction_refiner.cpp
        src/growth/sliding_constraint_builder.cpp
        src/growth/sliding_surface_builder.cpp
        src/growth/growth_profile_builder.cpp
        src/growth/regular_layer_stepper.cpp
        src/growth/regular_layer_generator.cpp
        src/growth/exposed_boundary.cpp
        src/growth/farfield_boundary_builder.cpp
        src/growth/layer_collision_checker.cpp
        src/growth/face_layer_constraint.cpp
        src/growth/termination_propagator.cpp
)
add_library(
    BoundaryMesh::BoundaryLayer
    ALIAS boundary_mesh_boundary_layer
)
target_compile_features(
    boundary_mesh_boundary_layer
    PUBLIC
        cxx_std_17
)
target_include_directories(
    boundary_mesh_boundary_layer
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(
    boundary_mesh_boundary_layer
    PUBLIC
        BoundaryMesh::Core
        BoundaryMesh::Surface
        BoundaryMesh::Quality
        BoundaryMesh::Spatial
)

add_library(
    boundary_mesh_multi_normal
    STATIC
        src/multi_normal/incident_face_fan.cpp
        src/multi_normal/blmesh_splitter.cpp
        src/multi_normal/blmesh_geometry.cpp
        src/multi_normal/blmesh_intersection_checker.cpp
        src/multi_normal/blmesh_topology_stitching.cpp
        third/blmesh_mnormal/src/combineoptimizer.cpp
        third/blmesh_mnormal/src/mergeoptimizer.cpp
        third/blmesh_mnormal/src/meshevaluation.cpp
        third/blmesh_mnormal/src/TopologyOptimizer.cpp
        third/blmesh_mnormal/src/VirtualSphereMeshStrategy.cpp
        third/blmesh_mnormal/src/BoundaryTriangulation.cpp
        third/blmesh_mnormal/src/VirtualSphereMeshHasher.cpp
        third/blmesh_mnormal/src/MeshSplitter.cpp
        third/blmesh_mnormal/src/pointoptimizer.cpp
        third/blmesh_mnormal/src/VirtualSphereMeshGenerator.cpp
        third/blmesh_mnormal/src/VirtualSphereMesh.cpp
        third/blmesh_mnormal/src/BLVector.cpp
        third/blmesh_mnormal/src/sphericalCap.cpp
        third/blmesh_mnormal/src/complexnode.cpp
        third/blmesh_mnormal/src/geometryfunction.cpp
        third/blmesh_mnormal/src/mostnormaloptimizer.cpp
        src/multi_normal/multi_normal_split_planner.cpp
        src/multi_normal/multi_normal_topology_builder.cpp
        src/multi_normal/multi_normal_quad_triangulator.cpp
        src/multi_normal/multi_normal_transition_builder.cpp
        src/multi_normal/multi_normal_intersection_resolver.cpp
        src/multi_normal/multi_normal_transition_generator.cpp
        src/multi_normal/multi_normal_mesh_merge.cpp
)
add_library(BoundaryMesh::MultiNormal ALIAS boundary_mesh_multi_normal)
target_compile_features(boundary_mesh_multi_normal PUBLIC cxx_std_17)
target_include_directories(
    boundary_mesh_multi_normal
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${PROJECT_SOURCE_DIR}/third/blmesh_mnormal/include
        ${PROJECT_SOURCE_DIR}/third/blmesh_mnormal/src
        ${TIGER_DEPENDENCIES_DIR}/geom
)
target_link_libraries(
    boundary_mesh_multi_normal
    PUBLIC
        BoundaryMesh::BoundaryLayer
        BoundaryMesh::Core
        BoundaryMesh::Surface
        BoundaryMesh::Quality
        BoundaryMesh::Spatial
)

add_library(
    boundary_mesh_transition
    STATIC
        src/transition/reserved_layer_growth.cpp
        src/transition/quad_diagonal.cpp
        src/transition/triangle_transition_template.cpp
        src/transition/quad_transition_template.cpp
        src/transition/layer_quad_diagonal_table.cpp
        src/transition/incremental_transition_templates.cpp
        src/transition/quad_high_neighbor_selector.cpp
        src/transition/corner_suppression.cpp
        src/transition/transition_boundary_checker.cpp
        src/transition/reserved_layer_transition.cpp
        src/transition/boundary_layer_generator.cpp
)
add_library(BoundaryMesh::Transition ALIAS boundary_mesh_transition)
target_compile_features(boundary_mesh_transition PUBLIC cxx_std_17)
target_include_directories(
    boundary_mesh_transition
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(
    boundary_mesh_transition
    PUBLIC
        BoundaryMesh::MultiNormal
        BoundaryMesh::BoundaryLayer
        BoundaryMesh::Surface
        BoundaryMesh::Core
)

add_library(
    boundary_mesh_io
    STATIC
        src/io/legacy_vtk_writer.cpp
)
add_library(BoundaryMesh::IO ALIAS boundary_mesh_io)
target_compile_features(boundary_mesh_io PUBLIC cxx_std_17)
target_include_directories(
    boundary_mesh_io
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        "${PROJECT_SOURCE_DIR}/src"
)
target_link_libraries(
    boundary_mesh_io
    PUBLIC BoundaryMesh::Core
)
target_link_libraries(boundary_mesh_multi_normal PUBLIC BoundaryMesh::IO)

if(BOUNDARY_MESH_ENABLE_CGNS_IO)
    target_sources(
        boundary_mesh_io
        PRIVATE
            src/io/boundary_condition_map.cpp
            src/io/cgns_surface_reader.cpp
    )
    target_link_libraries(
        boundary_mesh_io
        PRIVATE BoundaryMesh::CGNS
    )

    add_library(
        boundary_mesh_cli_support
        STATIC
            src/cli/boundary_mesh_command.cpp
    )
    add_library(BoundaryMesh::CLI ALIAS boundary_mesh_cli_support)
    target_compile_features(boundary_mesh_cli_support PUBLIC cxx_std_17)
    target_include_directories(
        boundary_mesh_cli_support
        PUBLIC "${PROJECT_SOURCE_DIR}/src"
    )
    target_link_libraries(
        boundary_mesh_cli_support
        PUBLIC
            BoundaryMesh::IO
            BoundaryMesh::BoundaryLayer
            BoundaryMesh::Transition
    )

    add_executable(
        boundary_mesh_cli
        apps/boundary_mesh_cli.cpp
    )
    target_link_libraries(
        boundary_mesh_cli
        PRIVATE BoundaryMesh::CLI
    )

    add_executable(
        boundary_mesh_cgns_pipeline_benchmark
        benchmarks/cgns_pipeline_benchmark.cpp
    )
    target_link_libraries(
        boundary_mesh_cgns_pipeline_benchmark
        PRIVATE
            BoundaryMesh::IO
            BoundaryMesh::BoundaryLayer
    )
    if(WIN32)
        target_link_libraries(
            boundary_mesh_cgns_pipeline_benchmark
            PRIVATE Psapi
        )
    endif()

endif()

include(CTest)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
