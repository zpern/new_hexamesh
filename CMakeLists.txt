cmake_minimum_required(VERSION 3.20)

project(BoundaryMesh VERSION 1.0.0 LANGUAGES CXX)

message(STATUS "##############################")
MESSAGE(STATUS "CMake config files are writtern by zpern")
MESSAGE(STATUS "config bug report to: zpern@outlook.com")

set(TIGER_ROOT_DIR "${CMAKE_SOURCE_DIR}/")
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/CompileOptions.cmake")
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/Dependencies.cmake")


add_library(boundary_mesh_core STATIC src/mesh/surface_topology_builder.cpp)
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
    boundary_mesh_boundary_layer
    STATIC
        src/growth/growth_patch_builder.cpp
        src/growth/growth_front_builder.cpp
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
)

include(CTest)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
