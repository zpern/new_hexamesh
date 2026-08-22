include_guard(GLOBAL)

# Set the root directory of TiGER
if(EXISTS "${TIGER_ROOT_DIR}/extern")
    set(TIGER_DEPENDENCIES_DIR "${TIGER_ROOT_DIR}/extern")
elseif(EXISTS "${TIGER_ROOT_DIR}/third")
    set(TIGER_DEPENDENCIES_DIR "${TIGER_ROOT_DIR}/third")
else()
    message(FATAL_ERROR "Neither '${TIGER_ROOT_DIR}/extern' nor '${TIGER_ROOT_DIR}/third' exists")
endif()

message(STATUS "TiGER dependency directory: ${TIGER_DEPENDENCIES_DIR}")

# Eigen
add_library(boundary_mesh_eigen INTERFACE)
add_library(Eigen3::Eigen ALIAS boundary_mesh_eigen
)

target_include_directories(
    boundary_mesh_eigen
    SYSTEM INTERFACE
        "${TIGER_DEPENDENCIES_DIR}/eigen"
)

# tiger_geom：父工程已经提供 target 时直接复用，否则使用固定子模块。
if(NOT TARGET tiger_geom)
    if(EXISTS "${TIGER_DEPENDENCIES_DIR}/geom/CMakeLists.txt")
        add_subdirectory(
            "${TIGER_DEPENDENCIES_DIR}/geom"
            "${CMAKE_BINARY_DIR}/boundary_mesh_geom"
        )
    else()
        message(FATAL_ERROR
            "tiger_geom is unavailable and third/geom is missing")
    endif()
endif()
