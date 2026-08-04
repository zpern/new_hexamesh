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