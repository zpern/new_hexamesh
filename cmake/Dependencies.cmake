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

# CGNS/HDF5：父工程可直接提供统一适配 target；独立构建时使用固定子模块。
if(BOUNDARY_MESH_ENABLE_CGNS_IO AND NOT TARGET BoundaryMesh::CGNS)
    if(TARGET CGNS::cgns-static)
        add_library(boundary_mesh_cgns INTERFACE)
        target_link_libraries(boundary_mesh_cgns INTERFACE CGNS::cgns-static)
        add_library(BoundaryMesh::CGNS ALIAS boundary_mesh_cgns)
    elseif(TARGET CGNS::cgns-shared)
        add_library(boundary_mesh_cgns INTERFACE)
        target_link_libraries(boundary_mesh_cgns INTERFACE CGNS::cgns-shared)
        add_library(BoundaryMesh::CGNS ALIAS boundary_mesh_cgns)
    elseif(EXISTS "${TIGER_DEPENDENCIES_DIR}/hdf5/CMakeLists.txt"
           AND EXISTS "${TIGER_DEPENDENCIES_DIR}/cgns/CMakeLists.txt")
        set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build shared libraries" FORCE)
        set(BUILD_STATIC_LIBS ON CACHE BOOL "Build static libraries" FORCE)
        set(HDF5_BUILD_TOOLS OFF CACHE BOOL "Build HDF5 tools" FORCE)
        set(HDF5_BUILD_EXAMPLES OFF CACHE BOOL "Build HDF5 examples" FORCE)
        set(HDF5_BUILD_FORTRAN OFF CACHE BOOL "Build HDF5 Fortran" FORCE)
        set(HDF5_BUILD_CPP_LIB OFF CACHE BOOL "Build HDF5 C++" FORCE)
        set(HDF5_BUILD_JAVA OFF CACHE BOOL "Build HDF5 Java" FORCE)
        set(HDF5_BUILD_HL_LIB OFF CACHE BOOL "Build HDF5 high-level library" FORCE)
        set(HDF5_ENABLE_Z_LIB_SUPPORT OFF CACHE BOOL "Enable HDF5 zlib" FORCE)
        set(HDF5_ENABLE_SZIP_SUPPORT OFF CACHE BOOL "Enable HDF5 szip" FORCE)
        set(HDF5_BUILD_TESTING OFF CACHE BOOL "Build HDF5 tests" FORCE)

        set(_boundary_mesh_hdf5_binary
            "${CMAKE_BINARY_DIR}/boundary_mesh_hdf5")
        add_subdirectory(
            "${TIGER_DEPENDENCIES_DIR}/hdf5"
            "${_boundary_mesh_hdf5_binary}"
            EXCLUDE_FROM_ALL
        )

        # HDF5 的 build-tree config 以 `hdf5` 判断目标是否已由父工程创建。
        # 官方子目录实际创建 `hdf5-static`，补上该兼容别名可避免重复导入。
        if(TARGET hdf5-static AND NOT TARGET hdf5)
            add_library(hdf5 ALIAS hdf5-static)
        endif()

        set(hdf5_DIR "${_boundary_mesh_hdf5_binary}" CACHE PATH
            "HDF5 build-tree package" FORCE)
        set(HDF5_DIR "${_boundary_mesh_hdf5_binary}" CACHE PATH
            "HDF5 build-tree package" FORCE)
        # CGNS 4.5.2 使用大写变量检查版本；HDF5 的嵌套 config
        # 在该调用方式下只填充小写包版本变量。
        set(HDF5_VERSION "1.14.6" CACHE STRING
            "HDF5 version exposed to CGNS" FORCE)
        set(CGNS_BUILD_SHARED OFF CACHE BOOL "Build shared CGNS" FORCE)
        set(CGNS_ENABLE_TESTS OFF CACHE BOOL "Build CGNS tests" FORCE)
        set(CGNS_BUILD_TESTING OFF CACHE BOOL "Build CGNS testing" FORCE)
        set(CGNS_ENABLE_FORTRAN OFF CACHE BOOL "Enable CGNS Fortran" FORCE)
        set(CGNS_ENABLE_HDF5 ON CACHE BOOL "Enable CGNS HDF5" FORCE)
        set(CGNS_BUILD_CGNSTOOLS OFF CACHE BOOL "Build CGNS tools" FORCE)
        add_subdirectory(
            "${TIGER_DEPENDENCIES_DIR}/cgns"
            "${CMAKE_BINARY_DIR}/boundary_mesh_cgns"
            EXCLUDE_FROM_ALL
        )

        add_library(boundary_mesh_cgns INTERFACE)
        target_link_libraries(boundary_mesh_cgns INTERFACE CGNS::cgns-static)
        add_library(BoundaryMesh::CGNS ALIAS boundary_mesh_cgns)
    else()
        message(FATAL_ERROR
            "CGNS is unavailable and third/cgns or third/hdf5 is missing")
    endif()
endif()

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
