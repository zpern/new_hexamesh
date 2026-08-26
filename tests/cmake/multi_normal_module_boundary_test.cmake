if(NOT DEFINED PROJECT_ROOT)
    message(FATAL_ERROR "PROJECT_ROOT is required")
endif()

set(required_paths
    "include/boundary_mesh/multi_normal/multi_normal_types.hpp"
    "include/boundary_mesh/multi_normal/multi_normal_transition_generator.hpp"
    "include/boundary_mesh/multi_normal/incident_face_fan.hpp"
    "include/boundary_mesh/multi_normal/detail/blmesh_geometry.hpp"
    "src/multi_normal/multi_normal_transition_generator.cpp"
    "src/multi_normal/incident_face_fan.cpp"
    "src/multi_normal/blmesh_geometry.cpp"
    "include/boundary_mesh/transition/boundary_layer_generator.hpp"
    "src/transition/boundary_layer_generator.cpp")

foreach(relative_path IN LISTS required_paths)
    if(NOT EXISTS "${PROJECT_ROOT}/${relative_path}")
        message(FATAL_ERROR
            "Missing standalone multi-normal path: ${relative_path}")
    endif()
endforeach()

set(forbidden_paths
    "include/boundary_mesh/growth/multi_normal_types.hpp"
    "include/boundary_mesh/growth/incident_face_fan.hpp"
    "include/boundary_mesh/growth/detail/blmesh_geometry.hpp"
    "src/growth/multi_normal_transition_generator.cpp"
    "src/growth/incident_face_fan.cpp"
    "src/growth/blmesh_geometry.cpp"
    "include/boundary_mesh/growth/boundary_layer_generator.hpp"
    "src/growth/boundary_layer_generator.cpp")

foreach(relative_path IN LISTS forbidden_paths)
    if(EXISTS "${PROJECT_ROOT}/${relative_path}")
        message(FATAL_ERROR
            "Growth still owns multi-normal path: ${relative_path}")
    endif()
endforeach()

file(READ "${PROJECT_ROOT}/CMakeLists.txt" root_cmake)
foreach(required_text IN ITEMS
        "boundary_mesh_multi_normal"
        "BoundaryMesh::MultiNormal")
    string(FIND "${root_cmake}" "${required_text}" position)
    if(position EQUAL -1)
        message(FATAL_ERROR "Root CMake is missing: ${required_text}")
    endif()
endforeach()
