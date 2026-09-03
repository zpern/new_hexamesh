if(NOT DEFINED PROJECT_ROOT)
    message(FATAL_ERROR "PROJECT_ROOT is required")
endif()

set(header_path
    "${PROJECT_ROOT}/include/boundary_mesh/transition/reserved_layer_transition.hpp")
set(source_path
    "${PROJECT_ROOT}/src/transition/reserved_layer_transition.cpp")
set(coordinator_header_path
    "${PROJECT_ROOT}/include/boundary_mesh/transition/transition_coordination.hpp")

file(READ "${header_path}" header_text)
file(READ "${source_path}" source_text)
file(READ "${coordinator_header_path}" coordinator_header_text)

foreach(forbidden_text IN ITEMS
        "using ReservedLayerTransitionError ="
        "// no multi-normal transition"
        "class TransitionLayerCoordinator")
    string(FIND "${header_text}${source_text}${coordinator_header_text}"
        "${forbidden_text}" position)
    if(NOT position EQUAL -1)
        message(FATAL_ERROR
            "Reserved transition still exposes the redundant entry: ${forbidden_text}")
    endif()
endforeach()

string(FIND "${header_text}" "const MultiNormalOptions &multi_normal_options"
    options_position)
if(options_position EQUAL -1)
    message(FATAL_ERROR
        "Reserved transition unified MultiNormalOptions entry is missing")
endif()
