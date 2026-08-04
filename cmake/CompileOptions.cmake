include_guard(GLOBAL)

add_library(CompileOptions INTERFACE)
add_library(BoundaryMesh::CompileOptions ALIAS CompileOptions
)

# Set the C++ standard to C++17
target_compile_features(CompileOptions INTERFACE cxx_std_17)
set_target_properties(CompileOptions PROPERTIES INTERFACE_CXX_EXTENSIONS OFF)

# Set compile options for different compilers
# Ban warning for MSVC, and disable all warnings for other compilers
target_compile_options(
    CompileOptions
    INTERFACE
        $<$<CXX_COMPILER_ID:MSVC>:/utf-8>
        $<$<CXX_COMPILER_ID:MSVC>:/W0>
        $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-w>
)

# Set position independent code for the interface library
set_target_properties(
    CompileOptions
    PROPERTIES INTERFACE_POSITION_INDEPENDENT_CODE ON
)

# Function to set large stack size for a target
function(set_large_stack target_name)
    if(MSVC)
        target_link_options(${target_name} PRIVATE /STACK:200000000)
    elseif(MINGW)
        target_link_options(${target_name} PRIVATE -Wl,--stack,200000000)
    endif()
endfunction()