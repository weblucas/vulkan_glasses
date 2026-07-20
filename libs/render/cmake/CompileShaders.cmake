# vkg_compile_shaders(<target> <out_dir> <shader_src_dir>)
#
# Compiles vkg_shader.vert / vkg_shader.frag from <shader_src_dir> to SPIR-V in
# <out_dir> and defines an ALL custom target <target> that produces them. The
# two .spv paths are returned to the caller in ${<target>_SPV_FILES}.
#
# Shaders are runtime assets (not part of the static library); each consumer is
# responsible for installing / locating the resulting .spv. This helper is shared
# by the render library and (later) the ROS2 package so shader compilation is
# defined in exactly one place.
function(vkg_compile_shaders target out_dir shader_src_dir)
    find_program(GLSLANG_VALIDATOR glslangValidator)
    if(NOT GLSLANG_VALIDATOR)
        message(FATAL_ERROR
            "glslangValidator not found. Install the Vulkan SDK / glslang-tools.")
    endif()

    set(_vert_spv "${out_dir}/vkg_shader.vert.spv")
    set(_frag_spv "${out_dir}/vkg_shader.frag.spv")

    add_custom_command(
        OUTPUT ${_vert_spv}
        COMMAND ${CMAKE_COMMAND} -E make_directory ${out_dir}
        COMMAND ${GLSLANG_VALIDATOR} -V ${shader_src_dir}/vkg_shader.vert -o ${_vert_spv}
        DEPENDS ${shader_src_dir}/vkg_shader.vert
        COMMENT "Compiling vkg_shader.vert -> SPIR-V")
    add_custom_command(
        OUTPUT ${_frag_spv}
        COMMAND ${CMAKE_COMMAND} -E make_directory ${out_dir}
        COMMAND ${GLSLANG_VALIDATOR} -V ${shader_src_dir}/vkg_shader.frag -o ${_frag_spv}
        DEPENDS ${shader_src_dir}/vkg_shader.frag
        COMMENT "Compiling vkg_shader.frag -> SPIR-V")

    add_custom_target(${target} ALL DEPENDS ${_vert_spv} ${_frag_spv})
    set(${target}_SPV_FILES ${_vert_spv} ${_frag_spv} PARENT_SCOPE)
endfunction()
