# Unit tests. Enabled from the top-level build with -DBUILD_TESTS=ON.
# The unit tests below are GPU-independent (no Vulkan device required) and only
# depend on glm / header-only utilities.

add_executable(test_pose_math test_pose_math.cc)
# vkg/pose_utils.h is a header-only helper shipped with the render library.
target_include_directories(test_pose_math PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/libs/render/include)
add_test(NAME pose_math COMMAND test_pose_math)

add_executable(test_csv_parse test_csv_parse.cc)
# vkg/string_utils.h is a header-only helper shipped with the render library.
target_include_directories(test_csv_parse PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/libs/render/include)
add_test(NAME csv_parse COMMAND test_csv_parse)

# Integration smoke test: when the CSV renderer app is built, render one frame of
# the bundled example scene headless and read the produced .h5 back, exercising
# the full render -> saveHdf5 -> read-back path. Requires a working Vulkan
# device/driver (a software driver such as Mesa lavapipe is sufficient).
#
# CTest fixtures make ordering deterministic:
#   clean (remove old output) -> render (produce .h5) -> verify (read + check).
if(TARGET vkg_csv_renderer)
    set(SMOKE_OUT ${CMAKE_BINARY_DIR}/test_output)

    add_test(NAME render_smoke_clean
             COMMAND ${CMAKE_COMMAND} -E rm -rf ${SMOKE_OUT})
    set_tests_properties(render_smoke_clean PROPERTIES FIXTURES_SETUP RENDER_OUT)

    # step_skip=1000 renders a single frame (only file index 0 passes the filter),
    # keeping the test fast while still exercising the real write path.
    add_test(
        NAME example_render_smoke
        COMMAND $<TARGET_FILE:vkg_csv_renderer>
                --output_folder_path=${SMOKE_OUT}/
                --fx=571.63 --fy=571.63 --cx=366.23 --cy=243.592
                --far=1000 --near=0.1 --output_h=480 --output_w=752
                --model_folder=${CMAKE_SOURCE_DIR}/example/models
                --model_list_file=${CMAKE_SOURCE_DIR}/example/scene_irchel140821_and_four_capsules/model_def_list.txt
                --model_pose_file=${CMAKE_SOURCE_DIR}/example/scene_irchel140821_and_four_capsules/model_poses_list.txt
                --ortho=false
                --pose_file=${CMAKE_SOURCE_DIR}/example/image_poses.txt
                --shader_folder=${VKG_SHADER_SPV_DIR}
                --step_skip=1000
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
    set_tests_properties(example_render_smoke PROPERTIES
        FIXTURES_SETUP RENDER_OUT
        DEPENDS render_smoke_clean)

    # Read the produced .h5 back and assert it is well-formed with sane depth.
    add_executable(verify_h5_output verify_h5_output.cc)
    target_link_libraries(verify_h5_output PRIVATE vkg::dataset)
    add_test(NAME verify_render_output
             COMMAND verify_h5_output ${SMOKE_OUT})
    set_tests_properties(verify_render_output PROPERTIES FIXTURES_REQUIRED RENDER_OUT)
endif()
