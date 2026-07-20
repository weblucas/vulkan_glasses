# Unit tests. Enabled from the top-level build with -DBUILD_TESTS=ON.
# The unit tests below are GPU-independent (no Vulkan device required) and only
# depend on glm, which the library already requires.

add_executable(test_pose_math test_pose_math.cc)
target_include_directories(test_pose_math PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
add_test(NAME pose_math COMMAND test_pose_math)

add_executable(test_csv_parse test_csv_parse.cc)
target_include_directories(test_csv_parse PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/lib/include)
add_test(NAME csv_parse COMMAND test_csv_parse)

# Integration smoke test: if the CSV renderer app was built
# (-DBUILD_WITH_CSV_PROCESSOR=ON), render one frame of the bundled example scene
# headless and then read the produced .h5 back, confirming the full
# render -> saveHdf5 -> read-back path (not just a dry run). Requires a working
# Vulkan device/driver (a software driver such as Mesa lavapipe is sufficient).
#
# Structured as CTest fixtures so ordering is deterministic:
#   clean (remove old output) -> render (produce .h5) -> verify (read + check).
if(TARGET ${PROJECT_NAME}_csv_renderer)
    set(SMOKE_OUT ${CMAKE_BINARY_DIR}/test_output)

    # The renderer refuses to start if the output folder already exists, so wipe
    # it first.
    add_test(NAME render_smoke_clean
             COMMAND ${CMAKE_COMMAND} -E rm -rf ${SMOKE_OUT})
    set_tests_properties(render_smoke_clean PROPERTIES FIXTURES_SETUP RENDER_OUT)

    # step_skip=1000 renders a single frame (only file index 0 passes the filter),
    # keeping the test fast while still exercising the real write path.
    add_test(
        NAME example_render_smoke
        COMMAND $<TARGET_FILE:${PROJECT_NAME}_csv_renderer>
                --output_folder_path=${SMOKE_OUT}/
                --fx=571.63 --fy=571.63 --cx=366.23 --cy=243.592
                --far=1000 --near=0.1 --output_h=480 --output_w=752
                --model_folder=${CMAKE_SOURCE_DIR}/example/models
                --model_list_file=${CMAKE_SOURCE_DIR}/example/scene_irchel140821_and_four_capsules/model_def_list.txt
                --model_pose_file=${CMAKE_SOURCE_DIR}/example/scene_irchel140821_and_four_capsules/model_poses_list.txt
                --ortho=false
                --pose_file=${CMAKE_SOURCE_DIR}/example/image_poses.txt
                --shader_folder=${CMAKE_BINARY_DIR}
                --step_skip=1000
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
    set_tests_properties(example_render_smoke PROPERTIES
        FIXTURES_SETUP RENDER_OUT
        DEPENDS render_smoke_clean)

    # Read the produced .h5 back and assert it is well-formed with sane depth.
    add_executable(verify_h5_output verify_h5_output.cc)
    target_link_libraries(verify_h5_output ${PROJECT_NAME}_h5_dataset)
    add_test(NAME verify_render_output
             COMMAND verify_h5_output ${SMOKE_OUT})
    set_tests_properties(verify_render_output PROPERTIES FIXTURES_REQUIRED RENDER_OUT)
endif()
