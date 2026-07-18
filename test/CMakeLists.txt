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
# (-DBUILD_WITH_CSV_PROCESSOR=ON), render the bundled example scene headless
# (dry run, no output written) to confirm the full pipeline works end to end.
# Requires a working Vulkan device/driver (a software driver such as Mesa
# lavapipe is sufficient).
if(TARGET ${PROJECT_NAME}_csv_renderer)
    add_test(
        NAME example_render_smoke
        COMMAND $<TARGET_FILE:${PROJECT_NAME}_csv_renderer>
                --output_folder_path=${CMAKE_BINARY_DIR}/test_output/
                --fx=571.63 --fy=571.63 --cx=366.23 --cy=243.592
                --far=1000 --near=0.1 --output_h=480 --output_w=752
                --model_folder=${CMAKE_SOURCE_DIR}/example/models
                --model_list_file=${CMAKE_SOURCE_DIR}/example/scene_irchel140821_and_four_capsules/model_def_list.txt
                --model_pose_file=${CMAKE_SOURCE_DIR}/example/scene_irchel140821_and_four_capsules/model_poses_list.txt
                --ortho=false
                --pose_file=${CMAKE_SOURCE_DIR}/example/image_poses.txt
                --shader_folder=${CMAKE_BINARY_DIR}
                --step_skip=1
                --dry_run=true
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
endif()
