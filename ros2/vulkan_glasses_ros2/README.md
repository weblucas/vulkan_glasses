# vulkan_glasses_ros2

ROS2 (ament_cmake) node wrapping the `vkg` Vulkan renderer. Subscribes to camera
odometry and publishes the rendered images.

It reuses the repo's `vkg::render` and `vkg::dataset` libraries directly via
`add_subdirectory` (see `CMakeLists.txt`), so a bare-checkout `colcon build`
works in one step — no separate install of the core libraries is required.

## Topics
| Direction | Topic | Type | Notes |
|-----------|-------|------|-------|
| sub | `odometry` | `nav_msgs/Odometry` | camera pose (throttled to `framerate`) |
| pub | `color_map` | `sensor_msgs/Image` | `bgr8` |
| pub | `depth_map` | `sensor_msgs/Image` | `32FC1`, metres (background = far) |
| pub | `semantic_map` | `sensor_msgs/Image` | `mono8` (scene item id) |
| pub | `camera_info` | `sensor_msgs/CameraInfo` | intrinsics used for the render |

## Parameters
See `config/params.yaml` (image size, `fx/fy/cx/cy`, `render_near/far`, scene
`model_folder`/`model_list_file`/`model_pose_file` or `mesh_obj_file`+`texture_file`,
`framerate`, `camera_frame_id`, optional `ortho`, optional body→camera extrinsic).
`shader_folder` defaults to the shaders installed in this package's `share/` dir.

## Build & run (ROS2 Humble)
```sh
# from the repo root (the colcon workspace)
colcon build --paths ros2/vulkan_glasses_ros2
source install/setup.bash
ros2 launch vulkan_glasses_ros2 render.launch.py config:=<your params.yaml>
```
Rendering needs a Vulkan driver (a GPU, or Mesa lavapipe for headless). See
`docker/` for a ready-made Humble image.
