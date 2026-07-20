# vulkan_glasses
Fast Renderer for models with baked illumination or directly from photogrammetry
For ROS (Robot Operating System) support, which is a flexible framework for writing robot software, use https://github.com/VIS4ROB-lab/vulkan_glasses_for_robots.
It is the same library but integrated with ros simulation. The scene definition works in both softwares.

## Dependencies

The core library requires:

- CMake >= 3.21 and a C++17 compiler
- Vulkan SDK / development files (`glslangValidator` is used to compile the shaders)
- Eigen3
- OpenCV (`core`, `highgui`, `imgproc`)
- glog, gflags
- glm

The optional CSV renderer application (`BUILD_CSV_RENDERER`) and the `.h5`
image viewer (`BUILD_DATASET_VIEWER`) additionally require HDF5.

On Ubuntu these can be installed with:

```sh
# core library
sudo apt-get install -y \
    libvulkan-dev glslang-tools \
    libeigen3-dev libopencv-dev \
    libgoogle-glog-dev libgflags-dev libglm-dev

# extra, only for the CSV renderer and the .h5 viewer applications
sudo apt-get install -y libhdf5-dev
```

Running the renderer needs a working Vulkan driver. A physical GPU works, and a
software driver (e.g. Mesa lavapipe, `mesa-vulkan-drivers`) is enough for
headless machines.

## Building

Out-of-source build into `build/`:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

The repo is organized so each piece builds independently:

```
libs/render/    # vkg::render  — the Vulkan renderer library (needs Vulkan)
libs/dataset/   # vkg::dataset — the .h5 dataset I/O library (no Vulkan)
apps/csv_renderer/       # standalone CSV renderer  -> vkg_csv_renderer
apps/vkg_dataset_viewer/ # standalone .h5 viewer    -> vkg_dataset_viewer
```

Build options (orthogonal — a library builds on its own; an app pulls in the
libraries it needs):

| Option                 | Default | Description                                      |
|------------------------|---------|--------------------------------------------------|
| `BUILD_RENDER_LIB`     | `ON`    | Build the render library `vkg::render`           |
| `BUILD_DATASET_LIB`    | `ON`    | Build the dataset library `vkg::dataset`         |
| `BUILD_CSV_RENDERER`   | `OFF`   | Build the `vkg_csv_renderer` app (needs both libs)|
| `BUILD_DATASET_VIEWER` | `OFF`   | Build the `vkg_dataset_viewer` app (needs dataset)|
| `BUILD_TESTS`          | `OFF`   | Build the unit tests in `test/`                  |

Examples:

```sh
# just the libraries (installable / find_package-able), the default:
cmake -S . -B build && cmake --build build -j$(nproc)

# the CSV renderer application (auto-enables both libraries):
cmake -S . -B build -DBUILD_CSV_RENDERER=ON && cmake --build build -j$(nproc)

# the dataset library + viewer only, no Vulkan needed:
cmake -S . -B build -DBUILD_RENDER_LIB=OFF -DBUILD_DATASET_VIEWER=ON \
      && cmake --build build -j$(nproc)
```

## Installing / consuming the libraries

Installing produces `find_package`-able CMake packages:

```sh
cmake --install build --prefix /path/to/prefix
```

```
<prefix>/lib/libvkg_render.a, libvkg_dataset.a
<prefix>/lib/cmake/vkg_render/…, vkg_dataset/…    # find_package config
<prefix>/include/vkg/*.h
<prefix>/share/vkg/shaders/*.spv
```

Downstream projects then:

```cmake
find_package(vkg_render REQUIRED)   # or vkg_dataset
target_link_libraries(my_target PRIVATE vkg::render vkg::dataset)
```

(The default prefix for a standalone build is `<repo>/bin`.)

### Running the apps (self-contained)

Build both apps and install into `install/`:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_CSV_RENDERER=ON -DBUILD_DATASET_VIEWER=ON
cmake --build build -j$(nproc)
cmake --install build --prefix "$(pwd)/install"
```

```
install/
├── vkg_csv_renderer
├── vkg_dataset_viewer
├── run_csv_renderer.sh            # launcher: sets --shader_folder automatically
├── run_vkg_dataset_viewer.sh
└── shaders/*.spv
```

Run via the launcher scripts from any directory — they resolve the shader folder
relative to their own location:

```sh
install/run_csv_renderer.sh --flagfile=example/vk_glasses_csv_flags.txt
install/run_vkg_dataset_viewer.sh --folder example/output_temp
```

The install tree is self-contained in project artifacts (binaries, shaders,
launchers). It still relies on the system-provided runtime libraries (OpenCV, HDF5,
glog, gflags) and a working Vulkan driver being installed on the machine.

## ROS2 node

`ros2/vulkan_glasses_ros2/` is an ament_cmake package that wraps `vkg::render`
and publishes rendered color/depth/semantic images + `CameraInfo` from camera
odometry. It reuses the libraries in-tree (`add_subdirectory`), so a bare-checkout
`colcon build` works in one step:

```sh
colcon build --paths ros2/vulkan_glasses_ros2
source install/setup.bash
ros2 launch vulkan_glasses_ros2 render.launch.py config:=<your params.yaml>
```

See `ros2/vulkan_glasses_ros2/README.md` for topics and parameters. ROS2 Humble
is required — use the Docker image below if it isn't installed locally.

## Docker

```sh
# ROS2 Humble image (builds the colcon overlay):
docker build -f docker/ros2.Dockerfile -t vkg-ros2 .
# non-ROS image (libraries + standalone apps):
docker build -f docker/standalone.Dockerfile -t vkg .
# or via compose:
docker compose -f docker/docker-compose.yml build
```

Both images fall back to Mesa lavapipe for headless rendering when no GPU is
passed through. A VS Code dev container (`.devcontainer/`, based on the Humble
image) provides a ready-to-use environment for all of the above. `./build.sh
{core|ros2|all}` is a single entrypoint that drives both build systems.

## Testing

Unit tests live in `test/` and are built when `-DBUILD_TESTS=ON` is passed. The
`pose_math` and `csv_parse` tests are GPU-independent. If the CSV renderer app
is also built, an `example_render_smoke` test renders the bundled example scene
headless (requires a Vulkan driver):

```sh
cmake -S . -B build -DBUILD_TESTS=ON -DBUILD_CSV_RENDERER=ON
cmake --build build -j$(nproc)
ctest --test-dir build --output-on-failure
```

## Usage

```sh
vulkan_glasses/bin$ ./vkg_csv_renderer --flagfile=../example/vk_glasses_csv_flags.txt
```

## Command-Line Flags for `vk_glasses_csv`

This section describes the available flags for configuring the `vk_glasses_csv` application. You can provide these flags in a text file (e.g., `vk_glasses_csv_flags.txt`) or as command-line arguments.

| Flag                      | Description                                                                                      | Example Value                                               |
|---------------------------|--------------------------------------------------------------------------------------------------|-------------------------------------------------------------|
| `--output_folder_path`    | Path to the output directory where results will be saved.                                        | `../example/output_temp/`                                   |
| `--fx`                    | Camera focal length in x (pixels).                                                               | `571.63`                                                    |
| `--fy`                    | Camera focal length in y (pixels).                                                               | `571.63`                                                    |
| `--cx`                    | Camera principal point x-coordinate (pixels).                                                    | `366.23`                                                    |
| `--cy`                    | Camera principal point y-coordinate (pixels).                                                    | `243.592`                                                   |
| `--far`                   | Far clipping plane distance.                                                                     | `1000`                                                      |
| `--near`                  | Near clipping plane distance.                                                                    | `0.1`                                                       |
| `--output_h`              | Output image height (pixels).                                                                    | `480`                                                       |
| `--output_w`              | Output image width (pixels).                                                                     | `752`                                                       |
| `--mesh_obj_file`         | Path to the mesh `.obj` file. Leave empty if using a model list.                                 | `model.obj` or empty                                        |
| `--mesh_texture_file`     | Path to the mesh texture file. Leave empty if using a model list.                                | `texture.png` or empty                                      |
| `--model_folder`          | Path to the folder containing 3D models.                                                         | `../example/models`                                         |
| `--model_list_file`       | Path to the file listing model definitions.                                                      | `../example/scene_irchel140821_and_four_capsules/model_def_list.txt` |
| `--model_pose_file`       | Path to the file listing model poses.                                                            | `../example/scene_irchel140821_and_four_capsules/model_poses_list.txt` |
| `--ortho`                 | Use orthographic projection (`true` or `false`).                                                 | `false`                                                     |
| `--pose_file`             | Path to the file containing camera poses.                                                        | `../example/image_poses.txt`                                |
| `--shader_folder`         | Path to the folder containing shader files.                                                      | `./shaders`                                                 |
| `--step_skip`             | Number of steps to skip between processed frames. This is useful if you use a high-rate IMU as a pose reference. For example, with `step_skip=10`, a pose file from a 200Hz IMU sensor will create a 20Hz camera rate. Otherwise, leave it as 1 so all the images in `image_poses.txt` will be created. | `1`                                                         |
| `--render_missing_images` | Also re-render frames that are recorded in the output `image_poses.csv` but whose `.h5` file is missing. When unset (default) such frames are left untouched. See the note on resuming below. | `false`                                                     |



**Note:**  
All paths are relative to the working directory where you run the application. Adjust them as needed for your setup.

**Resuming a render:**  
Each frame's pose is written to `<output_folder_path>/image_poses.csv` only after
that frame's `.h5` is saved, so the CSV is an accurate record of completed frames.
If that CSV already exists, the renderer resumes: frames already recorded whose
`.h5` is still present are skipped, and frames not yet recorded are rendered and
appended. Pass `--render_missing_images` to additionally re-render recorded frames
whose `.h5` has gone missing. To force a full fresh render, delete the output
folder (or its `image_poses.csv`) first.

## H5 Viewer

`vkg_dataset_viewer` browses a folder of `.h5` files produced by the CSV
renderer. It shows, in a single 2×2 window, the RGB image, a color-mapped depth
image (invalid/zero depth is drawn black), the semantics channel, and a text tile
with per-frame metadata.

Build it with:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_DATASET_VIEWER=ON
cmake --build build -j$(nproc)
```

Run it against an output folder:

```sh
vulkan_glasses/bin$ ./vkg_dataset_viewer --folder ../example/output_temp
```

Navigation keys:

| Key                 | Action        |
|---------------------|---------------|
| `space` / `→`       | next image    |
| `←`                 | previous image|
| `q` / `Esc`         | quit          |

### Flags

| Flag             | Description                                                                                                   | Example Value             |
|------------------|---------------------------------------------------------------------------------------------------------------|---------------------------|
| `--folder`       | Folder containing the `.h5` files to browse (required).                                                       | `../example/output_temp`  |
| `--pose_file`    | Optional pose file (`id, p_x..q_w`). If empty, `<folder>/image_poses.csv` is used when present.               | `../example/image_poses.txt` |
| `--depth_max`    | If `> 0`, color-map depth over the fixed metric range `[0, depth_max]` for consistent scaling across frames.  | `100`                     |
| `--start_index`  | Index of the first file to show.                                                                              | `0`                       |

The metadata tile shows the camera pose — position, quaternion, and roll/pitch/yaw
in degrees — read from the pose file. The CSV renderer writes an `image_poses.csv`
into its output folder while rendering, so the viewer picks it up automatically; if
no pose file is present the pose lines are simply omitted.