# vulkan_glasses
Fast Renderer for models with baked illumination or directly from photogrammetry
For ROS (Robot Operating System) support, which is a flexible framework for writing robot software, use https://github.com/VIS4ROB-lab/vulkan_glasses_for_robots.
It is the same library but integrated with ros simulation. The scene definition works in both softwares.

## Dependencies

The core library requires:

- CMake >= 3.14 and a C++17 compiler
- Vulkan SDK / development files (`glslangValidator` is used to compile the shaders)
- Eigen3
- OpenCV (`core`, `highgui`, `imgproc`)
- glog, gflags
- glm

The optional CSV renderer application (`BUILD_WITH_CSV_PROCESSOR`) and the `.h5`
image viewer (`BUILD_WITH_H5_VIEWER`) additionally require HDF5.

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

Useful options:

| Option                      | Default | Description                                  |
|-----------------------------|---------|----------------------------------------------|
| `BUILD_WITH_CSV_PROCESSOR`  | `OFF`   | Build the `vulkan_glasses_csv_renderer` app  |
| `BUILD_WITH_H5_VIEWER`      | `OFF`   | Build the `vulkan_glasses_h5_viewer` app     |
| `BUILD_TESTS`               | `OFF`   | Build the unit tests in `test/`              |
| `COMPILE_SHADERS`           | `ON`    | Compile the GLSL shaders to SPIR-V           |

To also build the CSV renderer application:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_WITH_CSV_PROCESSOR=ON
cmake --build build -j$(nproc)
```

## Installing

The install step copies the library, public headers and compiled shaders under
`bin/` by default (the install prefix is set in `CMakeLists.txt`):

```sh
cmake --install build
```

This produces:

```
bin/lib/libvulkan_glasses_lib.a
bin/include/vulkan_glasses/*.h
bin/shaders/*.spv
```

You can install elsewhere by overriding the prefix, e.g.
`cmake --install build --prefix ./install`.

### Installing both apps (self-contained)

To get a folder you can run the apps from directly, build with both apps enabled
and install into `install/`:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_WITH_CSV_PROCESSOR=ON -DBUILD_WITH_H5_VIEWER=ON
cmake --build build -j$(nproc)
cmake --install build --prefix "$(pwd)/install"
```

This produces a self-contained tree:

```
install/
├── vulkan_glasses_csv_renderer
├── vulkan_glasses_h5_viewer
├── run_csv_renderer.sh          # launcher: sets --shader_folder automatically
├── run_h5_viewer.sh
└── shaders/*.spv
```

Run the apps via the launcher scripts from any directory — they resolve the shader
folder relative to their own location:

```sh
install/run_csv_renderer.sh --flagfile=example/vk_glasses_csv_flags.txt
install/run_h5_viewer.sh --folder example/output_temp
```

The install tree is self-contained in project artifacts (binaries, shaders,
launchers). It still relies on the system-provided runtime libraries (OpenCV, HDF5,
glog, gflags) and a working Vulkan driver being installed on the machine.

## Testing

Unit tests live in `test/` and are built when `-DBUILD_TESTS=ON` is passed. The
`pose_math` and `csv_parse` tests are GPU-independent. If the CSV renderer app
is also built, an `example_render_smoke` test renders the bundled example scene
headless (requires a Vulkan driver):

```sh
cmake -S . -B build -DBUILD_TESTS=ON -DBUILD_WITH_CSV_PROCESSOR=ON
cmake --build build -j$(nproc)
ctest --test-dir build --output-on-failure
```

## Usage

```sh
vulkan_glasses/bin$ ./vulkan_glasses_csv_renderer --flagfile=../example/vk_glasses_csv_flags.txt
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

`vulkan_glasses_h5_viewer` browses a folder of `.h5` files produced by the CSV
renderer. It shows, in a single 2×2 window, the RGB image, a color-mapped depth
image (invalid/zero depth is drawn black), the semantics channel, and a text tile
with per-frame metadata.

Build it with:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_WITH_H5_VIEWER=ON
cmake --build build -j$(nproc)
```

Run it against an output folder:

```sh
vulkan_glasses/bin$ ./vulkan_glasses_h5_viewer --folder ../example/output_temp
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