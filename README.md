# vulkan_glasses
Fast Renderer for models with baked illumination or directly from photogrammetry

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



**Note:**  
All paths are relative to the working directory where you run the application. Adjust them as needed for your setup.