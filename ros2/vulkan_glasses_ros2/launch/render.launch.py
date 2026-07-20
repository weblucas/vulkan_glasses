"""Launch the vkg render node with parameters from a YAML config.

Override the config file or individual params on the command line, e.g.:
  ros2 launch vulkan_glasses_ros2 render.launch.py \
      config:=/path/to/params.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("vulkan_glasses_ros2")
    default_config = os.path.join(pkg_share, "config", "params.yaml")

    config_arg = DeclareLaunchArgument(
        "config",
        default_value=default_config,
        description="Path to the node parameter YAML file.",
    )

    node = Node(
        package="vulkan_glasses_ros2",
        executable="vkg_ros2_node",
        name="vkg_render_node",
        output="screen",
        parameters=[LaunchConfiguration("config")],
    )

    return LaunchDescription([config_arg, node])
