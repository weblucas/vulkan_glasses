# ROS2 Humble image for the vulkan_glasses_ros2 node.
# Build from the repo root:  docker build -f docker/ros2.Dockerfile -t vkg-ros2 .
FROM ros:humble

SHELL ["/bin/bash", "-c"]

# Build tools + Vulkan (dev headers + Mesa lavapipe for headless rendering) +
# the vkg libraries' system deps + the ROS message/bridge packages the node uses.
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake glslang-tools \
      libvulkan-dev mesa-vulkan-drivers vulkan-tools \
      libopencv-dev libeigen3-dev libhdf5-dev \
      libgflags-dev libgoogle-glog-dev libglm-dev \
      ros-humble-cv-bridge ros-humble-image-transport \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/vkg
COPY . /root/vkg

# One-step colcon build (the package add_subdirectory's ../../libs/*).
RUN source /opt/ros/humble/setup.bash \
    && colcon build --paths ros2/vulkan_glasses_ros2 \
                    --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source ROS + the overlay for interactive shells.
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc \
    && echo 'source /root/vkg/install/setup.bash' >> /root/.bashrc

# Headless fallback: use Mesa lavapipe if no GPU ICD is present at runtime.
ENV VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/lvp_icd.x86_64.json

CMD ["bash"]
