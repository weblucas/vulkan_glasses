# Non-ROS image: builds the vkg libraries + standalone apps.
# Build from the repo root:  docker build -f docker/standalone.Dockerfile -t vkg .
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake glslang-tools \
      libvulkan-dev mesa-vulkan-drivers vulkan-tools \
      libopencv-dev libeigen3-dev libhdf5-dev \
      libgflags-dev libgoogle-glog-dev libglm-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/vkg
COPY . /root/vkg

RUN cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_CSV_RENDERER=ON -DBUILD_DATASET_VIEWER=ON -DBUILD_TESTS=ON \
    && cmake --build build -j"$(nproc)"

ENV VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/lvp_icd.x86_64.json
CMD ["bash"]
