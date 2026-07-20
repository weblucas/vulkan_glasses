#ifndef VKG_POSE_UTILS_H_
#define VKG_POSE_UTILS_H_

// Camera-pose -> model-view-projection helpers, shared by the CSV renderer app
// and (later) the ROS2 node so the coordinate-convention-critical math lives in
// exactly one place. Header-only, glm-only — no Vulkan dependency.

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

namespace vkg {

// Inverse of a rigid-body transform [R|t]: [R^T | -R^T t]. Cheaper and more
// numerically stable than glm::inverse for an orthonormal rotation.
inline glm::mat4 poseInverse(const glm::mat4& in_mat) {
  glm::mat3 rot = glm::mat3(in_mat);
  glm::mat3 rott = glm::transpose(rot);
  glm::mat4 result(rott);
  glm::vec3 t = in_mat[3];
  result[3] = glm::vec4(-(rott * t), 1.0f);
  return result;
}

// Fixed OpenCV<->OpenGL camera-axis flip (negate Y and Z). Involutory.
inline glm::mat4 conversionGlCv() {
  return glm::mat4(1, 0, 0, 0,
                   0, -1, 0, 0,
                   0, 0, -1, 0,
                   0, 0, 0, 1);
}

// Model-view-projection for a camera whose body-to-world pose is
// world frame, given a projection matrix and the camera's body-to-world
// transform T_WC:
//   mvp = projection * (CV->GL flip) * world_to_camera
inline glm::mat4 computeMvp(const glm::mat4& projection, const glm::mat4& T_WC) {
  return projection * conversionGlCv() * poseInverse(T_WC);
}

// Overload taking the camera pose as (position, orientation).
inline glm::mat4 computeMvp(const glm::mat4& projection,
                            const glm::vec3& position,
                            const glm::quat& orientation) {
  glm::mat4 T_WC = glm::mat4_cast(orientation);
  T_WC[3] = glm::vec4(position, 1.0f);
  return computeMvp(projection, T_WC);
}

}  // namespace vkg

#endif  // VKG_POSE_UTILS_H_
