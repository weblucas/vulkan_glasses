// Unit tests for the camera-pose geometry used by the CSV renderer.
//
// These validate the exact math in apps/csv_renderer/src/csv_processor.cc:
//   * pose_inverse()  (rigid-body transform inverse, line ~285)
//   * the OpenCV->OpenGL axis conversion applied in glm2mvp()
// The code is small and GPU-independent, so it is reproduced here verbatim and
// checked against glm's own routines and against mathematical invariants.

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include "test_common.h"

// Mirror of CSVProcessor's free function pose_inverse() (csv_processor.cc).
static glm::mat4 pose_inverse(glm::mat4 in_mat) {
    glm::mat3 rot = glm::mat3(in_mat);
    glm::mat3 rott = glm::transpose(rot);
    glm::mat4 result(rott);

    glm::vec3 t = in_mat[3];
    glm::vec3 t_out = -(rott * t);
    result[3] = glm::vec4(t_out, 1.0);
    return result;
}

static void check_matrices_equal(const glm::mat4& a, const glm::mat4& b,
                                 double eps = 1e-4) {
    for (int c = 0; c < 4; ++c)
        for (int r = 0; r < 4; ++r)
            ::vgtest::report(::vgtest::approx(a[c][r], b[c][r], eps),
                             "matrix element mismatch", __FILE__, __LINE__);
}

int main() {
    const glm::mat4 I(1.0f);

    // Build a rigid transform from a rotation + translation.
    glm::quat q = glm::normalize(glm::quat(0.383373f, -0.922348f, -0.046673f,
                                           0.010963f));  // w, x, y, z
    glm::mat4 T = glm::mat4_cast(q);
    T[3] = glm::vec4(19.256670f, -11.677642f, 6.314486f, 1.0f);

    // 1. pose_inverse composed with the original yields the identity.
    check_matrices_equal(T * pose_inverse(T), I);
    check_matrices_equal(pose_inverse(T) * T, I);

    // 2. For a rigid transform, pose_inverse must equal glm::inverse.
    check_matrices_equal(pose_inverse(T), glm::inverse(T));

    // 3. Applying the inverse twice returns the original transform.
    check_matrices_equal(pose_inverse(pose_inverse(T)), T);

    // 4. The OpenCV<->OpenGL axis-flip matrix used by glm2mvp is an involution.
    glm::mat4 conversion_gl_cv = glm::mat4(1, 0, 0, 0,
                                           0, -1, 0, 0,
                                           0, 0, -1, 0,
                                           0, 0, 0, 1);
    check_matrices_equal(conversion_gl_cv * conversion_gl_cv, I);

    // 5. Translation is recovered as -R^T * t.
    glm::mat4 inv = pose_inverse(T);
    glm::vec3 t(T[3]);
    glm::mat3 Rt = glm::transpose(glm::mat3(T));
    glm::vec3 expected_t = -(Rt * t);
    CHECK_APPROX(inv[3][0], expected_t.x);
    CHECK_APPROX(inv[3][1], expected_t.y);
    CHECK_APPROX(inv[3][2], expected_t.z);

    if (::vgtest::failure_count() == 0)
        std::printf("test_pose_math: all checks passed\n");
    TEST_MAIN_RETURN();
}
