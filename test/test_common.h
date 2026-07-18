// Minimal dependency-free test helper (no external test framework needed).
// Each test file has its own main() and returns non-zero on failure so it can
// be registered with CTest via add_test().
#pragma once

#include <cmath>
#include <cstdio>
#include <string>

namespace vgtest {

inline int& failure_count() {
    static int failures = 0;
    return failures;
}

inline void report(bool ok, const char* expr, const char* file, int line) {
    if (!ok) {
        std::fprintf(stderr, "  FAIL: %s  (%s:%d)\n", expr, file, line);
        ++failure_count();
    }
}

inline bool approx(double a, double b, double eps = 1e-5) {
    return std::fabs(a - b) <= eps;
}

}  // namespace vgtest

#define CHECK(cond) ::vgtest::report((cond), #cond, __FILE__, __LINE__)
#define CHECK_APPROX(a, b) \
    ::vgtest::report(::vgtest::approx((a), (b)), #a " ~= " #b, __FILE__, __LINE__)

#define TEST_MAIN_RETURN() \
    return ::vgtest::failure_count() == 0 ? 0 : 1
