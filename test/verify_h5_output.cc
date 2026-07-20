// Verifies that a rendered .h5 dataset is well-formed and has physically sane
// depth. Used by the example_render_smoke integration test to exercise the full
// render -> saveHdf5 -> read-back path (which a --dry_run render never touches).
//
// Usage: verify_h5_output <folder>   (checks the first .h5 found in <folder>)

#include <cstdio>
#include <filesystem>

#include <opencv2/core.hpp>

#include <h5_dataset.h>

namespace fs = std::filesystem;

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s <folder>\n", argv[0]);
    return 2;
  }

  std::string path;
  for (const auto& entry : fs::directory_iterator(argv[1])) {
    if (entry.is_regular_file() && entry.path().extension() == ".h5") {
      path = entry.path().string();
      break;
    }
  }
  if (path.empty()) {
    std::fprintf(stderr, "no .h5 file found in: %s\n", argv[1]);
    return 1;
  }

  cv::Mat rgb, depth, sem;
  if (!h5_dataset::read(path, rgb, depth, sem)) {
    std::fprintf(stderr, "h5_dataset::read failed for: %s\n", path.c_str());
    return 1;
  }

  int fails = 0;
  auto check = [&](bool ok, const char* what) {
    if (!ok) {
      std::fprintf(stderr, "FAIL: %s\n", what);
      ++fails;
    }
  };

  check(!rgb.empty() && rgb.type() == CV_8UC3, "rgb is non-empty CV_8UC3");
  check(depth.type() == CV_32FC1 && depth.size() == rgb.size(),
        "depth is CV_32FC1 matching rgb size");
  check(sem.type() == CV_8UC1 && sem.size() == rgb.size(),
        "semantics is CV_8UC1 matching rgb size");

  // Depth sanity: the frame must contain some rendered geometry (depth > 0),
  // and every valid depth must be finite and within a generous metric range.
  cv::Mat mask = depth > 0.0f;
  const int valid = cv::countNonZero(mask);
  check(valid > 0, "depth has at least one valid (>0) pixel");
  double mn = 0.0, mx = 0.0;
  if (valid > 0) cv::minMaxLoc(depth, &mn, &mx, nullptr, nullptr, mask);
  check(valid == 0 || (mn > 0.0 && mx < 1e5),
        "valid depth is within (0, 1e5] meters");

  std::printf(
      "verify_h5_output: %s  %dx%d  valid=%.0f%%  depth=[%.3f, %.3f] m  %s\n",
      path.c_str(), rgb.cols, rgb.rows,
      100.0 * valid / (rgb.rows * rgb.cols), mn, mx, fails ? "FAILED" : "OK");
  return fails ? 1 : 0;
}
