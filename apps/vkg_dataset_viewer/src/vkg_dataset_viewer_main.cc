#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <vkg/h5_dataset.h>

DEFINE_string(folder, "", "folder containing the .h5 files to browse (required)");
DEFINE_string(pose_file, "",
              "optional pose file (id, p_x..q_w). If empty, <folder>/image_poses.csv "
              "is used when present.");
DEFINE_double(depth_max, 0,
              "if > 0, depth is color-mapped over the fixed range [0, depth_max] "
              "(metric) for consistent scaling across frames; otherwise per-frame "
              "min/max is used.");
DEFINE_int32(start_index, 0, "index of the first file to show");

namespace fs = std::filesystem;

namespace {

struct Pose {
  double px, py, pz;      // position
  double qx, qy, qz, qw;  // orientation quaternion
};

// Convert the stored camera-optical orientation quaternion to drone/aerospace
// roll/pitch/yaw in degrees.
//
// The pose quaternion is in the camera-optical frame (X-right, Y-down,
// Z-forward/into the scene). We first rotate it into an aerospace body frame
// (X-forward, Y-right, Z-down) via the fixed mapping Xb=+Zo, Yb=-Xo, Zb=-Yo,
// then extract the ZYX (yaw-Y-roll) Tait-Bryan angles. In this frame a level
// flight has roll ~ 0, a downward gimbal tilt shows up as pitch, and yaw is the
// heading. Ranges: roll (-180,180], pitch [-90,90], yaw [0,360).
//
// The formulas below are the closed form of
// R_body = R_world_optical(q) * [[0,-1,0],[0,0,-1],[1,0,0]] followed by a ZYX
// extraction; verified against scipy to < 1e-3 deg.
void quatToRpyDeg(const Pose& p, double& roll, double& pitch, double& yaw) {
  const double rad2deg = 180.0 / M_PI;
  const double x = p.qx, y = p.qy, z = p.qz, w = p.qw;

  double sinp = 2.0 * (x * x + y * y) - 1.0;      // = -R_body[2][0]
  sinp = std::max(-1.0, std::min(1.0, sinp));      // clamp at the singularity
  pitch = std::asin(sinp) * rad2deg;

  roll = std::atan2(y * w - x * z, -(y * z + x * w)) * rad2deg;

  yaw = std::atan2(y * z - x * w, x * z + y * w) * rad2deg;
  if (yaw < 0.0) yaw += 360.0;                     // normalize to [0, 360)
}

// Natural/numeric-aware comparison so "..._9" sorts before "..._10".
bool naturalLess(const std::string& a, const std::string& b) {
  size_t i = 0, j = 0;
  while (i < a.size() && j < b.size()) {
    if (std::isdigit(static_cast<unsigned char>(a[i])) &&
        std::isdigit(static_cast<unsigned char>(b[j]))) {
      size_t si = i, sj = j;
      while (i < a.size() && std::isdigit(static_cast<unsigned char>(a[i]))) ++i;
      while (j < b.size() && std::isdigit(static_cast<unsigned char>(b[j]))) ++j;
      std::string na = a.substr(si, i - si);
      std::string nb = b.substr(sj, j - sj);
      // Strip leading zeros for magnitude comparison.
      na.erase(0, std::min(na.find_first_not_of('0'), na.size() - 1));
      nb.erase(0, std::min(nb.find_first_not_of('0'), nb.size() - 1));
      if (na.size() != nb.size()) return na.size() < nb.size();
      if (na != nb) return na < nb;
    } else {
      if (a[i] != b[j]) return a[i] < b[j];
      ++i;
      ++j;
    }
  }
  return a.size() < b.size();
}

std::vector<std::string> trimSplit(const std::string& line) {
  std::vector<std::string> out;
  std::stringstream ss(line);
  std::string token;
  while (std::getline(ss, token, ',')) {
    size_t s = token.find_first_not_of(" \t\r\n");
    size_t e = token.find_last_not_of(" \t\r\n");
    out.push_back(s == std::string::npos ? "" : token.substr(s, e - s + 1));
  }
  return out;
}

// Parses a pose file (id, p_x, p_y, p_z, q_x, q_y, q_z, q_w). Header/garbage lines
// that don't parse are skipped. Keyed by id (column 0).
std::map<std::string, Pose> parsePoses(const std::string& path) {
  std::map<std::string, Pose> poses;
  std::ifstream in(path);
  if (!in.is_open()) return poses;
  std::string line;
  while (std::getline(in, line)) {
    std::vector<std::string> f = trimSplit(line);
    if (f.size() != 8 || f[0].empty()) continue;
    try {
      Pose p{std::stod(f[1]), std::stod(f[2]), std::stod(f[3]), std::stod(f[4]),
             std::stod(f[5]), std::stod(f[6]), std::stod(f[7])};
      poses[f[0]] = p;
    } catch (const std::exception&) {
      continue;  // header or malformed line
    }
  }
  return poses;
}

// Maps "DJI_0001.JPG.h5" -> "DJI_0001.JPG" (the pose-file id = column 0).
std::string idFromFilename(const std::string& filename) {
  const std::string ext = ".h5";
  if (filename.size() > ext.size() &&
      filename.compare(filename.size() - ext.size(), ext.size(), ext) == 0) {
    return filename.substr(0, filename.size() - ext.size());
  }
  return filename;
}

cv::Mat makeDepthColor(const cv::Mat& depth, double& min_valid, double& max_valid) {
  cv::Mat mask = depth > 0.0f;  // 0 marks invalid/background
  min_valid = 0.0;
  max_valid = 0.0;
  cv::minMaxLoc(depth, &min_valid, &max_valid, nullptr, nullptr, mask);

  cv::Mat norm8u;
  if (FLAGS_depth_max > 0) {
    depth.convertTo(norm8u, CV_8U, 255.0 / FLAGS_depth_max, 0.0);
  } else if (max_valid > min_valid) {
    depth.convertTo(norm8u, CV_8U, 255.0 / (max_valid - min_valid),
                    -min_valid * 255.0 / (max_valid - min_valid));
  } else {
    norm8u = cv::Mat::zeros(depth.size(), CV_8U);
  }

  cv::Mat colored;
  // COLORMAP_TURBO reads far better than JET but was only added in OpenCV 4.1.2.
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 2)
  cv::applyColorMap(norm8u, colored, cv::COLORMAP_TURBO);
#else
  cv::applyColorMap(norm8u, colored, cv::COLORMAP_JET);
#endif
  colored.setTo(cv::Scalar(0, 0, 0), ~mask);  // black background for invalid depth
  return colored;
}

void putLine(cv::Mat& tile, const std::string& text, int& y) {
  cv::putText(tile, text, cv::Point(12, y), cv::FONT_HERSHEY_SIMPLEX, 0.5,
              cv::Scalar(220, 220, 220), 1, cv::LINE_AA);
  y += 26;
}

cv::Mat makeMetaTile(const cv::Size& size, const std::string& filename, int index,
                     int total, int w, int h, double dmin, double dmax,
                     const Pose* pose) {
  cv::Mat tile = cv::Mat::zeros(size, CV_8UC3);
  int y = 32;
  putLine(tile, "file: " + filename, y);
  putLine(tile, "frame: " + std::to_string(index + 1) + " / " +
                    std::to_string(total),
          y);
  putLine(tile, "size: " + std::to_string(w) + " x " + std::to_string(h), y);
  char buf[128];
  std::snprintf(buf, sizeof(buf), "depth: %.3f .. %.3f", dmin, dmax);
  putLine(tile, buf, y);
  if (pose) {
    std::snprintf(buf, sizeof(buf), "pos:  %.3f  %.3f  %.3f", pose->px, pose->py,
                  pose->pz);
    putLine(tile, buf, y);
    std::snprintf(buf, sizeof(buf), "quat: %.3f %.3f %.3f %.3f", pose->qx,
                  pose->qy, pose->qz, pose->qw);
    putLine(tile, buf, y);
    double roll, pitch, yaw;
    quatToRpyDeg(*pose, roll, pitch, yaw);
    std::snprintf(buf, sizeof(buf), "rpy:  %.2f  %.2f  %.2f  (deg)", roll, pitch,
                  yaw);
    putLine(tile, buf, y);
  } else {
    putLine(tile, "pose: n/a", y);
  }
  y += 12;
  putLine(tile, "keys: space / ->  next", y);
  putLine(tile, "      <-           prev", y);
  putLine(tile, "      q / ESC      quit", y);
  return tile;
}

}  // namespace

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_logtostderr = true;

  if (FLAGS_folder.empty()) {
    LOG(ERROR) << "--folder is required";
    return 1;
  }
  if (!fs::exists(FLAGS_folder) || !fs::is_directory(FLAGS_folder)) {
    LOG(ERROR) << "not a directory: " << FLAGS_folder;
    return 1;
  }

  // Collect and sort the .h5 files.
  std::vector<std::string> files;
  for (const auto& entry : fs::directory_iterator(FLAGS_folder)) {
    if (entry.is_regular_file() && entry.path().extension() == ".h5") {
      files.push_back(entry.path().filename().string());
    }
  }
  if (files.empty()) {
    LOG(ERROR) << "no .h5 files found in: " << FLAGS_folder;
    return 1;
  }
  std::sort(files.begin(), files.end(), naturalLess);
  LOG(INFO) << "found " << files.size() << " .h5 files";

  // Resolve the pose file: explicit flag, else <folder>/image_poses.csv, else none.
  std::string pose_path = FLAGS_pose_file;
  if (pose_path.empty()) {
    fs::path candidate = fs::path(FLAGS_folder) / "image_poses.csv";
    if (fs::exists(candidate)) pose_path = candidate.string();
  }
  std::map<std::string, Pose> poses;
  if (!pose_path.empty()) {
    poses = parsePoses(pose_path);
    LOG(INFO) << "loaded " << poses.size() << " poses from " << pose_path;
  } else {
    LOG(INFO) << "no pose file found; pose metadata will be omitted";
  }

  const std::string window = "h5_viewer";
  cv::namedWindow(window, cv::WINDOW_NORMAL);

  int index = std::max(0, std::min(FLAGS_start_index,
                                   static_cast<int>(files.size()) - 1));
  int shown = -1;  // force initial draw
  while (true) {
    if (index != shown) {
      const std::string& fname = files[index];
      const std::string full = (fs::path(FLAGS_folder) / fname).string();
      cv::Mat rgb, depth, semantics;
      if (!h5_dataset::read(full, rgb, depth, semantics)) {
        LOG(ERROR) << "skipping unreadable file: " << fname;
      } else {
        double dmin = 0, dmax = 0;
        cv::Mat depth_color = makeDepthColor(depth, dmin, dmax);
        cv::Mat sem_bgr;
        cv::cvtColor(semantics, sem_bgr, cv::COLOR_GRAY2BGR);

        auto it = poses.find(idFromFilename(fname));
        const Pose* pose = (it != poses.end()) ? &it->second : nullptr;
        cv::Mat meta = makeMetaTile(rgb.size(), fname, index,
                                    static_cast<int>(files.size()), rgb.cols,
                                    rgb.rows, dmin, dmax, pose);

        cv::Mat top, bottom, canvas;
        cv::hconcat(rgb, depth_color, top);
        cv::hconcat(sem_bgr, meta, bottom);
        cv::vconcat(top, bottom, canvas);
        cv::imshow(window, canvas);
      }
      shown = index;
    }

    int key = cv::waitKeyEx(0);
    if (key == 'q' || key == 27) break;             // q / ESC
    if (key == ' ' || key == 65363 || key == 'd')   // space / -> / d
      index = std::min(index + 1, static_cast<int>(files.size()) - 1);
    else if (key == 65361 || key == 'a')            // <- / a
      index = std::max(index - 1, 0);
    else if (key == -1)                             // window closed
      break;
  }

  cv::destroyAllWindows();
  return 0;
}
