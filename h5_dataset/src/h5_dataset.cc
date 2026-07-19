#include <h5_dataset.h>

#include <cstddef>
#include <cstdint>
#include <vector>

#include <glog/logging.h>
#include <highfive/H5DataSet.hpp>
#include <highfive/H5DataSpace.hpp>
#include <highfive/H5File.hpp>
// HighFive reads/writes plain std::vector buffers; boost::multi_array is not used.

namespace h5_dataset {
namespace {

// Rebuild an [H][W] channel from a flat channel-first buffer:
// value at (row, col) = buffer[channel * H * W + row * W + col].
template <typename T>
cv::Mat channelToMat(const std::vector<T>& buffer, size_t channel, int h, int w,
                     int cv_type) {
  cv::Mat out(h, w, cv_type);
  const size_t offset = channel * static_cast<size_t>(h) * w;
  for (int row = 0; row < h; ++row) {
    T* dst = out.ptr<T>(row);
    const T* src = buffer.data() + offset + static_cast<size_t>(row) * w;
    for (int col = 0; col < w; ++col) dst[col] = src[col];
  }
  return out;
}

// Copy one CV_8UC1 channel into the flat [4,H,W] buffer at the given channel index.
void packChannel(const cv::Mat& ch, std::vector<unsigned char>& buf, size_t index,
                 size_t h, size_t w) {
  for (size_t row = 0; row < h; ++row)
    for (size_t col = 0; col < w; ++col)
      buf[index * h * w + row * w + col] =
          ch.at<unsigned char>(static_cast<int>(row), static_cast<int>(col));
}

}  // namespace

bool read(const std::string& path, cv::Mat& rgb, cv::Mat& depth,
          cv::Mat& semantics) {
  try {
    HighFive::File file(path, HighFive::File::ReadOnly);

    if (!file.exist("rgbs_data") || !file.exist("depth_data")) {
      LOG(ERROR) << "missing rgbs_data/depth_data dataset in: " << path;
      return false;
    }

    HighFive::DataSet rgb_ds = file.getDataSet("rgbs_data");
    HighFive::DataSet depth_ds = file.getDataSet("depth_data");

    std::vector<size_t> rgb_dims = rgb_ds.getDimensions();
    std::vector<size_t> depth_dims = depth_ds.getDimensions();

    if (rgb_dims.size() != 3 || rgb_dims[0] < 3) {
      LOG(ERROR) << "unexpected rgbs_data shape in: " << path;
      return false;
    }
    if (depth_dims.size() != 3 || depth_dims[0] < 1) {
      LOG(ERROR) << "unexpected depth_data shape in: " << path;
      return false;
    }

    const int h = static_cast<int>(rgb_dims[1]);
    const int w = static_cast<int>(rgb_dims[2]);
    if (static_cast<int>(depth_dims[1]) != h ||
        static_cast<int>(depth_dims[2]) != w) {
      LOG(ERROR) << "rgb/depth dimension mismatch in: " << path;
      return false;
    }

    // Read into pre-sized flat buffers via the raw-pointer overload; HighFive
    // fills contiguous row-major memory, matching the [C][H][W] on-disk order.
    // No transpose needed: element [c][row][col] maps directly.
    std::vector<uint8_t> rgb_buf(rgb_dims[0] * rgb_dims[1] * rgb_dims[2]);
    rgb_ds.read(rgb_buf.data());
    std::vector<float> depth_buf(depth_dims[0] * depth_dims[1] * depth_dims[2]);
    depth_ds.read(depth_buf.data());

    // Dataset channels 0,1,2 are already in OpenCV's native BGR order (the
    // renderer stored a BGR cv::Mat), so merge them directly.
    cv::Mat ch0 = channelToMat(rgb_buf, 0, h, w, CV_8UC1);
    cv::Mat ch1 = channelToMat(rgb_buf, 1, h, w, CV_8UC1);
    cv::Mat ch2 = channelToMat(rgb_buf, 2, h, w, CV_8UC1);
    std::vector<cv::Mat> bgr = {ch0, ch1, ch2};
    cv::merge(bgr, rgb);

    if (rgb_dims[0] >= 4) {
      semantics = channelToMat(rgb_buf, 3, h, w, CV_8UC1);
    } else {
      semantics = cv::Mat::zeros(h, w, CV_8UC1);
    }

    depth = channelToMat(depth_buf, 0, h, w, CV_32FC1);
    return true;
  } catch (const std::exception& e) {
    LOG(ERROR) << "failed to read " << path << ": " << e.what();
    return false;
  }
}

bool write(const std::string& path, const cv::Mat& rgb, const cv::Mat& depth,
           const cv::Mat& semantics) {
  try {
    if (rgb.empty() || depth.empty()) {
      LOG(ERROR) << "write: rgb/depth must be non-empty: " << path;
      return false;
    }
    if (rgb.type() != CV_8UC3) {
      LOG(ERROR) << "write: rgb must be CV_8UC3 (BGR): " << path;
      return false;
    }
    if (depth.type() != CV_32FC1 || depth.size() != rgb.size()) {
      LOG(ERROR) << "write: depth must be CV_32FC1 and match rgb size: " << path;
      return false;
    }

    const size_t h = static_cast<size_t>(rgb.rows);
    const size_t w = static_cast<size_t>(rgb.cols);

    cv::Mat sem;
    if (semantics.empty()) {
      sem = cv::Mat::zeros(rgb.rows, rgb.cols, CV_8UC1);
    } else if (semantics.type() == CV_8UC1 && semantics.size() == rgb.size()) {
      sem = semantics;
    } else {
      LOG(ERROR) << "write: semantics must be CV_8UC1 and match rgb size: " << path;
      return false;
    }

    // Pack channels into a flat [4,H,W] buffer: 0/1/2 = rgb's B/G/R split, 3 = sem.
    cv::Mat bgr[3];
    cv::split(rgb, bgr);
    std::vector<unsigned char> rgb_buf(4 * h * w);
    packChannel(bgr[0], rgb_buf, 0, h, w);
    packChannel(bgr[1], rgb_buf, 1, h, w);
    packChannel(bgr[2], rgb_buf, 2, h, w);
    packChannel(sem, rgb_buf, 3, h, w);

    std::vector<float> depth_buf(h * w);
    for (size_t row = 0; row < h; ++row)
      for (size_t col = 0; col < w; ++col)
        depth_buf[row * w + col] =
            depth.at<float>(static_cast<int>(row), static_cast<int>(col));

    HighFive::File file(path, HighFive::File::ReadWrite | HighFive::File::Create |
                                  HighFive::File::Truncate);

    HighFive::DataSetCreateProps props;
    props.add(HighFive::Chunking(std::vector<hsize_t>{1, 16, 16}));
    props.add(HighFive::Deflate(6));

    HighFive::DataSet rgb_ds = file.createDataSet<unsigned char>(
        "rgbs_data", HighFive::DataSpace({4, h, w}), props);
    rgb_ds.write_raw(rgb_buf.data());

    HighFive::DataSet depth_ds = file.createDataSet<float>(
        "depth_data", HighFive::DataSpace({1, h, w}), props);
    depth_ds.write_raw(depth_buf.data());

    file.flush();
    return true;
  } catch (const std::exception& e) {
    LOG(ERROR) << "failed to write " << path << ": " << e.what();
    return false;
  }
}

}  // namespace h5_dataset
