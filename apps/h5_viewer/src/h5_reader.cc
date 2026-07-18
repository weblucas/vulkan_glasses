#include <h5_reader.h>

#include <cstdint>
#include <vector>

#include <glog/logging.h>
#include <highfive/H5DataSet.hpp>
#include <highfive/H5File.hpp>
// HighFive reads directly into std::vector; boost::multi_array is not used here.

namespace {

// Rebuild an [H][W] channel from the flat channel-first buffer.
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

}  // namespace

bool readH5(const std::string& path, cv::Mat& rgb, cv::Mat& depth,
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
    // renderer stored a BGR cv::Mat), so merge them directly for cv::imshow.
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
