#ifndef VULKAN_GLASSES_H5_DATASET_H_
#define VULKAN_GLASSES_H5_DATASET_H_

#include <string>

#include <opencv2/core.hpp>

// Standalone read/write for the .h5 dataset produced by the CSV renderer.
//
// On-disk layout (root datasets, no groups/attributes):
//   "rgbs_data"  : uint8,   shape [4, H, W]  (channels 0/1/2 = B/G/R, 3 = semantics)
//   "depth_data" : float32, shape [1, H, W]  (raw metric depth, 0 = invalid)
// Both are chunked {1,16,16} and gzip-compressed (deflate level 6).
//
// This library depends only on OpenCV + HighFive/HDF5 (and glog for logging);
// it does NOT depend on Vulkan or the renderer, so any application can link it to
// load/save this format.
namespace h5_dataset {

// Read a file into OpenCV mats:
//   rgb       : CV_8UC3, BGR order (ready for cv::imshow)
//   depth     : CV_32FC1, raw metric depth
//   semantics : CV_8UC1 (zeros if the file stores only 3 channels)
// Returns false and logs on missing datasets or unexpected shapes.
bool read(const std::string& path, cv::Mat& rgb, cv::Mat& depth,
          cv::Mat& semantics);

// Write the dataset from OpenCV mats:
//   rgb       : CV_8UC3 (BGR); its 3 channels become dataset channels 0/1/2
//   depth     : CV_32FC1
//   semantics : CV_8UC1, becomes dataset channel 3. If empty, a zero channel is
//               written so the file always has the canonical [4,H,W] shape.
// rgb and depth must be non-empty and share the same size. Returns false and logs
// on invalid input or I/O failure.
bool write(const std::string& path, const cv::Mat& rgb, const cv::Mat& depth,
           const cv::Mat& semantics = cv::Mat());

}  // namespace h5_dataset

#endif  // VULKAN_GLASSES_H5_DATASET_H_
