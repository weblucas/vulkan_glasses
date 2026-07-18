#ifndef VULKAN_GLASSES_H5_VIEWER_H5_READER_H_
#define VULKAN_GLASSES_H5_VIEWER_H5_READER_H_

#include <string>

#include <opencv2/core.hpp>

// Reads a single .h5 file produced by the csv_renderer.
//
// On-disk layout (see CSVProcessor::saveHdf5):
//   dataset "rgbs_data" : uint8,  shape [4, H, W]  (ch 0/1/2 = R/G/B, ch 3 = semantics)
//   dataset "depth_data": float,  shape [1, H, W]  (raw metric depth, 0 = invalid)
//
// Outputs (OpenCV conventions):
//   rgb       : CV_8UC3, BGR order (ready for cv::imshow)
//   depth     : CV_32FC1, raw metric depth
//   semantics : CV_8UC1
//
// Returns false (and logs) on missing datasets or unexpected shapes.
bool readH5(const std::string& path, cv::Mat& rgb, cv::Mat& depth,
            cv::Mat& semantics);

#endif  // VULKAN_GLASSES_H5_VIEWER_H5_READER_H_
