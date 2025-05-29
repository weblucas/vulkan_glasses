#ifndef MESH_DEPTH_EVALUATOR_HDF5_UTILS_H_
#define MESH_DEPTH_EVALUATOR_HDF5_UTILS_H_

#include <opencv2/core/core.hpp>
#include <string>
#include <vector>

#undef H5_USE_BOOST
#undef H5_HAVE_PARALLEL
#define H5_USE_BOOST


#include <boost/multi_array.hpp>
#include <highfive/H5DataSet.hpp>
#include <highfive/H5DataSpace.hpp>
#include <highfive/H5File.hpp>
//#include <kindr/minimal/quat-transformation.h>
#include <multi_array_types.h>

class Hdf5Utils {
 public:
  static bool exr_rgbd_read(const std::string& filepath, cv::Mat& image);
//  static bool kindr_tf_to_h5(
//      const kindr::minimal::QuatTransformation& tf_kindr, MArray2d& tf_h5);

  static bool cv_mat_to_h5(const cv::Mat& img, MArray3u_view2& image_out);
  static bool cv_mat_to_h5(const cv::Mat& img, MArray3f_view2& image_out);
  static bool cv_mat_float_image_to_h5(
      const cv::Mat& cv_img, MArray3f_view2& image_out);
  static bool cv_mat3_transposed_to_h5(
      const cv::Mat& cv_img, MArray3f_view3& image_out);
  static bool cv_mat_float_image_to_h5(
      const cv::Mat& cv_img, MArray3u_view2& image_out);
  static bool cv_mat_transposed_to_h5(
      const cv::Mat& cv_img, MArray3u_view2& image_out);
};

class Hdf5OutputFile {
 public:
  Hdf5OutputFile() {}
  ~Hdf5OutputFile() {}

  void append(const cv::String& layer_name, cv::Mat* image) {
    images_.push_back(std::make_pair(layer_name, image));
  }

  bool save(const cv::String& filepath);

 private:
  std::vector<std::pair<std::string, cv::Mat*>> images_;
};

#endif
