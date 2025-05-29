

#include <hdf5_utils.h>

bool Hdf5Utils::exr_rgbd_read(const std::string& filepath, cv::Mat& image) {
  return false;
}

//bool Hdf5Utils::kindr_tf_to_h5(
//    const kindr::minimal::QuatTransformation& kindr_tf, MArray2d& h5_tf) {
//  h5_tf.resize(boost::extents[4][4]);
//  Eigen::Matrix<double, 4, 4, Eigen::ColMajor> eigen_tf =
//      kindr_tf.getTransformationMatrix();

//  for (MArray2d::index i = 0; i != 4; ++i)
//    for (MArray2d::index j = 0; j != 4; ++j)
//      h5_tf[i][j] = eigen_tf(i, j);
//}

bool Hdf5Utils::cv_mat_to_h5(const cv::Mat& cv_img, MArray3u_view2& image_out) {
  assert(image_out.shape()[1] == cv_img.rows);
  assert(image_out.shape()[0] == cv_img.cols);
  assert(cv_img.type() == CV_8U);
  for (size_t i = 0; i < image_out.shape()[1]; ++i)    // cv_img.rows
    for (size_t j = 0; j < image_out.shape()[0]; ++j)  // cv_img.cols
      image_out[j][i] = cv_img.at<unsigned char>(i, j);
  return true;
}

bool Hdf5Utils::cv_mat_transposed_to_h5(
    const cv::Mat& cv_img, MArray3u_view2& image_out) {
  assert(image_out.shape()[0] == cv_img.rows);
  assert(image_out.shape()[1] == cv_img.cols);
  assert(cv_img.type() == CV_8U);
  for (size_t i = 0; i < image_out.shape()[1]; ++i)    // cv_img.rows
    for (size_t j = 0; j < image_out.shape()[0]; ++j)  // cv_img.cols
      image_out[j][i] = cv_img.at<unsigned char>(j, i);
  return true;
}

bool Hdf5Utils::cv_mat_to_h5(const cv::Mat& cv_img, MArray3f_view2& image_out) {
  assert(image_out.shape()[1] == cv_img.rows);
  assert(image_out.shape()[0] == cv_img.cols);
  assert(cv_img.type() == CV_32F);
  for (size_t i = 0; i < image_out.shape()[1]; ++i)    // cv_img.rows
    for (size_t j = 0; j < image_out.shape()[0]; ++j)  // cv_img.cols
      image_out[j][i] = cv_img.at<float>(i, j);
  return true;
}

bool Hdf5Utils::cv_mat_float_image_to_h5(
    const cv::Mat& cv_img, MArray3f_view2& image_out) {
  assert(image_out.shape()[0] == cv_img.rows);
  assert(image_out.shape()[1] == cv_img.cols);
  assert(cv_img.type() == CV_32F);
  for (size_t j = 0; j < image_out.shape()[0]; ++j)    // cv_img.cols
    for (size_t i = 0; i < image_out.shape()[1]; ++i)  // cv_img.rows
      image_out[j][i] = cv_img.at<float>(j, i);
  return true;
}

bool Hdf5Utils::cv_mat_float_image_to_h5(
    const cv::Mat& cv_img, MArray3u_view2& image_out) {
  assert(image_out.shape()[1] == cv_img.cols);
  assert(image_out.shape()[0] == cv_img.rows);
  assert(cv_img.type() == CV_32F);
  for (size_t i = 0; i < image_out.shape()[0]; ++i)    // cv_img.rows
    for (size_t j = 0; j < image_out.shape()[1]; ++j)  // cv_img.cols
      image_out[i][j] = (unsigned char)(cv_img.at<float>(i, j) * 255.0);

  return true;
}

bool Hdf5Utils::cv_mat3_transposed_to_h5(
    const cv::Mat& cv_img, MArray3f_view3& image_out) {
  assert(image_out.shape()[1] == cv_img.rows);
  assert(image_out.shape()[2] == cv_img.cols);
  assert(cv_img.type() == CV_32FC3);
  for (size_t j = 0; j < image_out.shape()[1]; ++j)      // cv_img.cols
    for (size_t i = 0; i < image_out.shape()[2]; ++i)    // cv_img.rows
      for (size_t c = 0; c < image_out.shape()[0]; ++c)  // channels
        image_out[c][j][i] = cv_img.at<cv::Vec3f>(j, i)[c];
  return true;
}

bool Hdf5OutputFile::save(const cv::String& filepath) {
  return false;
}
