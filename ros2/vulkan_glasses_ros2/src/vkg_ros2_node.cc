// ROS2 node wrapping the vkg Vulkan renderer: subscribes to camera odometry and
// publishes the rendered color / depth / semantic images + CameraInfo. Port of
// the original ROS1 vrglasses_node to rclcpp, reusing the shared pose->MVP math
// in vkg/pose_utils.h so the coordinate conventions match the CSV renderer.

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/header.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <vkg/pose_utils.h>
#include <vkg/vulkan_renderer.h>

namespace {

// glm::quat is (w, x, y, z); ROS geometry_msgs quaternion is (x, y, z, w).
glm::quat toGlmQuat(const geometry_msgs::msg::Quaternion& q) {
  return glm::normalize(glm::quat(static_cast<float>(q.w), static_cast<float>(q.x),
                                  static_cast<float>(q.y), static_cast<float>(q.z)));
}

glm::mat4 toMat4(const glm::vec3& t, const glm::quat& r) {
  glm::mat4 m = glm::mat4_cast(r);
  m[3] = glm::vec4(t, 1.0f);
  return m;
}

}  // namespace

class VkgRenderNode : public rclcpp::Node {
 public:
  VkgRenderNode() : rclcpp::Node("vkg_render_node") {
    // --- Parameters -----------------------------------------------------------
    framerate_ = declare_parameter<double>("framerate", 20.0);
    camera_frame_id_ = declare_parameter<std::string>("camera_frame_id", "camera");
    width_ = declare_parameter<int>("output_w", 752);
    height_ = declare_parameter<int>("output_h", 480);
    near_ = declare_parameter<double>("render_near", 0.1);
    far_ = declare_parameter<double>("render_far", 1000.0);
    fx_ = declare_parameter<double>("fx", 571.63);
    fy_ = declare_parameter<double>("fy", 571.63);
    cx_ = declare_parameter<double>("cx", 366.23);
    cy_ = declare_parameter<double>("cy", 243.592);
    ortho_ = declare_parameter<bool>("ortho", false);
    ortho_width_ = declare_parameter<double>("ortho_width", 0.0);

    // Shader folder: default to the shaders installed in this package's share dir.
    std::string default_shaders =
        ament_index_cpp::get_package_share_directory("vulkan_glasses_ros2") +
        "/shaders";
    shader_folder_ = declare_parameter<std::string>("shader_folder", default_shaders);
    if (shader_folder_.empty()) shader_folder_ = default_shaders;  // "" -> package share

    model_folder_ = declare_parameter<std::string>("model_folder", "");
    model_list_file_ = declare_parameter<std::string>("model_list_file", "");
    model_pose_file_ = declare_parameter<std::string>("model_pose_file", "");
    mesh_obj_file_ = declare_parameter<std::string>("mesh_obj_file", "");
    texture_file_ = declare_parameter<std::string>("texture_file", "");

    // Optional body->camera extrinsic (default identity: odometry is the camera).
    auto ext_t = declare_parameter<std::vector<double>>(
        "body_to_camera_translation", {0.0, 0.0, 0.0});
    auto ext_q = declare_parameter<std::vector<double>>(
        "body_to_camera_quaternion_xyzw", {0.0, 0.0, 0.0, 1.0});
    glm::vec3 t_bc(ext_t.at(0), ext_t.at(1), ext_t.at(2));
    glm::quat q_bc = glm::normalize(
        glm::quat(static_cast<float>(ext_q.at(3)), static_cast<float>(ext_q.at(0)),
                  static_cast<float>(ext_q.at(1)), static_cast<float>(ext_q.at(2))));
    T_BC_ = toMat4(t_bc, q_bc);

    frame_period_ = rclcpp::Duration::from_seconds(framerate_ > 0.0 ? 1.0 / framerate_ : 0.0);

    setupRenderer();

    // --- Pub/sub --------------------------------------------------------------
    color_pub_ = create_publisher<sensor_msgs::msg::Image>("color_map", 1);
    depth_pub_ = create_publisher<sensor_msgs::msg::Image>("depth_map", 1);
    semantic_pub_ = create_publisher<sensor_msgs::msg::Image>("semantic_map", 1);
    camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 1);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odometry", rclcpp::QoS(50),
        std::bind(&VkgRenderNode::odomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "vkg_render_node ready (%dx%d, %.1f Hz)", width_,
                height_, framerate_);
  }

 private:
  void setupRenderer() {
    renderer_ = std::make_unique<vkg::VulkanRenderer>(
        static_cast<uint32_t>(width_), static_cast<uint32_t>(height_),
        static_cast<float>(near_), static_cast<float>(far_), shader_folder_);

    if (!model_folder_.empty() && !model_list_file_.empty()) {
      renderer_->loadMeshs(model_folder_, model_list_file_);
      if (!model_pose_file_.empty())
        renderer_->loadScene(model_pose_file_);
      else
        renderer_->noFileScene();
    } else if (!mesh_obj_file_.empty() && !texture_file_.empty()) {
      renderer_->loadMesh(mesh_obj_file_, texture_file_);
      renderer_->noFileScene();
    } else {
      RCLCPP_FATAL(get_logger(),
                   "provide model_folder+model_list_file or mesh_obj_file+texture_file");
      throw std::runtime_error("no mesh configuration");
    }

    if (ortho_) {
      if (ortho_width_ <= 0.0)
        throw std::runtime_error("ortho=true requires ortho_width > 0");
      renderer_->buildOrthographicProjection(
          projection_, static_cast<float>(ortho_width_),
          static_cast<float>(ortho_width_ * (height_ / static_cast<double>(width_))),
          static_cast<float>(near_), static_cast<float>(far_));
    } else {
      renderer_->buildPerpectiveProjection(projection_, width_, height_,
                                           static_cast<float>(fx_),
                                           static_cast<float>(fy_), 0.0f,
                                           static_cast<float>(cx_),
                                           static_cast<float>(cy_),
                                           static_cast<float>(near_),
                                           static_cast<float>(far_));
    }
    renderer_->setOrthographic(ortho_);
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Throttle to the target framerate.
    const rclcpp::Time stamp(msg->header.stamp);
    if (have_last_ && (stamp - last_stamp_) < frame_period_) return;
    last_stamp_ = stamp;
    have_last_ = true;

    // World<-body from odometry, composed with the body->camera extrinsic.
    glm::vec3 p(msg->pose.pose.position.x, msg->pose.pose.position.y,
                msg->pose.pose.position.z);
    glm::mat4 T_WB = toMat4(p, toGlmQuat(msg->pose.pose.orientation));
    glm::mat4 T_WC = T_WB * T_BC_;

    renderer_->setCamera(vkg::computeMvp(projection_, T_WC));
    renderer_->renderMesh(depth_map_, attribute_map_);  // CV_32FC1, CV_8UC4 (BGR+sem)

    // Split the 4-channel attribute map into BGR + semantics.
    cv::Mat channels[4];
    cv::split(attribute_map_, channels);
    std::vector<cv::Mat> bgr = {channels[0], channels[1], channels[2]};
    cv::Mat color;
    cv::merge(bgr, color);

    std_msgs::msg::Header header;
    header.stamp = msg->header.stamp;
    header.frame_id = camera_frame_id_;

    color_pub_->publish(*cv_bridge::CvImage(header, "bgr8", color).toImageMsg());
    depth_pub_->publish(*cv_bridge::CvImage(header, "32FC1", depth_map_).toImageMsg());
    semantic_pub_->publish(
        *cv_bridge::CvImage(header, "mono8", channels[3]).toImageMsg());
    camera_info_pub_->publish(makeCameraInfo(header));
  }

  sensor_msgs::msg::CameraInfo makeCameraInfo(const std_msgs::msg::Header& header) {
    sensor_msgs::msg::CameraInfo info;
    info.header = header;
    info.width = static_cast<uint32_t>(width_);
    info.height = static_cast<uint32_t>(height_);
    info.distortion_model = "plumb_bob";
    info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
    info.k = {fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0};
    info.p = {fx_, 0.0, cx_, 0.0, 0.0, fy_, cy_, 0.0, 0.0, 0.0, 1.0, 0.0};
    return info;
  }

  // Parameters / config.
  double framerate_, near_, far_, fx_, fy_, cx_, cy_, ortho_width_;
  int width_, height_;
  bool ortho_;
  std::string camera_frame_id_, shader_folder_, model_folder_, model_list_file_,
      model_pose_file_, mesh_obj_file_, texture_file_;
  glm::mat4 T_BC_{1.0f};
  glm::mat4 projection_{1.0f};
  rclcpp::Duration frame_period_{0, 0};
  rclcpp::Time last_stamp_;
  bool have_last_ = false;

  std::unique_ptr<vkg::VulkanRenderer> renderer_;
  cv::Mat depth_map_, attribute_map_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_, depth_pub_,
      semantic_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  // Single-threaded executor: the renderer is not reentrant.
  rclcpp::spin(std::make_shared<VkgRenderNode>());
  rclcpp::shutdown();
  return 0;
}
