
#ifndef VRGLASSES4ROBOTS_CSV_PROCESSOR_H__
#define VRGLASSES4ROBOTS_CSV_PROCESSOR_H__

#include <glog/logging.h>

#include <boost/filesystem.hpp>
#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/hash.hpp>

#include <vulkan_renderer.h>

class CSVProcessor {
 public:
    CSVProcessor(){}
    bool initialization(
      );

    void runHeadless();

private:
    glm::mat4 projection_matrix_;

    std::ifstream pose_file_;
    boost::filesystem::path output_folder_;

    double fx_,fy_,cx_,cy_;
    int h_,w_;

    void parseLine(std::string line, std::vector<std::string> &vec);

    //renderer

    vrglasses_for_robots::VulkanRenderer* render_app;

    void initIntrinsics();

    void initVulkan();
    void stopVulkan();
    void renderPose(glm::vec3 position, glm::quat orientation, cv::Mat & depth_map, cv::Mat & attribute_map);
    void saveEXR(std::string id,cv::Mat & depth_map, cv::Mat & attribute_map);
    void saveHdf5(std::string id, cv::Mat & depth_map, cv::Mat & attribute_map);

    void glm2mvp(glm::vec3 position, glm::quat orientation, glm::mat4 &mvp);
};

#endif
