
#include <iostream>

#include <opencv2/highgui.hpp>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <gflags/gflags.h>
#include <glm/gtc/matrix_inverse.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>


#include <csv_processor.h>
#include <hdf5_utils.h>

///general arguments
//poses of the images to be produced
DEFINE_string(pose_file, "datafolder/image_poses.txt", "file with the image poses");
//allow to skip poses, value 1 render all images, value 2 renders every 2 images in the pose file...
DEFINE_int32(step_skip, 1, "step skip");
//output folder
DEFINE_string(output_folder_path, "", "result path");
//enable result display, not supported by cloud server.
DEFINE_bool(display,false,"show result images, one by one");
DEFINE_bool(display_wait,false,"wait for a key press after every render");
DEFINE_bool(dry_run,false,"disable saving the result.");
//folder with the compiled shaders.
DEFINE_string(shader_folder, "../shaders", "compiled shader folders");


///instrinsics defined by application arguments or calibration file
DEFINE_int32(output_w, 0, "width");
DEFINE_int32(output_h, 0, "height");
DEFINE_double(fx, 0, "fx");
DEFINE_double(fy, 0, "fy");
DEFINE_double(cx, 0, "cx");
DEFINE_double(cy, 0, "cy");
DEFINE_string(calibration_file, "", "compiled shader folders");

///type of camera projection and parameters
// define orthographic projection or perspective
DEFINE_bool(ortho,false,"use orthographic projection");
DEFINE_double(ortho_width, 0, "default orthographic width (0 means read from csv, one per image)");
// far and near should be as closer as possible to avoid depth fight for the perspective projection
DEFINE_double(far, 1000, "far");
DEFINE_double(near, 0.1, "near");

///input model: either set the mesh object and texture, or define a model based files and folder
// OPT 1: simple definition
DEFINE_string(mesh_obj_file, "", "compiled shader folders");
DEFINE_string(mesh_texture_file, "", "compiled shader folders");
//OPT 2: model based-files (has the preference if both defined)
DEFINE_string(model_folder, "", "base folder contaning the meshes"); // data_folder/base-op/
DEFINE_string(model_list_file, "", "file defining the list of models with relation to the base folder"); // data_folder/irchel1408/model_def_list.txt
DEFINE_string(model_pose_file, "", "compiled shader folders"); // data_folder/irchel1408/model_poses_list.txt





bool CSVProcessor::initialization(
    ) { //const std::string& csv_pose_file, const std::string& output_folder_path
    if (!boost::filesystem::exists(FLAGS_output_folder_path)) {
        if (!boost::filesystem::create_directories(FLAGS_output_folder_path)) {
            LOG(ERROR) << "the output folder could not be created :"
                       << FLAGS_output_folder_path.c_str();
            return false;
        }
    } else {
        LOG(ERROR)
                << "the output folder already exist - please delete it or "
                   "change the arg:"
                << FLAGS_output_folder_path.c_str();

         //return false;
    }


    pose_file_.open (FLAGS_pose_file, std::ifstream::in);

    if (!pose_file_.is_open()) {
        LOG(ERROR) << "could not open the pose file:" << FLAGS_pose_file;
        return false;
    }

    LOG(INFO) << "Display:" << FLAGS_display;

    output_folder_ = FLAGS_output_folder_path;

    if(FLAGS_step_skip < 1) // fix in case of user misuse
        FLAGS_step_skip = 1;

    initIntrinsics();
    initVulkan();

    return true;
}

void CSVProcessor::initVulkan()
{
    render_app = new vrglasses_for_robots::VulkanRenderer(w_,h_,FLAGS_near,FLAGS_far,FLAGS_shader_folder);

#if 1
    if(!FLAGS_model_folder.empty() && !FLAGS_model_list_file.empty())
    {
        render_app->loadMeshs(FLAGS_model_folder,FLAGS_model_list_file);

        if(!FLAGS_model_pose_file.empty())
        {
            LOG(INFO) << "Load with pose file: " << FLAGS_model_pose_file;
            render_app->loadScene(FLAGS_model_pose_file);
        }
        else
        {
            LOG(INFO) << "Load without scene file";
            render_app->noFileScene();
        }

    }
    else if( !FLAGS_mesh_obj_file.empty() &&  !FLAGS_mesh_texture_file.empty())
    {
        // Load Mesh
        LOG(INFO) << "Loading single model file obj:" << FLAGS_mesh_obj_file << " tex:" << FLAGS_mesh_texture_file;
        render_app->loadMesh(FLAGS_mesh_obj_file,FLAGS_mesh_texture_file);
        render_app->noFileScene();
    }
    else{
        LOG(ERROR) << "mesh_obj_file and mesh_texture_file need to be defined parameter, alternatively model_folder and model_list_file";
    }
#endif
    if(FLAGS_ortho)
    {
        if(FLAGS_ortho_width != 0)
        {
            render_app->buildOrthographicProjection(projection_matrix_,FLAGS_ortho_width,FLAGS_ortho_width*(h_/(float)w_),FLAGS_near,FLAGS_far);
        }
    }
    else
    {
        render_app->buildPerpectiveProjection(projection_matrix_,w_,h_,fx_,fy_,0,cx_,cy_,FLAGS_near,FLAGS_far);
    }

}

void CSVProcessor::parseLine(std::string line,std::vector<std::string>& vec)
{
    boost::tokenizer<boost::escaped_list_separator<char> > tk(
        line, boost::escaped_list_separator<char>('\\', ',', '\"'));
    for (boost::tokenizer<boost::escaped_list_separator<char> >::iterator i(tk.begin());
         i!=tk.end();++i)
    {
        std::string curr = *i;
        boost::trim(curr);
        vec.push_back(curr);
    }
}

void CSVProcessor::initIntrinsics()
{
    if(FLAGS_calibration_file.empty())
    {
        fx_ = FLAGS_fx;
        fy_ = FLAGS_fy;
        cx_ = FLAGS_cx;
        cy_ = FLAGS_cy;
        w_  = FLAGS_output_w;
        h_  = FLAGS_output_h;
    }else
    {
        std::ifstream calibration_file;
        calibration_file.open(FLAGS_calibration_file, std::ifstream::in);
        if(calibration_file.is_open())
        {
            std::string buffer;
            std::getline(calibration_file, buffer); //header
            std::getline(calibration_file, buffer); //data
            std::sscanf(buffer.c_str(), "%d,%d,%lf,%lf,%lf,%lf", &w_, &h_,&fx_,&fy_,&cx_,&cy_);
        }else
            throw std::runtime_error("could not find the calibration file");

    }

    //clean variable to avoid bugs.
    FLAGS_fx = 0;
    FLAGS_fy= 0;
    FLAGS_cx= 0;
    FLAGS_cy= 0;
    FLAGS_output_w= 0;
    FLAGS_output_h= 0;

}


double protected_double_cast(std::string str)
{
    try
    {
        return boost::lexical_cast<double>(str);
    }
    catch(boost::bad_lexical_cast& e)
    {
        LOG(INFO) << "!!"<<  "cast issue: " << e.what() << " #" << str <<"#!!";
        throw std::runtime_error("cast issue");
    }
    return -1000000;
}


void CSVProcessor::runHeadless()
{
    std::string line;
    std::vector<std::string> vec;
    std::getline(pose_file_,line);
    cv::Mat result_depth_map, result_attribute_map;

    int line_counter = -1;

    while (!pose_file_.eof())
    {
        vec.clear();
        std::getline(pose_file_,line);
        line_counter++;
        parseLine(line,vec);
        if(vec.size() != 8)
            continue;
        if(line_counter % FLAGS_step_skip)
            continue;

        std::cout << '\r' << "rendering line: " << line_counter+1 << std::flush;

        try
        {
            glm::vec3 position;
            glm::quat orientation;

            std::string id = vec[0] ;
            position.x = protected_double_cast(vec[1]);
            position.y = protected_double_cast(vec[2]);
            position.z = protected_double_cast(vec[3]);

            orientation.x = protected_double_cast(vec[4]);
            orientation.y = protected_double_cast(vec[5]);
            orientation.z = protected_double_cast(vec[6]);
            orientation.w = protected_double_cast(vec[7]);
            orientation = glm::normalize(orientation);

            renderPose(position,orientation,result_depth_map, result_attribute_map);

            if(!FLAGS_dry_run)
            {
                saveHdf5(id,result_depth_map, result_attribute_map);
            }            
        }
        catch(std::runtime_error& e)
        {
            LOG(INFO) << "#"<<  "cast issue: " << e.what()  <<"#";
            continue;
        }
    }

    LOG(INFO) << "#"<<  "stopVulkan" << "#";
    stopVulkan();
}



void CSVProcessor::stopVulkan()
{
    delete render_app;
    render_app = nullptr;
}

/*!
 * @brief inverse orthonormal rotation + translation matrix (ridig-body)
 *
 * @code
 * X = | R  T |   X' = | R' -R'T |
 *     | 0  1 |        | 0     1 |
 * @endcode
 *
 */

glm::mat4 pose_inverse(glm::mat4 in_mat)
{
    glm::mat3 rot = glm::mat3(in_mat);
    glm::mat3 rott = glm::transpose(rot);
    glm::mat4 result(rott);

    glm::vec3 t = in_mat[3];
    glm::vec3 t_out = -(rott * t);
    result[3] = glm::vec4(t_out,1.0);
    return result;
}

void CSVProcessor::glm2mvp(glm::vec3 position, glm::quat orientation,glm::mat4& mvp){
    glm::mat4 T_WC = glm::mat4_cast(orientation);

    T_WC[3] = glm::vec4(position,1.0);

    glm::mat4 T_CW = pose_inverse(T_WC);
    glm::mat4 conversion_gl_cv = glm::mat4(1,0,0,0,
                                           0,-1,0,0,
                                           0,0,-1,0,
                                           0,0,0,1);
    mvp = projection_matrix_ * conversion_gl_cv * T_CW ;

}

void CSVProcessor::renderPose(glm::vec3 position, glm::quat orientation, cv::Mat & result_depth_map, cv::Mat & result_attribute_map)
{
//    static long int counter = 0;
//    counter++;
//    std::cout << '\r' << counter << std::flush;

    glm::mat4 mvp;

    glm2mvp(position,orientation,mvp);
    //std::cout << " mvp " << glm::to_string(mvp) << std::endl;
    //std::cout << " perpective " << glm::to_string(projection_matrix_) << std::endl;
    render_app->setCamera(mvp);
    //cv::Mat result_depth_map, result_rgb_map, result_semantic_map;
    render_app->renderMesh(result_depth_map, result_attribute_map);

    if (FLAGS_display)
    {
        cv::Mat show_img, channels[4];
        cv::split(result_attribute_map,channels);
        cv::imshow("RGB",result_attribute_map);// only 3 first channels
        cv::imshow("Semantics",channels[3]);
        double min_depth = 0, max_depth = 100;
        //cv::minMaxLoc(mesh_depth_image, &min_depth, &max_depth);

        result_depth_map.convertTo(
            show_img, CV_8U, 255.0 / (max_depth - min_depth),
            -min_depth * 255.0 / (max_depth - min_depth));
        cv::imshow("depth map render", show_img);
        //LOG(INFO) << "rende " << min_depth << " / " << max_depth << " - "
        //          << okvis_reader.getNextId() << " / " << okvis_reader.size();

        if(FLAGS_display_wait)
        {
            cv::waitKey(0);
        }else
        {
            cv::waitKey(1);
        }
    }

}

void CSVProcessor::saveHdf5(std::string id, cv::Mat &depth_map, cv::Mat &attribute_map)
{
    cv::Mat gt_depth_image, image_r, image_g, image_b,image_s;

    cv::Mat channels[4];
    cv::split(attribute_map,channels);
    image_r = channels[0];
    image_g = channels[1];
    image_b = channels[2];
    image_s = channels[3];


    std::string output_file =
        (boost::filesystem::path(FLAGS_output_folder_path) / (id + ".h5"))
            .c_str();
    HighFive::File file(
        output_file, HighFive::File::ReadWrite | HighFive::File::Create |
                         HighFive::File::Truncate);

    // Use chunking
    HighFive::DataSetCreateProps props;
    props.add(HighFive::Chunking(std::vector<hsize_t>{1, 16, 16}));

    // Enable deflate
    props.add(HighFive::Deflate(6));

    MArray3u rgb_data;
    rgb_data.resize(
        boost::extents[4][static_cast<int>(
            std::floor(h_))]
                      [static_cast<int>(
                          std::floor(w_))]);

    MArray3f depth_data;
    depth_data.resize(
        boost::extents[1][static_cast<int>(
            std::floor(h_))]
                      [static_cast<int>(
                          std::floor(w_))]);

    MArray3u_view2 r_view =
        rgb_data[boost::indices[0][MArray3u::index_range()]
                               [MArray3u::index_range()]];


    Hdf5Utils::cv_mat_transposed_to_h5(image_r, r_view);

    //cv::imshow("red", image_r);
    //cv::waitKey(1);

    MArray3u_view2 g_view =
        rgb_data[boost::indices[1][MArray3u::index_range()]
                               [MArray3u::index_range()]];
    Hdf5Utils::cv_mat_transposed_to_h5(image_g, g_view);

    MArray3u_view2 b_view =
        rgb_data[boost::indices[2][MArray3u::index_range()]
                               [MArray3u::index_range()]];
    Hdf5Utils::cv_mat_transposed_to_h5(image_b, b_view);

    MArray3u_view2 s_view =
        rgb_data[boost::indices[3][MArray3u::index_range()]
                               [MArray3u::index_range()]];
    Hdf5Utils::cv_mat_transposed_to_h5(image_s, s_view);

    HighFive::DataSet rgb_image_ds = file.createDataSet<uchar>(
        "rgbs_data", HighFive::DataSpace::From(rgb_data), props);
    rgb_image_ds.write(rgb_data);

    // dense_image_data
    MArray3f_view2 gt_depth_view =
        depth_data[boost::indices[0][MArray3f::index_range()]
                                  [MArray3f::index_range()]];
    Hdf5Utils::cv_mat_float_image_to_h5(depth_map, gt_depth_view);

    HighFive::DataSet dense_image_ds = file.createDataSet<float>(
        "depth_data", HighFive::DataSpace::From(depth_data),
        props);
    dense_image_ds.write(depth_data);


    file.flush();
}

