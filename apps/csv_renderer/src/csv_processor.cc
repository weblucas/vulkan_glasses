
#include <iostream>

#include <opencv2/highgui.hpp>
#include <chrono>
#include <cstdio>
#include <deque>
#include <stdexcept>
#include <vector>
#include <gflags/gflags.h>
#include <glm/gtc/matrix_inverse.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>

#include <csv_processor.h>
#include <h5_dataset.h>
#include <string_utils.h>

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
//Resume is always active: frames already recorded in the output image_poses.csv
//whose .h5 is present are never re-rendered, and frames not yet recorded are
//always rendered and appended. This flag additionally re-renders recorded frames
//whose .h5 file has gone missing (off by default).
DEFINE_bool(render_missing_images,false,"also re-render frames that are recorded in the output image_poses.csv but whose .h5 file is missing (off: such frames are left as-is)");
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
    if (!std::filesystem::exists(FLAGS_output_folder_path)) {
        if (!std::filesystem::create_directories(FLAGS_output_folder_path)) {
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

    if(!FLAGS_dry_run)
    {
        // Record the pose of every rendered frame next to the .h5 files so the
        // h5_viewer can display it. Format mirrors the input pose file. The line
        // is written only after the frame is successfully saved (see runHeadless),
        // so this file is an accurate record of completed renders and can be used
        // to resume a preempted process.
        std::filesystem::path pose_out_path = output_folder_ / "image_poses.csv";

        if (std::filesystem::exists(pose_out_path))
        {
            // Continue an existing render: keep the prior records and append the
            // frames that are not yet recorded (resume is always active; the
            // --render_missing_images flag only controls re-rendering of recorded
            // frames whose .h5 file has gone missing, see runHeadless).
            loadRecordedIds(pose_out_path.string());
            LOG(INFO) << "found existing pose file with " << recorded_ids_.size()
                      << " recorded frames; continuing in " << pose_out_path.c_str();
            pose_out_file_.open(pose_out_path.c_str(),
                                std::ofstream::out | std::ofstream::app);
            if (!pose_out_file_.is_open()) {
                LOG(ERROR) << "could not open the pose output file:" << pose_out_path.c_str();
                return false;
            }
        }
        else
        {
            // No prior record: create the file and write the header.
            pose_out_file_.open(pose_out_path.c_str(), std::ofstream::out);
            if (!pose_out_file_.is_open()) {
                LOG(ERROR) << "could not open the pose output file:" << pose_out_path.c_str();
                return false;
            }
            pose_out_file_ << "id, p_x, p_y, p_z, q_x, q_y, q_z, q_w\n";
        }
    }

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
        if(FLAGS_ortho_width == 0)
            throw std::runtime_error(
                "--ortho requires --ortho_width > 0 (world-units width of the view)");
        render_app->buildOrthographicProjection(projection_matrix_,FLAGS_ortho_width,FLAGS_ortho_width*(h_/(float)w_),FLAGS_near,FLAGS_far);
    }
    else
    {
        render_app->buildPerpectiveProjection(projection_matrix_,w_,h_,fx_,fy_,0,cx_,cy_,FLAGS_near,FLAGS_far);
    }
    // Depth read-back must match the projection (linear for ortho).
    render_app->setOrthographic(FLAGS_ortho);

}

void CSVProcessor::loadRecordedIds(const std::string& csv_path)
{
    std::ifstream in(csv_path);
    if (!in.is_open())
        return;
    std::string line;
    std::getline(in, line);  // header
    while (std::getline(in, line))
    {
        std::vector<std::string> vec;
        parseLine(line, vec);
        if (vec.size() == 8 && !vec[0].empty())
            recorded_ids_.insert(vec[0]);
    }
}

void CSVProcessor::parseLine(std::string line,std::vector<std::string>& vec)
{
    for (const std::string& field : vg_str::split(line, ','))
        vec.push_back(vg_str::trim(field));
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
        size_t consumed = 0;
        double value = std::stod(str, &consumed);
        // Reject trailing garbage to match boost::lexical_cast's strictness.
        if (consumed != str.size())
            throw std::invalid_argument("trailing characters");
        return value;
    }
    catch(const std::exception& e)
    {
        LOG(INFO) << "!!"<<  "cast issue: " << e.what() << " #" << str <<"#!!";
        throw std::runtime_error("cast issue");
    }
    return -1000000;
}


namespace {
// One validated camera pose loaded from the input pose file.
struct PoseEntry {
    std::string id;
    glm::vec3 position;
    glm::quat orientation;
};
}  // namespace

void CSVProcessor::runHeadless()
{
    // Preload and validate the whole pose file in a single pass: parse each data
    // line, drop the header and any malformed/uncastable line, and keep the valid
    // poses. The render loop then just iterates this vector, so the file is read
    // exactly once and the total is known up front for the progress display.
    std::vector<PoseEntry> entries;
    {
        std::string line;
        std::getline(pose_file_, line);  // header
        while (std::getline(pose_file_, line))
        {
            std::vector<std::string> vec;
            parseLine(line, vec);
            if (vec.size() != 8 || vec[0].empty())
                continue;  // header repeat, blank, or malformed line

            PoseEntry entry;
            entry.id = vec[0];
            try
            {
                entry.position = glm::vec3(protected_double_cast(vec[1]),
                                           protected_double_cast(vec[2]),
                                           protected_double_cast(vec[3]));
                entry.orientation.x = protected_double_cast(vec[4]);
                entry.orientation.y = protected_double_cast(vec[5]);
                entry.orientation.z = protected_double_cast(vec[6]);
                entry.orientation.w = protected_double_cast(vec[7]);
            }
            catch (const std::runtime_error& e)
            {
                LOG(INFO) << "skipping pose line (" << e.what() << "): " << line;
                continue;
            }
            entry.orientation = glm::normalize(entry.orientation);
            entries.push_back(std::move(entry));
        }
    }

    const int total = static_cast<int>(entries.size());
    LOG(INFO) << "loaded " << total << " valid pose entries";

    // Number of entries that pass the step_skip filter, i.e. the frames that will
    // actually be rendered (FLAGS_step_skip is >= 1, clamped in initialization).
    // Used for the ETA; on a resume, frames whose .h5 already exists are skipped
    // instantly, so the ETA is a (conservative) upper bound in that case.
    const int total_to_render = total > 0 ? (total - 1) / FLAGS_step_skip + 1 : 0;

    // Rolling window of the last 10 render+save durations (seconds) for a moving
    // average and ETA.
    std::deque<double> recent_times;
    int rendered_count = 0;

    cv::Mat result_depth_map, result_attribute_map;

    for (int i = 0; i < total; ++i)
    {
        // step_skip subsamples the loaded frames (e.g. a 200 Hz pose log with
        // step_skip=10 renders at 20 Hz).
        if (i % FLAGS_step_skip)
            continue;

        const PoseEntry& entry = entries[i];

        // Progress line: current/total plus a moving average and ETA from the
        // last <=10 rendered frames (shown as "--" until the first one finishes).
        // Trailing spaces pad over the previous, possibly longer, line.
        char status[128];
        if (!recent_times.empty())
        {
            double avg = 0.0;
            for (double t : recent_times) avg += t;
            avg /= recent_times.size();
            const double eta = avg * (total_to_render - rendered_count);
            std::snprintf(status, sizeof(status),
                          "  |  avg %.3f s/frame (last %zu)  |  ETA %.0f s        ",
                          avg, recent_times.size(), eta);
        }
        else
        {
            std::snprintf(status, sizeof(status), "  |  avg --  |  ETA --        ");
        }
        std::cout << '\r' << "rendering: " << (i + 1) << " / " << total << status
                  << std::flush;

        // Resume logic:
        //  - recorded & .h5 present  -> always skip (never re-render)
        //  - recorded & .h5 missing  -> re-render only if --render_missing_images
        //  - not recorded            -> always render + append the pose line
        const bool recorded = recorded_ids_.count(entry.id) > 0;
        if (recorded)
        {
            std::filesystem::path h5_path = output_folder_ / (entry.id + ".h5");
            if (std::filesystem::exists(h5_path))
                continue;  // already rendered and present: never re-render
            if (!FLAGS_render_missing_images)
                continue;  // recorded but missing, and re-render disabled
            // recorded but missing, and re-render enabled: fall through.
        }

        try
        {
            const auto t_start = std::chrono::steady_clock::now();

            renderPose(entry.position, entry.orientation, result_depth_map,
                       result_attribute_map);

            if (!FLAGS_dry_run)
            {
                // Append the pose line only if the .h5 was actually written and the
                // frame was not already recorded, so image_poses.csv stays an
                // accurate record of completed frames (used to resume) and never
                // lists a frame whose save failed. Flush so the record survives a
                // preemption.
                const bool saved =
                    saveHdf5(entry.id, result_depth_map, result_attribute_map);
                if (saved && !recorded)
                {
                    pose_out_file_ << entry.id << ',' << entry.position.x << ','
                                   << entry.position.y << ',' << entry.position.z
                                   << ',' << entry.orientation.x << ','
                                   << entry.orientation.y << ','
                                   << entry.orientation.z << ','
                                   << entry.orientation.w << '\n';
                    pose_out_file_.flush();
                }
            }

            const auto t_end = std::chrono::steady_clock::now();
            recent_times.push_back(
                std::chrono::duration<double>(t_end - t_start).count());
            if (recent_times.size() > 10)
                recent_times.pop_front();
            ++rendered_count;
        }
        catch (const std::exception& e)
        {
            LOG(WARNING) << "skipping frame " << entry.id << ": " << e.what();
            continue;
        }
    }

    // Terminate the in-place ('\r') progress line with a newline so the next log
    // message starts on its own line instead of being appended to it.
    std::cout << std::endl;

    LOG(INFO) << "#"<<  "stopVulkan" << "#";
    stopVulkan();
}



void CSVProcessor::stopVulkan()
{
    if (pose_out_file_.is_open())
    {
        pose_out_file_.flush();
        pose_out_file_.close();
    }
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

bool CSVProcessor::saveHdf5(std::string id, cv::Mat &depth_map, cv::Mat &attribute_map)
{
    // attribute_map is 4-channel: 0/1/2 = B/G/R, 3 = semantics. Split it into an
    // rgb (BGR) mat + a semantics mat and delegate to the standalone dataset lib,
    // which owns the on-disk format.
    cv::Mat channels[4];
    cv::split(attribute_map, channels);

    cv::Mat rgb;
    std::vector<cv::Mat> bgr = {channels[0], channels[1], channels[2]};
    cv::merge(bgr, rgb);

    std::string output_file =
        (std::filesystem::path(FLAGS_output_folder_path) / (id + ".h5")).string();

    if (!h5_dataset::write(output_file, rgb, depth_map, channels[3])) {
        LOG(ERROR) << "failed to save " << output_file;
        return false;
    }
    return true;
}

