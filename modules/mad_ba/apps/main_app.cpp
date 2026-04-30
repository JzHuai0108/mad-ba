// #include "point_cloud_proc.h"
#include <bits/stdc++.h>
#include <srrg_config/configurable_manager.h>
#include <srrg_config/pipeline_runner.h>
#include <srrg_data_structures/matrix.h>
#include <srrg_image/image.h>
#include <srrg_messages/message_handlers/message_file_source.h>
#include <srrg_messages/message_handlers/message_pack.h>
#include <srrg_messages/message_handlers/message_sorted_sink.h>
#include <srrg_messages/messages/camera_info_message.h>
#include <srrg_messages/messages/image_message.h>
#include <srrg_messages/messages/imu_message.h>
#include <srrg_messages/messages/transform_events_message.h>
#include <srrg_messages_ros/instances.h>
#include <srrg_pcl/instances.h>
#include <srrg_pcl/point_cloud.h>
#include <srrg_pcl/point_normal_curvature.h>
#include <srrg_pcl/point_projector.h>
#include <srrg_pcl/point_types.h>
#include <srrg_pcl/point_unprojector.h>
#include <srrg_solver/solver_core/solver.h>
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/shell_colors.h>
#include <srrg_system_utils/system_utils.h>
#include <mad_ba/point_cloud_proc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <filesystem>
#include <iostream>
#include <regex>
#include <vector>
#include <ros/ros.h>
#include <ros/package.h>

#include "pcd_folder_source.h"

using namespace srrg2_core;
using namespace srrg2_core_ros;
using namespace srrg2_lidar3d_utils;
using namespace srrg2_core;
using namespace srrg2_solver;

ConfigurableManager manager;
std::shared_ptr<PipelineRunner> runner;

namespace {
std::string shellEscapeJsonString(const std::string& value) {
    std::string escaped;
    escaped.reserve(value.size());
    for (const char c : value) {
        if (c == '\\' || c == '"') {
            escaped.push_back('\\');
        }
        escaped.push_back(c);
    }
    return escaped;
}

std::string inferCatkinDevelSpace(const std::string& package_path) {
    if (const char* env_devel = std::getenv("MAD_BA_DEVEL")) {
        if (*env_devel) {
            return env_devel;
        }
    }

    const std::string src_marker = "/src/";
    const std::size_t src_pos = package_path.find(src_marker);
    if (src_pos == std::string::npos) {
        return "";
    }
    return package_path.substr(0, src_pos) + "/devel";
}

std::string makeRuntimeDlConfig(const std::string& dl_config_path, const std::string& so_path) {
    if (so_path.empty()) {
        return dl_config_path;
    }

    std::ifstream input(dl_config_path);
    if (!input.good()) {
        throw std::runtime_error("unable to open dynamic loader config: " + dl_config_path);
    }

    std::stringstream buffer;
    buffer << input.rdbuf();
    std::string config = buffer.str();

    const std::regex so_paths_regex("\"so_paths\"\\s*:\\s*\\[[^\\]]*\\]");
    const std::string replacement = "\"so_paths\" : [ \"" + shellEscapeJsonString(so_path) + "\" ]";
    config = std::regex_replace(config, so_paths_regex, replacement, std::regex_constants::format_first_only);

    const std::filesystem::path runtime_path =
      std::filesystem::temp_directory_path() /
      ("mad_ba_dl_" + std::to_string(static_cast<long long>(getpid())) + ".conf");
    std::ofstream output(runtime_path);
    if (!output.good()) {
        throw std::runtime_error("unable to write runtime dynamic loader config: " + runtime_path.string());
    }
    output << config;
    return runtime_path.string();
}
}

int main(int argc, char** argv) {
    // Initalize ROS
    srrgInit(argc, argv, "mad_ba");
    ros::init(argc, argv, "mad_ba");

    // Parse commandline arguments
    ParseCommandLine cmd(argv);
    ArgumentString config_file(&cmd, "c", "config",   "config file to load",                          "");
    ArgumentString pcd_dir    (&cmd, "p", "pcd-dir",  "directory of undistorted PCD files",            "");
    ArgumentString tum_file   (&cmd, "t", "tum",      "TUM trajectory file with frame or selected frame poses",        "");
    ArgumentString dl_config  (&cmd, "dlc", "dl-config", "dynamic loader config file", "");
    ArgumentString dl_so_path (&cmd, "dlp", "dl-path",   "devel/install prefix where dynamic libraries are located", "");
    cmd.parse();

    // Find the dynamic libraries
    const std::string package_path = ros::package::getPath("mad_ba");
    const std::string dl_config_path = dl_config.value().empty() ? package_path + "/dl.conf" : dl_config.value();
    const std::string so_path = dl_so_path.value().empty() ? inferCatkinDevelSpace(package_path) : dl_so_path.value();
    const std::string runtime_dl_config_path = makeRuntimeDlConfig(dl_config_path, so_path);
    std::cerr << "main_app|dynamic loader config: " << runtime_dl_config_path << std::endl;
    if (!so_path.empty()) {
        std::cerr << "main_app|dynamic library prefix: " << so_path << std::endl;
    }
    ConfigurableManager::initFactory(runtime_dl_config_path);

    // Read the config file
    manager.read(config_file.value());

    if (!pcd_dir.value().empty() && !tum_file.value().empty()) {
        // --- PCD folder + TUM trajectory mode ---
        // PointCloudProc is configured via the config file (iter_num, output_folder, …).
        // The bag source / pipeline runner entries in the config are ignored.
        auto proc = manager.getByName<mad_ba::PointCloudProc>("point_cloud_proc");
        if (!proc) {
            std::cerr << std::string(environ[0])
                      << "|ERROR: cannot find 'point_cloud_proc' in config "
                      << config_file.value() << std::endl;
            return 1;
        }
        mad_ba::runFromPCDFolder(proc, pcd_dir.value(), tum_file.value());
    } else {
        // --- Original ROS bag pipeline mode ---
        runner = manager.getByName<PipelineRunner>("runner");
        if (!runner) {
            std::cerr << std::string(environ[0]) + "|ERROR, cannot find runner, maybe wrong configuration path!" << std::endl;
        }
        // Retrieve a bag source
        auto source = dynamic_pointer_cast<MessageFileSourceBase>(runner->param_source.value());
        if (!source) {
            std::cerr << std::string(environ[0]) + "|ERROR, cannot find source, maybe wrong configuration path!" << std::endl;
        }
        // Retrieve a sink
        auto sink = manager.getByName<MessageSortedSink>("sink");
        if (!sink) {
            std::cerr << std::string(environ[0]) + "|ERROR, cannot find sink, maybe wrong configuration path!" << std::endl;
        }

        runner->compute();

        manager.erase(runner);
        manager.erase(source);
        runner.reset();
        source.reset();
    }
    return 0;
}
