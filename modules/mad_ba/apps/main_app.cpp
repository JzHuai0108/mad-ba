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

#include <iostream>
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

int main(int argc, char** argv) {
    // Initalize ROS
    srrgInit(argc, argv, "mad_ba");
    ros::init(argc, argv, "mad_ba");

    // Parse commandline arguments
    ParseCommandLine cmd(argv);
    ArgumentString config_file(&cmd, "c", "config",   "config file to load",                          "");
    ArgumentString pcd_dir    (&cmd, "p", "pcd-dir",  "directory of undistorted PCD files",            "");
    ArgumentString tum_file   (&cmd, "t", "tum",      "TUM trajectory file with keyframe poses",        "");
    cmd.parse();

    // Find the dynamic libraries
    const std::string dl_path = ros::package::getPath("mad_ba") + "/dl.conf";
    ConfigurableManager::initFactory(dl_path);

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
