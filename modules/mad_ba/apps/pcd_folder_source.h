#pragma once

#include <cstdint>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>

#include <boost/make_shared.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <srrg_converters/converter.h>
#include <srrg_messages/messages/odometry_message.h>
#include <srrg_messages/messages/point_cloud2_message.h>

#include <mad_ba/point_cloud_proc.h>

namespace mad_ba {

inline bool parseTimestamp(const std::string& ts_str, uint32_t& secs, uint32_t& nsecs) {
    const size_t dot_pos = ts_str.find('.');
    if (dot_pos == std::string::npos || dot_pos == 0 || dot_pos + 10 != ts_str.size())
        return false;

    uint64_t secs64 = 0;
    uint32_t nsecs32 = 0;
    for (size_t i = 0; i < ts_str.size(); ++i) {
        if (i == dot_pos)
            continue;
        if (ts_str[i] < '0' || ts_str[i] > '9')
            return false;

        const uint32_t digit = static_cast<uint32_t>(ts_str[i] - '0');
        if (i < dot_pos) {
            if (secs64 > (std::numeric_limits<uint32_t>::max() - digit) / 10)
                return false;
            secs64 = secs64 * 10 + digit;
        } else {
            nsecs32 = nsecs32 * 10 + digit;
        }
    }

    secs  = static_cast<uint32_t>(secs64);
    nsecs = nsecs32;
    return true;
}

/**
 * Feed undistorted PCD files and their keyframe poses into PointCloudProc,
 * bypassing the ROS bag pipeline.
 *
 * TUM format (per line):  timestamp tx ty tz qx qy qz qw [extra cols ignored]
 * PCD naming convention:  <pcd_dir>/<timestamp>.pcd
 *   where <timestamp> is the exact string from column 0 of the TUM file.
 *
 * PointCloudProc parameters (iter_num, output_folder, …) are loaded before
 * this function is called. main_app also applies CLI overrides and assigns
 * sequence length / decimation from the TUM file.
 *
 * processSequence() is called at the end when all entries have been fed but
 * clouds_to_process was not yet reached; if it was already triggered from
 * inside putMessage the program will have exited before reaching that call.
 */
inline void runFromPCDFolder(std::shared_ptr<PointCloudProc> proc,
                              const std::string& pcd_dir,
                              const std::string& tum_file) {
    using namespace srrg2_core;
    using namespace srrg2_core_ros;

    std::ifstream ifs(tum_file);
    if (!ifs.is_open()) {
        std::cerr << "runFromPCDFolder | cannot open TUM file: " << tum_file << std::endl;
        return;
    }

    int seq = 0;
    std::string line;
    while (std::getline(ifs, line)) {
        if (line.empty() || line[0] == '#')
            continue;

        // Parse: timestamp tx ty tz qx qy qz qw [extra columns ignored]
        std::istringstream ss(line);
        std::string ts_str;
        double tx, ty, tz, qx, qy, qz, qw;
        ss >> ts_str >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        if (ss.fail()) {
            std::cerr << "runFromPCDFolder | skipping malformed line: " << line << std::endl;
            continue;
        }

        uint32_t secs = 0, nsecs = 0;
        if (!parseTimestamp(ts_str, secs, nsecs)) {
            std::cerr << "runFromPCDFolder | skipping malformed timestamp: "
                      << ts_str << std::endl;
            continue;
        }

        // Load PCD.  The file uses x y z intensity [normal_x normal_y normal_z curvature];
        // loading as PointXYZI is sufficient since mad_ba recomputes normals internally.
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        const std::string pcd_path = pcd_dir + "/" + ts_str + ".pcd";
        if (pcl::io::loadPCDFile(pcd_path, *pcl_cloud) < 0) {
            std::cerr << "runFromPCDFolder | cannot load PCD: " << pcd_path << std::endl;
            continue;
        }

        // Build nav_msgs::Odometry from TUM entry
        nav_msgs::Odometry ros_odom;
        ros_odom.header.stamp      = ros::Time(secs, nsecs);
        ros_odom.header.frame_id   = "map";
        ros_odom.header.seq        = seq;
        ros_odom.child_frame_id    = "pcd_frame";
        ros_odom.pose.pose.position.x    = tx;
        ros_odom.pose.pose.position.y    = ty;
        ros_odom.pose.pose.position.z    = tz;
        ros_odom.pose.pose.orientation.x = qx;
        ros_odom.pose.pose.orientation.y = qy;
        ros_odom.pose.pose.orientation.z = qz;
        ros_odom.pose.pose.orientation.w = qw;

        // Build sensor_msgs::PointCloud2 from PCL cloud
        sensor_msgs::PointCloud2 ros_cloud;
        pcl::toROSMsg(*pcl_cloud, ros_cloud);
        ros_cloud.header.stamp    = ros::Time(secs, nsecs);
        ros_cloud.header.frame_id = "map";
        ros_cloud.header.seq      = seq++;

        // Convert to srrg2 message types via the ROS converter
        auto odom_srrg = std::dynamic_pointer_cast<OdometryMessage>(
            Converter::convert(boost::make_shared<nav_msgs::Odometry>(ros_odom)));
        auto cloud_srrg = std::dynamic_pointer_cast<PointCloud2Message>(
            Converter::convert(boost::make_shared<sensor_msgs::PointCloud2>(ros_cloud)));

        if (!odom_srrg || !cloud_srrg) {
            std::cerr << "runFromPCDFolder | message conversion failed at ts "
                      << ts_str << std::endl;
            continue;
        }

        // Feed odometry first: PointCloudProc matches cloud[i] to pose[i] by index
        proc->putMessage(odom_srrg);
        proc->putMessage(cloud_srrg);
    }

    // Reached only when all TUM entries were fed but processSequence was not
    // triggered from inside putMessage.
    proc->processSequence();
}

} // namespace mad_ba
