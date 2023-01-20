// Copyright (c) 2022 Leo Drive Teknoloji A.Ş.

#include <memory>
#include <pc_utm_to_mgrs_converter/pc_utm_to_mgrs_converter.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <GeographicLib/MGRS.hpp>
#include <rclcpp/rclcpp.hpp>
#include <GeographicLib/UTMUPS.hpp>

PcUtm2MgrsConverter::PcUtm2MgrsConverter()
        : Node("PcUtm2MgrsConverter") {

    //read pcd file
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/melike/projects/autoware_data/gebze_pospac_map/pointcloud_map.pcd", *inputCloud) == -1) {
        PCL_ERROR("Couldn't read file test_pcd.pcd  \n");
    }


    GNSSStat gnss_stat_utm;
    GNSSStat gnss_stat_mgrs;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_mgrs(new pcl::PointCloud<pcl::PointXYZ>);
    for (long unsigned int i = 0; i < inputCloud->points.size(); i++) {
        pcl::PointXYZ point = inputCloud->points[i];
        gnss_stat_utm.x = point.x + 699078.50;
        gnss_stat_utm.y = point.y + 4521081.55;
        gnss_stat_utm.z = point.z;
        gnss_stat_utm.coordinate_system = CoordinateSystem::UTM;
        gnss_stat_utm.zone = 35;
        gnss_stat_utm.northup = true;

        GeographicLib::UTMUPS::Reverse(gnss_stat_utm.zone,gnss_stat_utm.northup, gnss_stat_utm.x, gnss_stat_utm.y, gnss_stat_utm.latitude, gnss_stat_utm.longitude);

//        std::cout<< "latitude: " << gnss_stat_utm.latitude << " longitude: " << gnss_stat_utm.longitude << std::endl;
//        gnss_stat_utm.mgrs_zone = "35T";
        const rclcpp::Logger & logger = this->get_logger();
        gnss_stat_mgrs = convertUTM2MGRS(gnss_stat_utm,MGRSPrecision::_100MICRO_METER,logger);
        pcl::PointXYZ point_mgrs;
        point_mgrs.x = gnss_stat_mgrs.x;
        point_mgrs.y = gnss_stat_mgrs.y;
        point_mgrs.z = gnss_stat_mgrs.z;
        cloud_mgrs->points.push_back(point_mgrs);
    }

    //save pcd file
     pcl::io::savePCDFileASCII("/home/melike/projects/autoware_data/gebze_pospac_map/pointcloud_map_mgrs.pcd", *cloud_mgrs);
     std::cerr << "Saved " << cloud_mgrs->points.size() << " data points to test_pcd_mgrs.pcd." << std::endl;



}

PcUtm2MgrsConverter::GNSSStat PcUtm2MgrsConverter::convertUTM2MGRS(PcUtm2MgrsConverter::GNSSStat gnss_stat_utm, const PcUtm2MgrsConverter::MGRSPrecision precision, const rclcpp::Logger &logger){

        constexpr int GZD_ID_size = 5;  // size of header like "53SPU"

    PcUtm2MgrsConverter::GNSSStat mgrs = gnss_stat_utm;
    mgrs.coordinate_system = PcUtm2MgrsConverter::CoordinateSystem::MGRS;
    try {
        std::string mgrs_code;
        GeographicLib::MGRS::Forward(
                gnss_stat_utm.zone, gnss_stat_utm.northup, gnss_stat_utm.x, gnss_stat_utm.y, gnss_stat_utm.latitude, static_cast<int>(precision), mgrs_code);
        mgrs.mgrs_zone = std::string(mgrs_code.substr(0, GZD_ID_size));
        mgrs.x = std::stod(mgrs_code.substr(GZD_ID_size, static_cast<int>(precision))) *
                 std::pow(
                         10, static_cast<int>(PcUtm2MgrsConverter::MGRSPrecision::_1_METER) -
                             static_cast<int>(precision));  // set unit as [m]
        mgrs.y = std::stod(mgrs_code.substr(
                GZD_ID_size + static_cast<int>(precision), static_cast<int>(precision))) *
                 std::pow(
                         10, static_cast<int>(PcUtm2MgrsConverter::MGRSPrecision::_1_METER) -
                             static_cast<int>(precision));  // set unit as [m]
        mgrs.z = gnss_stat_utm.z;                                 // set unit as [m]
    } catch (const GeographicLib::GeographicErr & err) {
        RCLCPP_ERROR_STREAM(logger, "Failed to convert from UTM to MGRS" << err.what());
    }
    return mgrs;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PcUtm2MgrsConverter>());
    rclcpp::shutdown();
    return 0;
}
