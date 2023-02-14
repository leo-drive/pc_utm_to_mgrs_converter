// Copyright (c) 2022 Leo Drive Teknoloji A.Ş.

#include <memory>
#include <pc_utm_to_mgrs_converter/pc_utm_to_mgrs_converter.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <GeographicLib/MGRS.hpp>
#include <rclcpp/rclcpp.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <rclcpp/logging.hpp>
#include <GeographicLib/Geoid.hpp>
PcUtm2MgrsConverter::PcUtm2MgrsConverter()
        : Node("PcUtm2MgrsConverter") {

    const auto input_file_path_ = declare_parameter("input_file_path", "");
    const auto output_file_path_ = declare_parameter("output_file_path", "");
    RCLCPP_INFO(get_logger(), "input_file_path: %s", input_file_path_.c_str());
    RCLCPP_INFO(get_logger(), "output_file_path: %s", output_file_path_.c_str());

    const double map_origin_northing = declare_parameter("Northing", 0.0);
    const double map_origin_easting = declare_parameter("Easting", 0.0);
    const double map_origin_height = declare_parameter("ElipsoidHeight", 0.0);
    RCLCPP_INFO(get_logger(), "map_origin_northing: %f", map_origin_northing);
    RCLCPP_INFO(get_logger(), "map_origin_easting: %f", map_origin_easting);
    RCLCPP_INFO(get_logger(), "map_origin_height: %f", map_origin_height);

    //read pcd file
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_file_path_, *inputCloud) == -1) {
        PCL_ERROR("Couldn't read file test_pcd.pcd  \n");
    }

    double counter = 0;
    GNSSStat gnss_stat_utm;
    GNSSStat gnss_stat_mgrs;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_mgrs(new pcl::PointCloud<pcl::PointXYZ>);
    for (long unsigned int i = 0; i < inputCloud->points.size(); i++) {

        pcl::PointXYZ point = inputCloud->points[i];

        // convert local UTM to global UTM coordinates
        gnss_stat_utm.x = point.x + map_origin_easting;
        gnss_stat_utm.y = point.y + map_origin_northing;
        gnss_stat_utm.z = point.z + map_origin_height;
        gnss_stat_utm.coordinate_system = CoordinateSystem::UTM;
        gnss_stat_utm.zone = 35;
        gnss_stat_utm.northup = true;

        // convert latitude and longitude from UTM
        GeographicLib::UTMUPS::Reverse(gnss_stat_utm.zone,gnss_stat_utm.northup, gnss_stat_utm.x, gnss_stat_utm.y, gnss_stat_utm.latitude, gnss_stat_utm.longitude);

        gnss_stat_utm.altitude = gnss_stat_utm.z;

        // convert height from ellipsoid to orthometric
        double OrthometricHeight{0.0};
        try {
            GeographicLib::Geoid egm2008("egm2008-1");
            OrthometricHeight = egm2008.ConvertHeight(
                    gnss_stat_utm.latitude, gnss_stat_utm.longitude, gnss_stat_utm.altitude,
                    GeographicLib::Geoid::ELLIPSOIDTOGEOID);
        } catch (const GeographicLib::GeographicErr & err) {
            RCLCPP_ERROR(this->get_logger(), "Failed to convert Height from Ellipsoid to Orthometric");
        }

        // convert global UTM to MGRS
        const rclcpp::Logger & logger = this->get_logger();
        gnss_stat_mgrs = convertUTM2MGRS(gnss_stat_utm,MGRSPrecision::_100MICRO_METER,logger);

        //create new pointcloud with mgrs coordinates
        pcl::PointXYZ point_mgrs;
        point_mgrs.x = gnss_stat_mgrs.x;
        point_mgrs.y = gnss_stat_mgrs.y;
        point_mgrs.z = OrthometricHeight;
        cloud_mgrs->width = inputCloud->width;
        cloud_mgrs->height = inputCloud->height;
        cloud_mgrs->is_dense = inputCloud->is_dense;
        cloud_mgrs->points.push_back(std::move(point_mgrs));

        if (counter == 1000000){
            RCLCPP_INFO(this->get_logger(), "continue.");
            counter = 0;
        }
        counter++;
    }
        //save pcd file
        pcl::io::savePCDFileASCII(output_file_path_, *cloud_mgrs);
        std::cerr << "Saved " << cloud_mgrs->points.size() << " data points saved to " << output_file_path_ << std::endl;


}

// convert UTM to MGRS function
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
