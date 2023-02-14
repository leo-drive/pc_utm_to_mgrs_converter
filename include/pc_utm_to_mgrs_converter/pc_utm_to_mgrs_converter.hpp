// Copyright (c) 2022 Leo Drive Teknoloji A.Ş.

#ifndef PC_UTM_TO_MGRS_CONVERTER__PC_UTM_TO_MGRS_CONVERTER_HPP_
#define PC_UTM_TO_MGRS_CONVERTER__PC_UTM_TO_MGRS_CONVERTER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class PcUtm2MgrsConverter : public rclcpp::Node {
public:
    PcUtm2MgrsConverter();

    enum class MGRSPrecision {
        _1_METER = 5,
        _100MICRO_METER = 9,
    };
    enum class CoordinateSystem {
        UTM = 0,
        MGRS = 1,
    };
    struct GNSSStat
    {

        GNSSStat()
                : coordinate_system(CoordinateSystem::MGRS),
                  northup(true),
                  zone(0),
                  mgrs_zone(""),
                  x(0),
                  y(0),
                  z(0),
                  latitude(0),
                  longitude(0),
                  altitude(0)
        {
        }

        CoordinateSystem coordinate_system;
        bool northup;
        int zone;
        std::string mgrs_zone;
        double x;
        double y;
        double z;
        double latitude;
        double longitude;
        double altitude;
    };

    GNSSStat convertUTM2MGRS(GNSSStat gnss_stat_utm, const MGRSPrecision precision, const rclcpp::Logger &logger);
};

#endif  // PC_UTM_TO_MGRS_CONVERTER__PC_UTM_TO_MGRS_CONVERTER_HPP_