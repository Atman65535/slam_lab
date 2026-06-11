#pragma once
#include <cstdint>

#include <Eigen/Core>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_types.h>

pcl::PointXYZI p;

p.

struct CustomPnt {
    float offset_time;
    float x;
    float y;
    float z;
    uint8_t reflectivity;
    uint8_t tag;
    uint8_t line;
};
struct CloudMsg {
    // Header header  # the ros standard message header
    float starttime; // use nano second
    float endtime;
    uint32_t point_num;
    uint8_t lidar_id;
    uint8_t rsvd[3];  // reserved message
    std::vector<CustomPnt> points;
};
struct IMUMsg {
    float time;
    Eigen::Vector3d angular_velocity;
    Eigen::Matrix3d angular_velocity_covariance;

    Eigen::Vector3d linear_acceleration;
    Eigen::Matrix3d linear_acceleration_covariance;
};

