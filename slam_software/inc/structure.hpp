#pragma once
#include <cstdint>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

typedef pcl::PointXYZINormal PointType;

typedef Eigen::Vector3d V3D;
typedef Eigen::Vector3f V3F;
typedef Eigen::Matrix3d M3D;

struct IMUMsg {
    uint64_t timestamp;    //us
    Eigen::Vector3d angular_velocity;    // rad/s
    Eigen::Vector3d linear_acceleration;    // m/s^2

    IMUMsg() {
        timestamp = 0;
        angular_velocity = {0, 0, 0};
        linear_acceleration = {0, 0, 0};
    }
    
    IMUMsg(const IMUMsg& i) {
        timestamp = i.timestamp;
        angular_velocity = i.angular_velocity;
        linear_acceleration = i.linear_acceleration;
    }
};

struct LiDARMsg {
    uint64_t starttime;    // us, the global time of the first pnt;
    uint64_t endtime;
    pcl::PointCloud<PointType>::Ptr cloud_ptr;    // here we store points. The relative time is stored in curvature.
};

struct FeaturePoint {
    
};

struct StatusTangent {
    V3D dtheta;    // \in so(3) 
    V3D dp;       // position, p = \hat p + dp
    V3D dv;       // velocity
    V3D dbw;      // bw = \hat bw + dbw
    V3D dba;
    V3D dg;
    StatusTangent() {
        dtheta = Eigen::Vector3d(0, 0, 0);
        dp = Eigen::Vector3d(0, 0, 0);
        dv = Eigen::Vector3d(0, 0, 0);
        dbw = Eigen::Vector3d(0, 0, 0);
        dba = Eigen::Vector3d(0, 0, 0);
        dg = Eigen::Vector3d(0, 0, 0);
    }
    StatusTangent(Eigen::Vector<double, 18>& v) {
        dtheta = v.block(0, 0, 3, 1);
        dp = v.block(3, 0, 3, 1);
        dv = v.block(6, 0, 3, 1);
        dbw = v.block(9, 0, 3, 1);
        dba = v.block(12, 0, 3, 1);
        dg = v.block(15, 0, 3, 1);
    }
};

struct StatusManifold {
    Eigen::Matrix3d R;    // ego -> world
    Eigen::Vector3d p;    // absolute position
    Eigen::Vector3d v;    // velocity
    V3D bw;    // bias of gyro. \omega - bw = \hat omega
    V3D ba;    // bias of acceleration. Definition as above
    V3D g;     // gravity, global
    StatusManifold() {
        R = Eigen::Matrix3d::Identity();
        p = Eigen::Vector3d(0, 0, 0);
        v = Eigen::Vector3d(0, 0, 0);
        bw = Eigen::Vector3d(0, 0, 0);
        ba = Eigen::Vector3d(0, 0, 0);
        g = Eigen::Vector3d(0, 0, -9.81);
    }
};

struct PlanarFeature {
    double residual;
    V3D normal;
    V3D p_loc;  // point local_frame
    
};

struct EdgeFeature {
    V3D residual;
    V3D direction;
    V3D p_loc;
};
