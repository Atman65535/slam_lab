#pragma once
#include <deque>
#include <iostream>

#include <Eigen/Core>
#include <stdexcept>

#include "config.hpp"
#include "dataloader.hpp"

class StatusManifold {
public:
    Eigen::Matrix<double, 3, 3> R;
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Vector3d bw;
    Eigen::Vector3d ba;
    Eigen::Vector3d g;

    StatusManifold() {
        g.x() = 0;
        g.y() = 0;
        g.z() = cfg::gravity;
    }
    
};

class ErrorTangent {
public:
    Eigen::Vector3d dtheta;
    Eigen::Vector3d dp;
    Eigen::Vector3d dv;
    Eigen::Vector3d dbw;
    Eigen::Vector3d dba;
    Eigen::Vector3d dg;

};

class Measurement {
public:
    DataLoader::CloudMsg* cloud;
    std::deque<DataLoader::IMUMsg> imu_deq;
    float starttime;
    float endtime;
    bool imu_ok;
    bool lidar_ok;
    bool empty;
    void clear() {
        imu_ok = false;
        lidar_ok = false;
        empty = true;
        
        delete cloud;
        cloud = nullptr;
        imu_deq.clear();
        starttime = 0;
        endtime = 0;
    }
};



class ESKF {
public:
    float time;
    StatusManifold nomial_status;
    ErrorTangent error;
    
    ESKF();
    void init(Measurement& mea) {
        time = mea.endtime;
    }
    void imu_forward(DataLoader::IMUMsg& msg) {
        double delta_t = msg.time - time;
        time = msg.time;
        
    }
    
private:
    Eigen::Matrix3d cov_update_theta;
    Eigen::Matrix3d cov_theta;

    Eigen::Matrix3d hat(Eigen::Vector3d v) {
        Eigen::Matrix3d m;
        m << 0, -v.z(), v.y(),
             v.z(), 0, -v.x(),
             -v.y(), v.x(), 0;
        return m;
    }

    Eigen::Vector3d vee(Eigen::Matrix3d m) {
        if (m != -1 * m.transpose()) {
            throw std::runtime_error("not asymmetric");            
        }
        Eigen::Vector3d v {m(2, 1), m(0, 2), m(1, 0)};
        return v;
    }

    Eigen::Matrix3d exp(Eigen::Vector3d lie_alg) {
        double theta = std::sqrt(lie_alg.x() * lie_alg.x() + lie_alg.y() * lie_alg.y() + lie_alg.z() * lie_alg.z());
        Eigen::Vector3d dir = lie_alg / theta;
        Eigen::Matrix3d m = hat(lie_alg);
        return Eigen::Matrix3d::Identity() + m * std::sin(theta) + m * m * (1 - std::cos(theta));
    }
};

