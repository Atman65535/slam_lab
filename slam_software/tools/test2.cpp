#include <iostream>
#include <Eigen/Core>
#include <thread>
#include <chrono>

#include "view.hpp"
using namespace std::chrono_literals;

int main() {
    Eigen::Matrix3d m;
    m.setRandom();

    Eigen::Vector3d v;
    v.setRandom();
    Eigen::Vector3d e = m * v;
    std::cout << e.block(0, 0, 2, 1) << std::endl;

    auto viewer = Viewer();
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    for (int i = 0; i < 20; i++) {
        Eigen::Vector3d v(i, i*i, 0.6*i);
        viewer.update_posture(R, v);
        viewer.spin_once(10);
    }
    while (1) {
        viewer.spin_once(10);
    }
}
