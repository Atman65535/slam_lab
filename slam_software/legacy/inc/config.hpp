#pragma once
#include <cstdint>

namespace cfg {
    static constexpr uint32_t imu_data_length = 16; // except head
    static constexpr uint32_t lidar_data_length = 106; // except head
    static constexpr double gyro_lsb = 65.5;
    static constexpr double accel_lsb = 16384;
    static constexpr double gravity = 9.81;

    static constexpr uint32_t kf_init_frame = 1;



    static constexpr double process_noise[] = {0.01, 0.01, 0.01, 0.01, 0.01, 0.01};
    static constexpr double measurement_noise[] = {0.01, 0.01, 0.01};
}
