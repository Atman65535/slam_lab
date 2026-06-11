#pragma once
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <numbers>
#include <string>
#include <vector>
#include <cstdint>
#include <fcntl.h>
#include <termios.h>
#include <deque>

#include <Eigen/Core>
#include <Eigen/Geometry>

// #include "serial.hpp"
#include "serial_boost.hpp"
#include "config.hpp"

class DataLoader {
    // this class provide a blocking data collection pipe.
    // use multithread for read and process is essential
    // coordinate: right hand, front right up
public:
    // the definition of point cloud follows Livox LiDAR
    // while the one for IMU follows ROS standard message.
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
        Eigen::Quaternion<double> orientation;
        Eigen::Matrix3d orientation_covariance;

        Eigen::Vector3d angular_velocity;
        Eigen::Matrix3d angular_velocity_covariance;

        Eigen::Vector3d linear_acceleration;
        Eigen::Matrix3d linear_acceleration_covariance;
    };
        
    std::deque<CloudMsg> lidar_q;
    std::deque<IMUMsg> imu_q;
    
public:
    DataLoader(const std::string serial_port, const int baudrate)
    : ser(Serial(serial_port, baudrate)) {}

    void receive_data(bool lidar=true, bool imu=true) {
        uint8_t byte;
        while (true) {
            ser.read((char*)&byte, 1);
            // imu
            if (byte == 0xA1) {
                ser.read((char*)&byte, 1);
                if (byte == 0x1A) {
                    #ifdef DEBUG
                    std::cout << "call imu" << std::endl;
                    #endif
                    imu_msg_cbk();
                }
            }
            // lidar
            else if (byte == 0xA5) {
                ser.read((char*)&byte, 1);
                if (byte == 0x5A) {
                    #ifdef DEBUG
                    std::cout << "call lidar" << std::endl;
                    #endif
                    lidar_msg_cbk();
                }
            }
            else continue;
        }
    }
    
private:
    // definition of received data.
    // refer to lower-level machine 
    #pragma pack(1)
    struct IMUFrame {
	uint8_t head_1 = 0xA1;
	uint8_t head_2 = 0x1A;
	uint32_t timestamp;
	int16_t accel_xyz[3];
	int16_t gyro_xyz[3];
	uint8_t crc;
    };
    struct Point {
	uint16_t distance;
	uint8_t intensity;
    };
    struct LiDARFrame {
	uint8_t head1 = 0xA5;
	uint8_t head2 = 0x5A;
	uint32_t timestamp;
	uint16_t speed;
	uint16_t start_angle;
	uint16_t end_angle;
	Point echo[32];
	uint8_t crc;
    };
    #pragma pack()
    
    void imu_msg_cbk() {
        // serial read, return number of bytes.
        // f**k boost.asio.read_some, really it read some thing, usually less than desired.
        size_t num = ser.read((char*)&imu_buf.timestamp, cfg::imu_data_length + 1);
        if (num != cfg::imu_data_length + 1) {
            std::cout << "imu read num expected " << cfg::imu_data_length + 1 << " but " <<num << std::endl;
            return;
        }
        
        if (imu_buf.crc != xor_crc(reinterpret_cast<uint8_t*>(&imu_buf.timestamp),
                                   cfg::imu_data_length)) {
            std::cout << "crc fail" << std::endl;
            return;
        }
        
        IMUMsg msg;
        // unit of original timestamp is 1/10 ms
        msg.time = (float)imu_buf.timestamp / 1e4;
        msg.angular_velocity.x() = static_cast<double>(imu_buf.gyro_xyz[0]) / cfg::gyro_lsb;
        msg.angular_velocity.y() = static_cast<double>(imu_buf.gyro_xyz[1]) / cfg::gyro_lsb;
        msg.angular_velocity.z() = static_cast<double>(imu_buf.gyro_xyz[2]) / cfg::gyro_lsb;

        msg.linear_acceleration.x() = static_cast<double>(imu_buf.accel_xyz[0]) / cfg::accel_lsb * cfg::gravity;
        msg.linear_acceleration.y() = static_cast<double>(imu_buf.accel_xyz[1]) / cfg::accel_lsb * cfg::gravity;
        msg.linear_acceleration.z() = static_cast<double>(imu_buf.accel_xyz[2]) / cfg::accel_lsb * cfg::gravity;

        imu_q.push_back(msg);
    }
    
    void lidar_msg_cbk() {
        uint32_t num = ser.read((char*)&lidar_buf.timestamp, cfg::lidar_data_length + 1);
        if (num != cfg::lidar_data_length + 1) {
            printf("lidar expected %d but %u", cfg::lidar_data_length + 1, num);
            return;
        }
        uint8_t crc = xor_crc((uint8_t*)(&lidar_buf.timestamp), 106);
        if (lidar_buf.crc != crc) {
            printf("%02X fail %02X\n",crc, lidar_buf.crc); //<< " lidar crc fail " << lidar_buf.crc << std::endl;
            return;
        }


        if (frame_angle == 0) {
            frame = new CloudMsg;
            frame->starttime = (float)lidar_buf.timestamp / 1e4;
            frame->point_num = 0;
            frame->lidar_id = 1;
        }
        
        double start_angle = deg2rad(static_cast<double>(lidar_buf.start_angle) / 100);
        double end_angle = deg2rad(static_cast<double>(lidar_buf.end_angle) / 100);
        if (end_angle < start_angle) {
            end_angle += 2 * std::numbers::pi;
        }

        double d_angle = (end_angle - start_angle) / 15;
        float segment_time = (float)lidar_buf.timestamp / 1e4;
        for (uint32_t i = 0; i < 16; i ++) {
            CustomPnt p;
            p.line = 1;
            p.offset_time = segment_time - frame->starttime;
            p.reflectivity = lidar_buf.echo[i].intensity;
            double dist = static_cast<double>(lidar_buf.echo[i].distance);
            double angle = start_angle + i * d_angle;
            // coordinate sys transformation. from front x, right y lidar to front, left, up
            p.x = dist * std::cos(angle);
            p.y = -1 * dist * std::sin(angle);
            p.z = 0;

            frame->points.push_back(p);
            frame->point_num += 1;
        }
        
        
        frame_angle += (end_angle - start_angle);
        if (frame_angle - 2 * std::numbers::pi > -1e-5) {
            frame->endtime = segment_time;
            lidar_q.push_back(*frame);
            frame_angle = 0;
            frame = nullptr;
        }
    }

    uint8_t xor_crc(uint8_t* start, uint32_t size) {
        uint8_t crc = 0x00;
        for (uint32_t i = 0; i < size; i ++) {
            crc ^= start[i];
        }
        return crc;
    }
    double deg2rad(double deg) {
        return deg / 180.0 * std::numbers::pi;
    }
private:
    Serial ser;

    IMUFrame imu_buf;
    LiDARFrame lidar_buf;
    CloudMsg* frame;
    double frame_angle = 0;
};
