#pragma once
#include <mutex>
#include <deque>
#include <nlohmann/json.hpp>

#include "structure.hpp"
#include "serial_boost.hpp"

class DataLoader {
public:
    std::deque<IMUMsg> imu_q;
    std::deque<LiDARMsg> lidar_q;
    std::condition_variable cv;
    std::mutex mtx;
    std::atomic<bool> running = false;
private:
    
    uint8_t byte;
    Serial port;
    LiDARMsg lidar_msg;
    IMUMsg imu_msg;
    
    double d_radian;  // angular gain per point. about 0.68 deg.
    float d_time;  // cuz the relative time is s, and saved in double
    float accumulated_d_time;
    double rot_freq;
    double scan_freq;
    size_t frame_size;
    double nearst_sq;
    double max_dist_sq;
    size_t lidar_cnt = 0;

    double accel_lsb;
    double gyro_lsb;
    uint64_t timestamp_to_ns;

    
    
public:
    DataLoader(const nlohmann::json loader_config)
    : port(Serial(loader_config["serial_cfg"])) {
        std::string accel_fs = loader_config["imu_cfg"]["accel_fs"];
        accel_lsb = loader_config["imu_cfg"]["accel_lsb"][accel_fs].get<double>();

        std::string gyro_fs = loader_config["imu_cfg"]["gyro_fs"];
        gyro_lsb = loader_config["imu_cfg"]["gyro_lsb"][gyro_fs].get<double>();

        timestamp_to_ns = loader_config["timestamp_to_ns"].get<uint64_t>();

        rot_freq = loader_config["lidar_cfg"]["rot_freq"].get<double>();
        scan_freq = loader_config["lidar_cfg"]["scan_freq"].get<double>();
        nearst_sq = loader_config["lidar_cfg"]["nearst_sq"].get<double>();
        max_dist_sq = loader_config["lidar_cfg"]["max_dist_sq"].get<double>();
        d_radian = 2 * std::numbers::pi * rot_freq / scan_freq;
        d_time = (1. / scan_freq);
        frame_size = (size_t)(scan_freq / rot_freq * 1.10);
    }

    void start_receiving(bool lidar_enb=false, bool imu_enb=false) {
        if (!(lidar_enb | imu_enb)) {
            throw std::runtime_error("both lidar and imu are disabled in data loader.");
        }
        // start this thread.
        running = true;

        while (running) {
            // the serial is block reading process.
            // imu
            port.read((char*)&byte, 1);
            if (byte == 0xAA) {
                port.read((char*)&byte, 1);
                if (byte == 0x55) {
                    #ifdef DEBUG
                    std::cout << "call imu" << std::endl;
                    #endif
                    imu_msg_cbk();
                }
            }
            // lidar
            else if (byte == 0xA5) {
                port.read((char*)&byte, 1);
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

 
    void stop_receiving() {
        running = false;
    }

private:
    #pragma pack(1)
    struct IMUFrame {
	uint8_t head_1 = 0xAA;
	uint8_t head_2 = 0x55;
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
        IMUFrame imu_buf;
        port.read((char*)&imu_buf.timestamp, 17);
        if (imu_buf.crc != xor_crc((uint8_t*)&imu_buf.timestamp, 16)) {
            #ifdef DEBUG
            std::cout << "crc fail" << std::endl;
            #endif
            return;
        }
        imu_msg.timestamp = (uint64_t)imu_buf.timestamp * timestamp_to_ns;    // to ns
        if (!imu_q.empty() && imu_msg.timestamp <= imu_q.back().timestamp) {
            throw std::range_error("the timestamp of current imu smaller than last.");
        }
        imu_msg.angular_velocity = {deg2rad((float)imu_buf.gyro_xyz[0] / gyro_lsb),
                                    deg2rad((float)imu_buf.gyro_xyz[1] / gyro_lsb),
                                    deg2rad((float)imu_buf.gyro_xyz[2] / gyro_lsb)};
        imu_msg.linear_acceleration = {9.81 * (float)imu_buf.accel_xyz[0] / accel_lsb,
                                       9.81 * (float)imu_buf.accel_xyz[1] / accel_lsb,
                                       9.81 * (float)imu_buf.accel_xyz[2] / accel_lsb};
        // lock, avoid multi thread race.
        std::unique_lock<std::mutex> lock(mtx);
        imu_q.push_back(imu_msg);
        lock.unlock();
        cv.notify_all();
    }

    // remove zero point and NAN point.
    // Check valid here.
    void lidar_msg_cbk() {
        LiDARFrame lidar_buf;
        port.read((char*)&lidar_buf.timestamp, 107);
        if (lidar_buf.crc != xor_crc((uint8_t*)&lidar_buf.timestamp, 106)){
            #ifdef DEBUG
            std::cout << "lidar crc failed" << std::endl;
            #endif
            return;
        }

        if (lidar_msg.cloud_ptr == nullptr) {
            lidar_cnt = 0;
            lidar_msg.starttime = lidar_buf.timestamp * timestamp_to_ns;
            accumulated_d_time = 0;
            // if (!lidar_q.empty() && lidar_msg.starttime <= lidar_q.back().endtime) {
            //     std::cout << lidar_q.back().endtime << " " << lidar_msg.starttime << std::endl;
            //     throw std::range_error("current lidar start time smaller than last end");
            // }
            lidar_msg.cloud_ptr = pcl::PointCloud<PointType>::Ptr (new pcl::PointCloud<PointType>);
            lidar_msg.cloud_ptr->reserve(frame_size);
        }
        double start_radian = deg2rad((double)lidar_buf.start_angle / 100);
        double end_radian = deg2rad((double)lidar_buf.end_angle / 100);
        
        float segment_time_bias = (float)(lidar_buf.timestamp * timestamp_to_ns - lidar_msg.starttime) / 1e9;
        
        for (size_t i = 0; i < 16; i++) {
            PointType p;
            accumulated_d_time = segment_time_bias + d_time * i;
            p.curvature = accumulated_d_time;
            double dist = (double)(lidar_buf.echo[i].distance) / 1000;
            double radian = start_radian + i * d_radian;
            p.x = dist * std::cos(radian);
            p.y = -1 * dist * std::sin(radian);
            p.z = 0;
            lidar_cnt += 1;  // yes we consider all points, no matter valid or not
            double norm_sq = p.x * p.x + p.y * p.y + p.z * p.z;
            if ( norm_sq > nearst_sq && norm_sq < max_dist_sq) {
                lidar_msg.cloud_ptr->push_back(p);
            }
        }
        // get one full frame.
        if (lidar_cnt >= scan_freq / rot_freq) {
            // std::cout << "cnt" << lidar_cnt << std::endl;
            lidar_msg.endtime = lidar_msg.starttime + (uint64_t)(accumulated_d_time * 1e9);
            std::unique_lock<std::mutex> lock(mtx);
            lidar_q.push_back(lidar_msg);
            lock.unlock();
            cv.notify_all();
            lidar_msg.cloud_ptr = nullptr;
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

};
