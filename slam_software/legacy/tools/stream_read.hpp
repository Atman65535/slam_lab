#include <cmath>
#include <Eigen/Core>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <ostream>
#include <pcl/impl/point_types.hpp>
#include <string>
#include <memory>
#include <system_error>
#include <vector>
#include <termios.h>
#include <fcntl.h>
// #include "serial.hpp"
#include "serial_boost.hpp"

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class N10PProtocol {
public:
    struct RawPoint {
        uint16_t echo1;
        uint8_t intensity1;
        uint16_t echo2;
        uint8_t intensity2;
        float radian;

        RawPoint(uint16_t echo1, uint8_t intensity1,
                 uint16_t echo2, uint8_t intensity2,
                 float radian): echo1(echo1), intensity1(intensity1),
        echo2(echo2), intensity2(intensity2), radian(radian){}
        RawPoint() = default;
    };

    
    N10PProtocol(const std::string serial_name="/dev/ttyACM0", int speed = 460800)
    :serail_name(serial_name), ser(Serial(serial_name, speed)){
    // fd = open(serial_name.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    // if (fd < 0) {
    //     throw "Error opening serial";
    // }
    // else {
    //     std::cout << "Open fd" << fd << std::endl;
    // }
    // struct termios tty;
    // if (tcgetattr(fd, &tty) != 0) {
        
    // }
    // cfsetospeed(&tty, speed);
    // cfsetispeed(&tty, speed);

    // tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;  // CSIZE the character size mask
    // tty.c_cflag &= ~IGNBRK; // disable break processing
    
    // tty.c_lflag = 0;
    
    // tty.c_oflag = 0;
    
    // tty.c_cc[VMIN] = 0; // read doesn't block
    // tty.c_cc[VTIME] = 5; // 0.5 second read timeout

    // tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    // tty.c_cflag |= (CLOCAL | CREAD); // ignore moden controls, enable reading;
    // tty.c_cflag &= ~(PARENB | PARODD); // shut off parity
    // tty.c_cflag &= ~CSTOPB;
    // tty.c_cflag &= ~CRTSCTS;
    // if (tcsetattr(fd, TCSANOW, &tty) == 0) {
    //     std::cout << "set " + serial_name << "succey" << std::endl;
    // }
}

    void read_point_cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud) {
        uint8_t temp_byte;
        uint8_t length = 0x00;
        float start_angle, end_angle, speed;
        while (true) {
            if (ser.read((char*)&temp_byte, sizeof(uint8_t)) && temp_byte == 0xA5) {
                if (ser.read((char*) &temp_byte, 1) && temp_byte == 0x5A) {
                    // Read logic
                    ser.read((char*) readbuf+2, 106);
                    speed = ((uint16_t)readbuf[3] << 8) | readbuf[4];
                    start_angle = (float)(((uint16_t)readbuf[5] << 8) | readbuf[6]);
                    start_angle = start_angle / 100;
                    for (int i = 0; i < 16; i ++) {
                        uint16_t dist1 = ((uint16_t)readbuf[7 + 6 * i + 0] << 8) | readbuf[7 + 6 * i + 1];
                        uint8_t intensity1 = readbuf[7 + 6 * i + 2];
                        uint16_t dist2 = ((uint16_t)readbuf[7 + 6*i + 3] << 8) | readbuf[7 + 6*i + 4];
                        uint8_t intensity2 = readbuf[7 + 6*i + 5];

                        // coordinate transformation. The custom coordinate
                        float angle = (start_angle + i * 0.68) * 3.14159 / 180;
                        point_cloud->push_back(pcl::PointXYZI(std::cos(angle) * dist1,
                            -std::sin(angle) * dist1,
                            1,
                            intensity1));                        
                        point_cloud->push_back(pcl::PointXYZI(std::cos(angle) * dist2,
                            -std::sin(angle) * dist2,
                            1,
                            intensity2));
                    }
                    break;
                }
            }
        }
    }
private:
    Serial ser;
    int fd;
    std::string serail_name;
    uint8_t readbuf[108];
// commands
private:
    
};
