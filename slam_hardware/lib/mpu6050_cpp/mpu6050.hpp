#pragma once
#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "stm32f4xx_hal_i2c.h"

namespace MPU6050 {
// address
constexpr uint8_t ADDR            = 0x68 << 1;
    namespace Reg {
        // configure
        constexpr uint8_t SMPLRT_DIV   = 0x19;
        constexpr uint8_t CONFIG       = 0x1A;
        constexpr uint8_t GYRO_CONFIG  = 0x1B;
        constexpr uint8_t ACCEL_CONFIG = 0x1C;

        // interrupt
        constexpr uint8_t INT_PIN_CFG   = 0x37;
        constexpr uint8_t INT_ENABLE   = 0x38;
        constexpr uint8_t INT_STATUS   = 0x3A;

        // data
        constexpr uint8_t ACCEL_XOUT_H = 0x3B;
        constexpr uint8_t GYRO_XOUT_H  = 0x43;

        // power
        constexpr uint8_t PWR_MGMT_1   = 0x6B;
        constexpr uint8_t WHO_AM_I     = 0x75;
    }

    constexpr float acc_lsb[4] = {16384.0, 8192.0, 4096.0, 2048.0};
    constexpr float gyro_lsb[4] = {131.0, 65.5, 32.8, 16.4};
    
    enum class AccFullScale:uint8_t {
        PM2GPS, PM4GPS, PM8GPS, PM16GPS
    };
    enum class GyroFullScale:uint8_t {
        PM250DPS, PM500DPS, PM1000DPS, PM2000DPS
    };
    
    class MPU6050 {
    public:
        static constexpr uint8_t read_all_size = 14;
        #pragma pack(1)
        struct SendFrame {
            uint8_t head_1 = 0xAA;
            uint8_t head_2 = 0x55;
            uint32_t timestamp;
            int16_t accel_xyz[3];
            int16_t gyro_xyz[3];
            uint8_t crc;
        };
        struct ReadBuf {
            uint32_t timestamp;
            uint8_t data[14];
            bool ready = true;
        };
        #pragma pack()

        ReadBuf buf1;
        ReadBuf buf2;
        ReadBuf *read_ptr = &buf1;
        ReadBuf *process_ptr = &buf2;
        I2C_HandleTypeDef* i2c_ptr;
        
        MPU6050(I2C_HandleTypeDef *i2cx):i2c_ptr(i2cx) {}
        HAL_StatusTypeDef init(AccFullScale afs, GyroFullScale gfs, bool ready_interrupt) {
            uint8_t buf;
            HAL_I2C_Mem_Read(i2c_ptr, ADDR, Reg::WHO_AM_I, 1, &buf, 1, 100);
            if (buf != 0x68) {
                return HAL_ERROR;
            }
            HAL_Delay(100); 
            uint8_t pwr_cfg = 0x01; // set bit 6 = 0 for activate, bit 0 =1 use gyro clock
            uint8_t dlpf_cfg = 0x01; // only level 1
            uint8_t accel_cfg = static_cast<uint8_t>(afs) << 3; 
            uint8_t gyro_cfg = static_cast<uint8_t>(gfs) << 3;
            uint8_t int_cfg = 0x01 << 7; // logic low, push pull
            uint8_t int_enable;
            if (ready_interrupt) {
                int_enable = 0x01;  // only data ready
            }
            else {
                int_enable = 0x00;
            }
            uint8_t smplrt_div = 0x04; // div 5, 200Hz

            cur_acc_lsb = acc_lsb[static_cast<uint8_t>(afs)];
            cur_gyro_lsb = gyro_lsb[static_cast<uint8_t>(gfs)];
        
            HAL_I2C_Mem_Write(i2c_ptr, ADDR, Reg::PWR_MGMT_1, 1, &pwr_cfg, 1,100);
            HAL_I2C_Mem_Write(i2c_ptr, ADDR, Reg::SMPLRT_DIV, 1, &smplrt_div, 1, 100);
            HAL_I2C_Mem_Write(i2c_ptr, ADDR, Reg::CONFIG, 1, &dlpf_cfg, 1, 100);
            HAL_I2C_Mem_Write(i2c_ptr, ADDR, Reg::ACCEL_CONFIG, 1, &accel_cfg, 1, 100);
            HAL_I2C_Mem_Write(i2c_ptr, ADDR, Reg::GYRO_CONFIG, 1, &gyro_cfg, 1, 100);
            HAL_I2C_Mem_Write(i2c_ptr, ADDR, Reg::INT_PIN_CFG, 1, &int_cfg, 1, 100);
            HAL_I2C_Mem_Write(i2c_ptr, ADDR, Reg::INT_ENABLE, 1, &int_enable, 1, 100); // last step enable
            return HAL_OK; 
        }
        /* This function update the double buf,
         * return the current process buf */
        ReadBuf* swap_read_process_buf() {
            ReadBuf* tmp_ptr;
            tmp_ptr = read_ptr;
            read_ptr = process_ptr;
            process_ptr = tmp_ptr;
            return process_ptr;
        }

        uint8_t xor_crc(uint8_t* start, size_t len) {
            uint8_t crc = 0x00;
            for (size_t i = 0; i< len; i++) {
                crc ^= start[i];
            }
            return crc;
        }
        
    private:
        float gravity = 9.81; //m/s2
        float cur_acc_lsb;
        float cur_gyro_lsb;
    };

}

