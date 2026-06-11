#include "stm32f4xx_hal.h"

class N10PLiDAR {
public:
    static constexpr uint16_t buf_size = 256;
    static constexpr uint32_t timestamp_offset = 27; // +2.68ms for one pack
    static constexpr uint8_t command_len = 188;
    #pragma pack(1)
    // 112 len
    struct Point {
        uint16_t distance;
        uint8_t intensity;
    };
    struct SendFrame {
        uint8_t head1 = 0xA5;
        uint8_t head2 = 0x5A;
        uint32_t timestamp;
        uint16_t speed;
        uint16_t start_angle;
        uint16_t end_angle;
        Point echo[32];
        // Point echo2[16];
        uint8_t crc;
    };
    struct RawFrame {
        uint8_t head1 = 0xA5;
        uint8_t head2 = 0x5A;
        uint32_t timestamp;
        uint8_t data[105];
        uint8_t crc;
    };
    #pragma pack()

    uint8_t command[command_len];
    uint8_t buf1[16];
    uint8_t buf2[buf_size];
    uint8_t *read_ptr = buf1;
    uint8_t *process_ptr = buf2;
    UART_HandleTypeDef *uart_ptr;

    size_t packlose = 0;
    
    N10PLiDAR(UART_HandleTypeDef* uartx): uart_ptr(uartx){
        command[0] = 0xa5;
        command[1] = 0x5a;
        command[2] = 0x55;
        command[command_len - 4] = 0x01;
        command[command_len - 2] = 0xfa;
        command[command_len - 1] = 0xfb;
    }
    HAL_StatusTypeDef start() {
        command[command_len - 3] = 0x01;
        return HAL_UART_Transmit(uart_ptr, command, command_len, 500);
    }
    HAL_StatusTypeDef stop() {
        command[command_len - 3] = 0x00;
        return HAL_UART_Transmit(uart_ptr, command, command_len, 500);
    }
    uint8_t* swap_read_process_buf() {
            uint8_t* tmp_ptr;
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
    uint8_t add_crc(uint8_t* start, size_t len) {
        uint8_t crc = 0xA5 + 0x5a;
        for (size_t i = 0; i < len; i++) {
            crc += start[i];
        }
        return crc;
    }
};
