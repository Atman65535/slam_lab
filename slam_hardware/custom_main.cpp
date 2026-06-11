#include "stm32f4xx_hal.h"
#include "gpio.h"
#include "i2c.h"
#include "dma.h"
#include "usart.h"
#include "tim.h"
#include "usart.h"

#include "queue.hpp"
#include "mpu6050.hpp"
#include "n10p_lidar.hpp"

extern "C" {
    #include "ssd1306.h"
    #include "ssd1306.h"
    #include <string.h>
    #include <mpu6050.h>
    #include <stdio.h>
}

#define TIME_DEBUG

#ifdef TIME_DEBUG 
    uint32_t busy_time = 0;
    uint32_t start_busy = 0;
    float efficiency = 0;

    uint32_t imu_send_pack = 0;
    uint32_t lidar_send_pack = 0;
    uint32_t current_time;
    float imu_freq;
    float lidar_freq;
#endif

MPU6050::MPU6050 ins_handler = MPU6050::MPU6050(&hi2c2);
N10PLiDAR lidar_handler(&huart2);

Queue<MPU6050::MPU6050::SendFrame, 5> ins_half_buf;
Queue<MPU6050::MPU6050::SendFrame,5> ins_send_buf;
MPU6050::MPU6050::SendFrame ins_send;

Queue<N10PLiDAR::RawFrame, 5> lidar_half_buf;
Queue<N10PLiDAR::SendFrame, 5> lidar_send_buf;
N10PLiDAR::SendFrame lidar_send;

void SystemClock_Config(void);

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
void half_to_send(void) {
    if (!ins_half_buf.is_empty()) {
        MPU6050::MPU6050::SendFrame raw = ins_half_buf.out_queue();
        raw.crc = ins_handler.xor_crc(reinterpret_cast<uint8_t*>(&raw.timestamp), 16);
        ins_send_buf.in_queue(raw);
    }
    else if (!lidar_half_buf.is_empty()) {
        N10PLiDAR::RawFrame raw = lidar_half_buf.out_queue();
        uint8_t check_crc = lidar_handler.add_crc(raw.data, 107 - 2);
        if (check_crc == raw.crc) {
            N10PLiDAR::SendFrame send;
            send.timestamp = raw.timestamp - N10PLiDAR::timestamp_offset;
            send.speed = (raw.data[1] << 8) | raw.data[2];
            send.start_angle = (raw.data[3] << 8) | raw.data[4];
            send.end_angle = (raw.data[103] << 8) | raw.data[104];
            for (uint8_t i = 0; i < 16; i++) {
                send.echo[i].distance = (raw.data[5 + 6*i] << 8) | raw.data[5 + 6*i + 1];
                send.echo [i].intensity = raw.data[5 + 6*i + 2];

                send.echo[i + 16].distance = (raw.data[5 + 6*i + 3] << 8) | raw.data[5 + 6*i + 4];
                // note that this is for preveting break
                send.echo[i + 16].intensity = raw.data[5 + 6*i + 5];
            }
            send.crc = lidar_handler.xor_crc((uint8_t*)&send.timestamp, sizeof(N10PLiDAR::SendFrame)-3);
            lidar_send_buf.in_queue(send);
        }
        else
            lidar_handler.packlose += 1;
    }
}

int main(void) {
    // assert_param(true==false);
    HAL_Init();
    SystemClock_Config();
    
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_I2C2_Init();
    MX_TIM2_Init();

    lidar_handler.stop();
    
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
    if (ssd1306_Init(&hi2c1) != 0)
        Error_Handler();
    
    HAL_StatusTypeDef ins_status = ins_handler.init(MPU6050::AccFullScale::PM2GPS, MPU6050::GyroFullScale::PM500DPS, true);
    assert_param(ins_status == HAL_OK);
    
    __HAL_UART_CLEAR_OREFLAG(&huart2);
    HAL_StatusTypeDef ret = HAL_UARTEx_ReceiveToIdle_DMA(lidar_handler.uart_ptr, lidar_handler.read_ptr, lidar_handler.buf_size);
    lidar_handler.start();
    HAL_UART_StateTypeDef state; 
    
    #ifdef TIME_DEBUG
    start_busy = htim2.Instance->CNT;
    HAL_UART_StateTypeDef last_state=HAL_UART_STATE_RESET;
    #endif
    
    while (1) {
        #ifdef TIME_DEBUG
            efficiency = busy_time / (float)htim2.Instance->CNT;
            current_time = htim2.Instance->CNT;
            imu_freq = (float)imu_send_pack / ((float)current_time/10);
            lidar_freq = (float)lidar_send_pack / ((float)current_time/10);
        #endif
        
        state = HAL_UART_GetState(&huart1);               
        switch (state) {
            case HAL_UART_STATE_READY:
                #ifdef TIME_DEBUG
                if (last_state == HAL_UART_STATE_BUSY_TX)
                    busy_time += htim2.Instance->CNT - start_busy;
                #endif
                if (!ins_send_buf.is_empty()) {
                    ins_send = ins_send_buf.out_queue();
                    HAL_UART_Transmit_DMA(&huart1,reinterpret_cast<uint8_t*>(&ins_send), sizeof(MPU6050::MPU6050::SendFrame));
                    #ifdef TIME_DEBUG
                    start_busy = htim2.Instance->CNT;
                    imu_send_pack +=1 ;
                    #endif
                }
                else if (!lidar_send_buf.is_empty()) {
                    lidar_send = lidar_send_buf.out_queue();
                    HAL_UART_Transmit_DMA(&huart1, reinterpret_cast<uint8_t*>(&lidar_send), sizeof(N10PLiDAR::SendFrame));
                    #ifdef TIME_DEBUG
                    start_busy = htim2.Instance->CNT;
                    lidar_send_pack += 1;
                    #endif
                }
                else 
                    half_to_send();
                break;
            case HAL_UART_STATE_BUSY_TX:
                half_to_send();
                break;
            default:
                assert_param(true == false);
                break;
        }
        #ifdef TIME_DEBUG
        last_state = state;
        #endif
    }
}

/* INS callback */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        ins_handler.read_ptr->timestamp = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);
        HAL_I2C_Mem_Read_DMA(ins_handler.i2c_ptr, MPU6050::ADDR, MPU6050::Reg::ACCEL_XOUT_H, 1,
                             ins_handler.read_ptr->data, 14); // 0-5 acc, 6-7 tmp, 8-13 gyro
    }
}

// just data alignment to send queue
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c == &hi2c2) {
        // swap read and process. We only use process ptr
        MPU6050::MPU6050::ReadBuf* process_buf = ins_handler.swap_read_process_buf();
        
        MPU6050::MPU6050::SendFrame frame;
        frame.timestamp = ins_handler.process_ptr->timestamp;
        for (uint8_t idx=0; idx < 3; idx ++) {
            frame.accel_xyz[idx] = ((int16_t)process_buf->data[idx * 2] << 8) | process_buf->data[idx*2 + 1];
            frame.gyro_xyz[idx] = ((int16_t)process_buf->data[idx * 2 + 8] << 8) | process_buf->data[idx*2 + 8 + 1];
        }
        ins_half_buf.in_queue(frame);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart == lidar_handler.uart_ptr && Size > 0) {
        if (Size >= 108 && Size <= lidar_handler.buf_size) {
            
            uint32_t timestamp = htim2.Instance->CNT;
            uint8_t* process_buf = lidar_handler.swap_read_process_buf();
            HAL_UARTEx_ReceiveToIdle_DMA(lidar_handler.uart_ptr, lidar_handler.read_ptr, lidar_handler.buf_size);
            uint16_t idx = 0;
            bool find = false;
            while (idx+107 < Size) {
                if (process_buf[idx] == 0xA5) {
                    if (process_buf[idx + 1] == 0x5A) {
                        find = true;
                        break;
                    }
                }
                idx ++;
            }
            if (find) {
                N10PLiDAR::RawFrame frame;
                frame.timestamp = timestamp;
                memcpy(frame.data, process_buf + idx + 2, 106);
                lidar_half_buf.in_queue(frame);
            }
        }
        else {
            HAL_UARTEx_ReceiveToIdle_DMA(lidar_handler.uart_ptr, lidar_handler.read_ptr, lidar_handler.buf_size);
        }
    }
}



/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType =
      RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name  of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
     ssd1306_Fill(Black);
     char* filebuf = reinterpret_cast<char*>(file);
     ssd1306_SetCursor(0, 0);
     if (strlen(filebuf) > 18)
         filebuf = filebuf + strlen(filebuf) - 18;
     ssd1306_WriteString(filebuf, Font_7x10, White);
     char buf[64];
     sprintf(buf, "line : %d", line);
     ssd1306_SetCursor(0, 10);
     ssd1306_WriteString(buf, Font_7x10, White);
     ssd1306_UpdateScreen(&hi2c1);
    
}
#endif /* USE_FULL_ASSERT */
