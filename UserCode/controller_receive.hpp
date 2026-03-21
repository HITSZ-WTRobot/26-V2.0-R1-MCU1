#include "usart.h"
#include "cmsis_os2.h"
#include "string.h"
#include "stdint.h"
#include "stdbool.h"
#include "chassis.hpp"
#include "eventflags.hpp"

#define MAX_JOYSTICK                   2000.0f // 遥控器摇杆数据最大值
#define __JOYSTICK2VEL__(__JOYSTICK__) (__JOYSTICK__ * MAX_VEL / MAX_JOYSTICK)
#define __JOYSTICK2WZ__(__JOYSTICK__)  (__JOYSTICK__ * MAX_WZ / MAX_JOYSTICK)

uint8_t CRC8(uint8_t* buffer, uint8_t len);
// void    Decode(void* argument);
void Buffer_Decode(void);
void controller_task(void* argument);

typedef enum
{
    CHASSIS_MODE = 0,
    CLAMP_MODE   = 1,
    ARM_MODE     = 2,
    AUTO_ALIGN_MODE = 3
} JOYSTICK_MODE_E;

// USART2 视觉串口最小诊断计数器（可在调试器Watch窗口直接观察）
extern volatile uint32_t vision_uart2_diag_rx_irq_cnt;
extern volatile uint32_t vision_uart2_diag_rx_byte_cnt;
extern volatile uint32_t vision_uart2_diag_rearm_fail_cnt;
extern volatile uint32_t vision_uart2_diag_err_cnt;
extern volatile uint32_t vision_uart2_diag_last_err_code;

void Controller_receiver_Init(void);
void TIM10_Callback(TIM_HandleTypeDef* htim);