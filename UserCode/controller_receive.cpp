
#include "controller_receive.hpp"
#include <string.h>

#define RAWDATA_SIZE 14   // 每一帧大小
#define BUFFER_SIZE 14    // DMA接收缓冲区大小
#define RX_DMA_BUF_SIZE 64
#define FRAME_HEADER 0xAA // 帧头
#define FRAME_TAIL 0xBB   // 帧尾
#define BUTTON_NUM 9      // 按钮个数

uint32_t bbb = 0;
// 测试用计数器

uint32_t decode_count = 0;
uint8_t buffer[14];
static uint8_t rx_dma_buf[RX_DMA_BUF_SIZE];
static uint8_t rx_frame_buf[RAWDATA_SIZE];
static uint8_t rx_frame_fill = 0;
static volatile uint32_t controller_last_rx_tick = 0;
static const uint32_t CONTROLLER_TIMEOUT_MS = 150;
uint32_t decodesuccess_count = 0;             // 成功解码次数
bool decode_enable = false;                   // 解码使能标志
bool is_controller_connected = true;          // 遥控器连接状态
JOYSTICK_MODE_E joystick_mode = CHASSIS_MODE; // 遥控器模式，默认底盘模式

int16_t LX_T; // 左摇杆x值数据转换后数据
int16_t LY_T; // 左摇杆y值数据转换后数据
int16_t RX_T; // 右摇杆x值数据转换后数据
int16_t RY_T; // 右摇杆y值数据转换后数据

uint16_t LX; // 左摇杆x值数据原始数据
uint16_t LY; // 左摇杆y值数据原始数据
uint16_t RX; // 右摇杆x值数据原始数据
uint16_t RY; // 右摇杆y值数据原始数据

bool joystick_button_L; // 左摇杆按键状态
bool joystick_button_R; // 右摇杆按键状态

bool button[9];      // 矩阵键盘按钮
bool button_last[9]; // 矩阵键盘按钮上次状态
uint32_t button_status;
uint8_t crc = 0; // CRC校验值

osThreadId_t controllerHandle;
const osThreadAttr_t controller_attributes = {
    .name = "controller",
    .stack_size = 128 * 8,
    .priority = (osPriority_t)osPriorityHigh,
};

static void ProcessRxBytes(const uint8_t *data, uint16_t size) {
  for (uint16_t i = 0; i < size; ++i) {
    const uint8_t byte = data[i];

    // 等待帧头，避免固定分包错位导致长期解码失败。
    if (rx_frame_fill == 0U && byte != FRAME_HEADER) {
      continue;
    }

    rx_frame_buf[rx_frame_fill++] = byte;

    if (rx_frame_fill >= RAWDATA_SIZE) {
      decode_count++;
      memcpy(buffer, rx_frame_buf, RAWDATA_SIZE);
      Buffer_Decode();
      rx_frame_fill = 0U;
    }
  }
}

void Controller_receiver_Init(void) {
  for (int i = 0; i < BUTTON_NUM; i++) {
    button[i] = 0;
    button_last[i] = 0;
  }
  osThreadNew(controller_task, NULL, &controller_attributes);
  controller_last_rx_tick = HAL_GetTick();
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_dma_buf, RX_DMA_BUF_SIZE);
  __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    ProcessRxBytes(rx_dma_buf, RX_DMA_BUF_SIZE);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_dma_buf, RX_DMA_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart->Instance == USART1) {
    ProcessRxBytes(rx_dma_buf, Size);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_dma_buf, RX_DMA_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  }
}

void controller_task(void *argument) {
  for (;;) {
    if ((osEventFlagsWait(flags_id, 0x00000010U, osFlagsWaitAny, 0) &
         0xFF000010U) == 0x00000010U) {
      joystick_mode =
          (JOYSTICK_MODE_E)(((int)joystick_mode + 1) % 3); // 三种模式循环切换
    }
    switch (joystick_mode) {
    case CHASSIS_MODE:
      chassis_v.vx = __JOYSTICK2VEL__(LY_T);
      chassis_v.vy = __JOYSTICK2VEL__(-1.0f * LX_T);
      chassis_v.wz = __JOYSTICK2WZ__(-1.0f * RX_T);
      break;
    case CLAMP_MODE:
      chassis_v.vx = 0;
      chassis_v.vy = 0;
      chassis_v.wz = 0;

      break;
    default:
      break;
    }

    osDelay(10);
  }
}
void Buffer_Decode(void) {
  if (buffer[0] != FRAME_HEADER)
    return;
  if (buffer[13] != FRAME_TAIL)
    return;
  crc = CRC8(buffer, 11);
  if (crc != buffer[12])
    return;

  // 如果能执行到这说明包头包尾正确，crc校验通过，可以解析数据

  LX = (uint16_t)(buffer[1] << 8 | buffer[2]);
  LY = (uint16_t)(buffer[3] << 8 | buffer[4]);
  RX = (uint16_t)(buffer[5] << 8 | buffer[6]);
  RY = (uint16_t)(buffer[7] << 8 | buffer[8]);
  joystick_button_L = buffer[9] & 0x01;
  joystick_button_R = (buffer[9] >> 1) & 0x01;
  for (int i = 0; i < BUTTON_NUM; i++) {
    button_last[i] = button[i];
  }
  button[0] = buffer[11] & 0x01;
  button[1] = (buffer[11] >> 1) & 0x01;
  button[2] = (buffer[11] >> 2) & 0x01;
  button[3] = (buffer[11] >> 3) & 0x01;
  button[4] = (buffer[11] >> 4) & 0x01;
  button[5] = (buffer[11] >> 5) & 0x01;
  button[6] = (buffer[11] >> 6) & 0x01;
  button[7] = (buffer[11] >> 7) & 0x01;
  button[8] = buffer[10] & 0x01;
  button_status = 0;
  for (int i = 0; i < BUTTON_NUM; i++) {
    if (button[i] == 0 && button_last[i] == 1) {
      button_status |= (1 << i);
    }
  }
  bbb = osEventFlagsSet(flags_id, button_status);
  LX_T = (int16_t)LX;
  LY_T = (int16_t)LY;
  RX_T = (int16_t)RX;
  RY_T = (int16_t)RY;
  decodesuccess_count++;
  controller_last_rx_tick = HAL_GetTick();
  return;
}
// 类似于实现一种软看门狗，让遥控器断联后底盘不会疯跑
void TIM10_Callback(TIM_HandleTypeDef *htim) {
  (void)htim;
  const uint32_t now = HAL_GetTick();
  if ((now - controller_last_rx_tick) > CONTROLLER_TIMEOUT_MS) {
    is_controller_connected = false;
    chassis_v.vx = 0;
    chassis_v.vy = 0;
    chassis_v.wz = 0;
  } else {
    is_controller_connected = true;
  }
}

uint8_t CRC8(uint8_t *buffer, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= buffer[i + 1];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0x07;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}
