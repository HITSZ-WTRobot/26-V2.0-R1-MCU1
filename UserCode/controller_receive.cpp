
#include "controller_receive.hpp"
#include "interboard_comm.hpp"
#include "vision_lower_receive.hpp"
#include "watchdog.hpp"
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
static uint8_t lr_uart2_rx_byte = 0;
uint32_t decodesuccess_count = 0;             // 成功解码次数
bool decode_enable = false;                   // 解码使能标志
bool is_controller_connected = true;          // 遥控器连接状态
JOYSTICK_MODE_E joystick_mode = CHASSIS_MODE; // 遥控器模式，默认底盘模式

static service::Watchdog controller_watchdog;

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

constexpr float kAutoAlignStepM = 0.15f;
constexpr float kInterboardRetreatDistanceM = 0.20f;
static bool g_step_cmd_active = false;
static bool g_interboard_retreat_active = false;
static bool g_interboard_retreat_last_req = false;

static void ApplyButtonStepAlignFallback(void) {
  const uint32_t event = button_status;
  float step_x = 0.0f;
  float step_y = 0.0f;

  // Keep current step target until chassis reports trajectory finished.
  if (g_step_cmd_active) {
    chassis_control_mode = POS_Control;
    if (chassis_ && chassis_->isTrajectoryFinished()) {
      target_x = 0.0f;
      target_y = 0.0f;
      target_yaw = 0.0f;
      g_step_cmd_active = false;
    }
    return;
  }

  // keypad 1/3/5/7 -> 前/左/右/后，每次按下步进0.05m。
  // 按键索引与编号关系：button1->bit0, button3->bit2, button5->bit4, button7->bit6.
  if (event & (1U << 1)) {
    step_x = kAutoAlignStepM;
  } else if (event & (1U << 3)) {
    step_y = kAutoAlignStepM;
  } else if (event & (1U << 5)) {
    step_y = -kAutoAlignStepM;
  } else if (event & (1U << 7)) {
    step_x = -kAutoAlignStepM;
  }

  target_x = step_x;
  target_y = step_y;
  target_yaw = 0.0f;
  g_step_cmd_active = (step_x != 0.0f || step_y != 0.0f);
  chassis_control_mode = POS_Control;
}

static void ApplyVisionAutoAlign(void) {
  chassis_v.vx = 0.0f;
  chassis_v.vy = 0.0f;
  chassis_v.wz = 0.0f;

  const int has_apriltag = (lr_apriltag_count > 0);
  const int has_detect = (lr_detect_count > 0);
  if (!has_apriltag && !has_detect) {
    // 无视觉数据时：进入按键步进位置环测试模式。
    ApplyButtonStepAlignFallback();
    return;
  }

  g_step_cmd_active = false;

  LR_DataPacket src = {0};
  if (has_apriltag) {
    const int latest_idx = (lr_apriltag_write_idx + LR_DATA_MAX_NUM - 1) % LR_DATA_MAX_NUM;
    src = lr_apriltag_buffer[latest_idx];
  } else {
    const int latest_idx = (lr_detect_write_idx + LR_DATA_MAX_NUM - 1) % LR_DATA_MAX_NUM;
    src = lr_detect_buffer[latest_idx];
  }

  const LR_DataPacket body_pkt = LR_Convert_Packet_CameraToArm(&src);

  target_x = body_pkt.x;
  target_y = body_pkt.y;
  target_yaw = body_pkt.yaw;
  chassis_control_mode = POS_Control;
}

static void AbortAutoAlignAndStop(void) {
  g_step_cmd_active = false;
  g_interboard_retreat_active = false;
  target_x = 0.0f;
  target_y = 0.0f;
  target_yaw = 0.0f;
  chassis_control_mode = VEL_Control;
  chassis_v.vx = 0.0f;
  chassis_v.vy = 0.0f;
  chassis_v.wz = 0.0f;
}

static void ApplyInterboardRetreatByPosition(void) {
  const bool retreat_req = InterboardComm_IsRetreatRequested();

  // 上升沿触发一次固定距离后退，距离定义在 MCU1 侧。
  if (retreat_req && !g_interboard_retreat_last_req) {
    g_interboard_retreat_active = true;
    g_step_cmd_active = false;
    target_x = -kInterboardRetreatDistanceM;
    target_y = 0.0f;
    target_yaw = 0.0f;
  }
  g_interboard_retreat_last_req = retreat_req;

  if (g_interboard_retreat_active) {
    chassis_control_mode = POS_Control;
    if (chassis_ && chassis_->isTrajectoryFinished()) {
      g_interboard_retreat_active = false;
      target_x = 0.0f;
      target_y = 0.0f;
      target_yaw = 0.0f;
      chassis_control_mode = VEL_Control;
      chassis_v.vx = 0.0f;
      chassis_v.vy = 0.0f;
      chassis_v.wz = 0.0f;
    }
  }
}

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
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_dma_buf, RX_DMA_BUF_SIZE);
  __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);

  HAL_UART_Receive_IT(&huart2, &lr_uart2_rx_byte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    ProcessRxBytes(rx_dma_buf, RX_DMA_BUF_SIZE);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_dma_buf, RX_DMA_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  } else if (huart->Instance == USART2) {
    LR_Parse_And_Store(lr_uart2_rx_byte);
    HAL_UART_Receive_IT(&huart2, &lr_uart2_rx_byte, 1);
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
          (JOYSTICK_MODE_E)(((int)joystick_mode + 1) % 4); // 四种模式循环切换
    }
    switch (joystick_mode) {
    case CHASSIS_MODE:
      chassis_control_mode = VEL_Control;
      chassis_v.vx = __JOYSTICK2VEL__(LY_T);
      chassis_v.vy = __JOYSTICK2VEL__(-1.0f * LX_T);
      chassis_v.wz = __JOYSTICK2WZ__(-1.0f * RX_T);
      break;
    case CLAMP_MODE:
      chassis_control_mode = VEL_Control;
      chassis_v.vx = 0;
      chassis_v.vy = 0;
      chassis_v.wz = 0;

      break;
    case AUTO_ALIGN_MODE:
      if (button_status & (1U << 8)) {
        AbortAutoAlignAndStop();
      } else if (InterboardComm_IsRetreatRequested()) {
        ApplyInterboardRetreatByPosition();
      } else if (g_interboard_retreat_active) {
        ApplyInterboardRetreatByPosition();
      } else {
        ApplyVisionAutoAlign();
      }
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
  controller_watchdog.feed(100); // 收到有效遥控器数据喂狗（100ms 断连窗口）
  return;
}
// 类似于实现一种软看门狗，让遥控器断联后底盘不会疯跑
void TIM10_Callback(TIM_HandleTypeDef *htim) {
  if (!controller_watchdog.isFed()) {
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
