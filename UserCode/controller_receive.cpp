
#include "controller_receive.hpp"
#include "chassis.hpp"
#include "interboard_comm.hpp"
#include "vision_lower_receive.hpp"
#include "watchdog.hpp"
#include <cmath>
#include <cstdint>
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
static uint8_t interboard_uart4_rx_byte = 0;

volatile uint32_t vision_uart2_diag_rx_irq_cnt = 0;
volatile uint32_t vision_uart2_diag_rx_byte_cnt = 0;
volatile uint32_t vision_uart2_diag_rearm_fail_cnt = 0;
volatile uint32_t vision_uart2_diag_err_cnt = 0;
volatile uint32_t vision_uart2_diag_last_err_code = 0;
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

float x = 0.0f,
      y = 0.0f,
      yaw = 0.0f; // 来自视觉的目标位置和朝向数据（单位米/度）

uint8_t auto_mode = 0; // 自动模式选择，0=没有视觉信息模式，1=自动对齐

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

// 滤波参数和状态
constexpr float kAutoAlignStepM = 0.15f;
constexpr float kInterboardRetreatDistanceM = 0.20f;
constexpr float kMmToMScale = 0.001f;
constexpr float kVisionLpfAlpha = 0.85f;
constexpr float kVisionMaxStepPerCycleM = 0.03f;
constexpr float kVisionMaxStepPerCycleDeg = 12.0f;
constexpr float kVisionPosDeadbandM = 0.03f;
constexpr float kVisionYawLpfAlpha = 0.60f;
constexpr float kVisionYawDeadbandDeg = 0.8f;
constexpr float kAutoAlignYawLockDeg = 2.0f;
constexpr uint32_t kVisionLostCycleThreshold = 10U; // controller_task 10ms周期，约100ms
static bool g_step_cmd_active = false;
static bool g_interboard_retreat_active = false;
static bool g_interboard_retreat_last_req = false;
static bool g_emergency_hold_active = false;
static bool g_wait_interboard_target = false;
static bool g_vision_filter_inited = false;
static float g_target_x_filtered = 0.0f;
static float g_target_y_filtered = 0.0f;
static uint32_t g_vision_last_update_seq = 0U;
static uint32_t g_vision_stale_cycles = 0U;
static bool g_auto_align_translation_phase = false;

static inline float ClampFloat(float value, float min_value, float max_value) {
  return value < min_value ? min_value : (value > max_value ? max_value : value);
}

static void ResetVisionTargetFilter(void) {
  g_vision_filter_inited = false;
  g_target_x_filtered = 0.0f;
  g_target_y_filtered = 0.0f;
}

static void ResetVisionSignalState(void) {
  g_vision_last_update_seq = 0U;
  g_vision_stale_cycles = 0U;
}

static void ResetAutoAlignPhaseState(void) {
  g_auto_align_translation_phase = false;
}

static void ApplyVisionTargetFilter(float raw_x, float raw_y, float *out_x, float *out_y) {
  if (!out_x || !out_y) {
    return;
  }

  if (!g_vision_filter_inited) {
    g_target_x_filtered = raw_x;
    g_target_y_filtered = raw_y;
    g_vision_filter_inited = true;
  }

  const float dx = ClampFloat(raw_x - g_target_x_filtered,
                              -kVisionMaxStepPerCycleM,
                              kVisionMaxStepPerCycleM);
  const float dy = ClampFloat(raw_y - g_target_y_filtered,
                              -kVisionMaxStepPerCycleM,
                              kVisionMaxStepPerCycleM);

  g_target_x_filtered += kVisionLpfAlpha * dx;
  g_target_y_filtered += kVisionLpfAlpha * dy;

  if (fabsf(g_target_x_filtered) < kVisionPosDeadbandM) {
    g_target_x_filtered = 0.0f;
  }
  if (fabsf(g_target_y_filtered) < kVisionPosDeadbandM) {
    g_target_y_filtered = 0.0f;
  }

  *out_x = g_target_x_filtered;
  *out_y = g_target_y_filtered;
}

static void ApplyYawTargetFilter(float raw_yaw, float *yaw)
{
  if (!yaw)
  {
    return;
  }

  const float dyaw = ClampFloat(raw_yaw - *yaw,
                              -kVisionMaxStepPerCycleDeg,
                              kVisionMaxStepPerCycleDeg);
  float yaw_output = *yaw + kVisionYawLpfAlpha * dyaw;

  if (fabsf(yaw_output) < kVisionYawDeadbandDeg) {
    yaw_output = 0.0f;
  }

  *yaw = yaw_output;
} 

static void ApplyButtonStepAlignFallback(void) {
  ResetVisionTargetFilter();

  const uint32_t event = button_status;
  float step_x = 0.0f;
  float step_y = 0.0f;
  float step_yaw = 0.0f;

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
    step_yaw = 5.0f; // 同时增加一个小的旋转，帮助打破纯平移可能遇到的局部最优问题。
  } else if (event & (1U << 3)) {
    step_y = kAutoAlignStepM;
    step_yaw = -5.0f;
  } else if (event & (1U << 5)) {
    step_y = -kAutoAlignStepM;
    step_yaw = 5.0f;
  } else if (event & (1U << 7)) {
    step_x = -kAutoAlignStepM;
    step_yaw = -5.0f;
  }

  target_x = step_x;
  target_y = step_y;
  target_yaw = step_yaw;
  g_step_cmd_active = (step_x != 0.0f || step_y != 0.0f);
  chassis_control_mode = POS_Control;
}

static bool ApplyVisionAutoAlign(void) {
  chassis_v.vx = 0.0f;
  chassis_v.vy = 0.0f;
  chassis_v.wz = 0.0f;

  const uint32_t apriltag_seq = lr_apriltag_update_seq;
  const uint32_t detect_seq = lr_detect_update_seq;
  const uint32_t latest_seq = (apriltag_seq >= detect_seq) ? apriltag_seq : detect_seq;
  const int has_apriltag = (apriltag_seq > 0U);
  const int has_detect = (detect_seq > 0U);
  if (!has_apriltag && !has_detect) {
    auto_mode = 0;
    return false;
  }

  if (g_vision_last_update_seq == 0U || latest_seq != g_vision_last_update_seq) {
    g_vision_last_update_seq = latest_seq;
    g_vision_stale_cycles = 0U;
  } else {
    if (g_vision_stale_cycles < 0xFFFFFFFFU) {
      g_vision_stale_cycles++;
    }
    if (g_vision_stale_cycles > kVisionLostCycleThreshold) {
      auto_mode = 0;
      return false;
    }
  }

  const int use_apriltag = has_apriltag && (!has_detect || (apriltag_seq >= detect_seq));
  auto_mode = use_apriltag ? 2 : 1; // 2=apriltag, 1=detect
  g_step_cmd_active = false;

  LR_DataPacket src = {0};
  if (use_apriltag) {
    const int latest_idx = (lr_apriltag_write_idx + LR_DATA_MAX_NUM - 1) % LR_DATA_MAX_NUM;
    src = lr_apriltag_buffer[latest_idx];
  } else {
    const int latest_idx = (lr_detect_write_idx + LR_DATA_MAX_NUM - 1) % LR_DATA_MAX_NUM;
    src = lr_detect_buffer[latest_idx];
  }

  float cam_x = src.x;
  float cam_y = src.y;
  float cam_z = src.z;
  float cam_yaw = src.yaw;
  //直接走向目标点，暂不区分旋转和位移阶段，开环控制，后续可增加分阶段闭环控制
  LR_Compute_Target(cam_x, cam_y, cam_z, cam_yaw, &target_x, &target_y, &target_yaw);

}

static void AbortAutoAlignAndStop(void) {
  ResetVisionTargetFilter();
  ResetVisionSignalState();
  ResetAutoAlignPhaseState();

  g_step_cmd_active = false;
  g_interboard_retreat_active = false;
  auto_mode = 0;
  target_x = 0.0f;
  target_y = 0.0f;
  target_yaw = 0.0f;
  chassis_control_mode = VEL_Control;
  chassis_v.vx = 0.0f;
  chassis_v.vy = 0.0f;
  chassis_v.wz = 0.0f;
}

static bool TryApplyInterboardTarget(void) {
  int32_t x_mm = 0;
  int32_t y_mm = 0;
  int32_t yaw_mm = 0;
  if (!InterboardComm_TryConsumeTargetMm(&x_mm, &y_mm, &yaw_mm)) {
    return false;
  }

  ResetVisionTargetFilter();
  ResetAutoAlignPhaseState();
  g_step_cmd_active = false;
  // Keep retreat path latched until trajectory finishes, otherwise
  // AUTO_ALIGN vision branch may overwrite the freshly received target.
  g_interboard_retreat_active = true;

  target_x = (float)x_mm * kMmToMScale;
  target_y = (float)y_mm * kMmToMScale;
  target_yaw = (float)yaw_mm * kMmToMScale;
  chassis_control_mode = POS_Control;
  return true;
}

static bool ArmActionKeyTriggered(void) {
  const uint32_t event = button_status;
  return ((event & (1U << 0)) != 0U) ||
         ((event & (1U << 2)) != 0U) ||
         ((event & (1U << 6)) != 0U);
}

static void ApplyInterboardRetreatByPosition(void) {
  ResetVisionTargetFilter();
  ResetAutoAlignPhaseState();

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

  if (HAL_UART_Receive_IT(&huart2, &lr_uart2_rx_byte, 1) != HAL_OK) {
    vision_uart2_diag_rearm_fail_cnt++;
  }

  if (HAL_UART_Receive_IT(&huart4, &interboard_uart4_rx_byte, 1) != HAL_OK) {
    // UART4 版间通信若启动失败，InterboardComm 内部会保持超时保护。
  }
}

uint8_t byte_test = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    ProcessRxBytes(rx_dma_buf, RX_DMA_BUF_SIZE);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_dma_buf, RX_DMA_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  } else if (huart->Instance == USART2) {
    const uint8_t rx_byte = lr_uart2_rx_byte;
    vision_uart2_diag_rx_irq_cnt++;
    vision_uart2_diag_rx_byte_cnt++;
    // 先重启接收，尽量缩短无保护窗口，避免连续字节导致ORE。
    if (HAL_UART_Receive_IT(&huart2, &lr_uart2_rx_byte, 1) != HAL_OK) {
      vision_uart2_diag_rearm_fail_cnt++;
      return;
    }
    LR_Parse_And_Store(rx_byte);
  } else if (huart->Instance == UART4) {
    const uint8_t rx_byte = interboard_uart4_rx_byte;
    if (HAL_UART_Receive_IT(&huart4, &interboard_uart4_rx_byte, 1) != HAL_OK) {
      return;
    }
    InterboardComm_OnUartByte(rx_byte);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2) {
    vision_uart2_diag_err_cnt++;
    vision_uart2_diag_last_err_code = huart->ErrorCode;

    // 清除常见UART错误标志，避免错误中断反复触发导致接收回调停滞。
    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);

    if (HAL_UART_Receive_IT(&huart2, &lr_uart2_rx_byte, 1) != HAL_OK) {
      vision_uart2_diag_rearm_fail_cnt++;

    }
  } else if (huart->Instance == UART4) {
    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);

    (void)HAL_UART_Receive_IT(&huart4, &interboard_uart4_rx_byte, 1);
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
      ResetVisionTargetFilter();
      ResetVisionSignalState();
      ResetAutoAlignPhaseState();
      chassis_control_mode = VEL_Control;
      chassis_v.vx = __JOYSTICK2VEL__(LY_T);
      chassis_v.vy = __JOYSTICK2VEL__(-1.0f * LX_T);
      chassis_v.wz = __JOYSTICK2WZ__(-1.0f * RX_T);
      break;
    case CLAMP_MODE:
      ResetVisionTargetFilter();
      ResetVisionSignalState();
      ResetAutoAlignPhaseState();
      chassis_control_mode = VEL_Control;
      chassis_v.vx = 0;
      chassis_v.vy = 0;
      chassis_v.wz = 0;

      break;
    case AUTO_ALIGN_MODE:
      if (ArmActionKeyTriggered()) {
        g_wait_interboard_target = true;
      }

      if (button[8] || (button_status & (1U << 8))) {
        // 按下或触发按钮8都立即锁止，且按住期间持续锁止。
        g_emergency_hold_active = true;
      }

      if (g_emergency_hold_active) {
        AbortAutoAlignAndStop();
        g_wait_interboard_target = false;
        if (!button[8]) {
          // 释放后退出锁止态，避免永久占用自动对齐流程。
          g_emergency_hold_active = false;
        }
      } else if (g_wait_interboard_target) {
        // 收到机械臂动作按键后立即停对齐，等待MCU2下发目标信息。
        AbortAutoAlignAndStop();
        if (TryApplyInterboardTarget()) {
          g_wait_interboard_target = false;
        }
      } else if (InterboardComm_IsRetreatRequested()) {// 远程请求后退优先级高于自动对齐，确保安全。
        ApplyInterboardRetreatByPosition();
      } else if (g_interboard_retreat_active) {// 如果正在执行远程后退，则持续执行，直到完成。避免在后退过程中被自动对齐命令打断。
        ApplyInterboardRetreatByPosition();
      } 
      else {
        if (!ApplyVisionAutoAlign()) {
          ApplyButtonStepAlignFallback();
        }
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
  uint32_t publish_flags = button_status;
  if (joystick_mode == AUTO_ALIGN_MODE) {
    // 自动对齐模式下仅保留模式切换键事件，避免其他任务消费同一按键位产生冲突。
    publish_flags &= (1U << 4);
  }
  bbb = osEventFlagsSet(flags_id, publish_flags);
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
