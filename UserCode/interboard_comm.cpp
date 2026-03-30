#include "interboard_comm.hpp"

#include "usart.h"

#include <string.h>

namespace {

constexpr uint8_t kFrameHeader = 0xAAU;
constexpr uint8_t kCmdRetreat = 0x01U;
constexpr uint8_t kCmdTarget = 0x02U;
constexpr uint8_t kAckMask = 0x80U;
constexpr uint8_t kTlvTypeX = 0x01U;
constexpr uint8_t kTlvTypeY = 0x02U;
constexpr uint8_t kTlvTypeYaw = 0x03U;
constexpr uint8_t kTlvI32Len = 4U;

constexpr uint8_t kMaxPayloadLen = 32U; // CMD(1) + DATA
constexpr uint32_t kRetreatCmdTimeoutMs = 80U;
constexpr uint32_t kTxTimeoutMs = 5U;

volatile uint8_t g_retreat_enable = 0U;
volatile uint32_t g_last_rx_tick_ms = 0U;
volatile uint8_t g_target_pending = 0U;
volatile int32_t g_target_x_mm = 0;
volatile int32_t g_target_y_mm = 0;
volatile int32_t g_target_yaw_mm = 0;

uint8_t g_rx_state = 0U;
uint8_t g_rx_payload_len = 0U;
uint8_t g_rx_payload_fill = 0U;
uint8_t g_rx_checksum = 0U;
uint8_t g_rx_payload[kMaxPayloadLen] = {0};

static uint8_t CalcChecksum(uint8_t len, const uint8_t *payload) {
  uint8_t sum = len;
  if (payload) {
    for (uint8_t i = 0; i < len; ++i) {
      sum ^= payload[i];
    }
  }
  return sum;
}

static uint8_t AppendTlvI32(uint8_t *out_buf,
                            uint8_t offset,
                            uint8_t type,
                            int32_t value) {
  if (!out_buf) {
    return offset;
  }
  out_buf[offset++] = type;
  out_buf[offset++] = kTlvI32Len;
  out_buf[offset++] = (uint8_t)(value & 0xFF);
  out_buf[offset++] = (uint8_t)((value >> 8) & 0xFF);
  out_buf[offset++] = (uint8_t)((value >> 16) & 0xFF);
  out_buf[offset++] = (uint8_t)((value >> 24) & 0xFF);
  return offset;
}

static void SendFrame(uint8_t cmd, const uint8_t *data, uint8_t data_len) {
  const uint8_t payload_len = (uint8_t)(1U + data_len);
  uint8_t frame[2U + kMaxPayloadLen + 1U] = {0}; // HEADER + LEN + PAYLOAD + CHK
  uint8_t payload[kMaxPayloadLen] = {0};

  if (payload_len > kMaxPayloadLen) {
    return;
  }

  payload[0] = cmd;
  if (data && data_len > 0U) {
    memcpy(&payload[1], data, data_len);
  }

  frame[0] = kFrameHeader;
  frame[1] = payload_len;
  memcpy(&frame[2], payload, payload_len);
  frame[2U + payload_len] = CalcChecksum(payload_len, payload);

  (void)HAL_UART_Transmit(&huart4,
                          frame,
                          (uint16_t)(3U + payload_len),
                          kTxTimeoutMs);
}

static void SendAck(uint8_t cmd) { SendFrame((uint8_t)(kAckMask | cmd), nullptr, 0U); }

static bool ParseTlvI32(const uint8_t *data,
                        uint8_t data_len,
                        uint8_t target_type,
                        int32_t *value_out) {
  if (!data || !value_out) {
    return false;
  }

  uint8_t idx = 0U;
  while (idx + 2U <= data_len) {
    const uint8_t type = data[idx++];
    const uint8_t len = data[idx++];
    if (idx + len > data_len) {
      return false;
    }
    if (type == target_type && len == kTlvI32Len) {
      *value_out = (int32_t)((uint32_t)data[idx] |
                             ((uint32_t)data[idx + 1U] << 8) |
                             ((uint32_t)data[idx + 2U] << 16) |
                             ((uint32_t)data[idx + 3U] << 24));
      return true;
    }
    idx = (uint8_t)(idx + len);
  }
  return false;
}

static void HandleFrame(uint8_t payload_len, const uint8_t *payload) {
  if (!payload || payload_len < 1U) {
    return;
  }

  const uint8_t cmd = payload[0];
  const uint8_t data_len = (uint8_t)(payload_len - 1U);
  const uint8_t *data = &payload[1];

  if ((cmd & kAckMask) != 0U) {
    return;
  }

  if (cmd == kCmdRetreat) {
    if (data_len >= 1U) {
      g_retreat_enable = data[0] ? 1U : 0U;
      g_last_rx_tick_ms = HAL_GetTick();
      SendAck(cmd);
    }
    return;
  }

  if (cmd == kCmdTarget) {
    int32_t x_mm = 0;
    int32_t y_mm = 0;
    int32_t yaw_mm = 0;
    const bool has_x = ParseTlvI32(data, data_len, kTlvTypeX, &x_mm);
    const bool has_y = ParseTlvI32(data, data_len, kTlvTypeY, &y_mm);
    const bool has_yaw = ParseTlvI32(data, data_len, kTlvTypeYaw, &yaw_mm);
    if (has_x && has_y && has_yaw) {
      g_target_x_mm = x_mm;
      g_target_y_mm = y_mm;
      g_target_yaw_mm = yaw_mm;
      g_target_pending = 1U;
      g_last_rx_tick_ms = HAL_GetTick();
      SendAck(cmd);
    }
  }
}

} // namespace

void InterboardComm_Init(void) {
  g_rx_state = 0U;
  g_rx_payload_len = 0U;
  g_rx_payload_fill = 0U;
  g_target_pending = 0U;
}

void InterboardComm_SendRetreatCommand(bool enable) {
  const uint8_t payload = (uint8_t)(enable ? 1U : 0U);
  SendFrame(kCmdRetreat, &payload, 1U);
}

void InterboardComm_OnUartByte(uint8_t byte) {
  switch (g_rx_state) {
  case 0U:
    g_rx_state = (byte == kFrameHeader) ? 1U : 0U;
    break;
  case 1U:
    if (byte == 0U || byte > kMaxPayloadLen) {
      g_rx_state = 0U;
    } else {
      g_rx_payload_len = byte;
      g_rx_payload_fill = 0U;
      g_rx_state = 2U;
    }
    break;
  case 2U:
    g_rx_payload[g_rx_payload_fill++] = byte;
    if (g_rx_payload_fill >= g_rx_payload_len) {
      g_rx_state = 3U;
    }
    break;
  case 3U:
    g_rx_checksum = byte;
    if (g_rx_checksum == CalcChecksum(g_rx_payload_len, g_rx_payload)) {
      HandleFrame(g_rx_payload_len, g_rx_payload);
    }
    g_rx_state = 0U;
    break;
  default:
    g_rx_state = 0U;
    break;
  }
}

bool InterboardComm_IsRetreatRequested(void) {
  if (g_retreat_enable == 0U) {
    return false;
  }
  const uint32_t now = HAL_GetTick();
  return ((uint32_t)(now - g_last_rx_tick_ms) <= kRetreatCmdTimeoutMs);
}

void InterboardComm_SendTargetMm(int32_t x_mm, int32_t y_mm, int32_t yaw_mm) {
  uint8_t data[18] = {0};
  uint8_t len = 0U;
  len = AppendTlvI32(data, len, kTlvTypeX, x_mm);
  len = AppendTlvI32(data, len, kTlvTypeY, y_mm);
  len = AppendTlvI32(data, len, kTlvTypeYaw, yaw_mm);
  SendFrame(kCmdTarget, data, len);
}

bool InterboardComm_TryConsumeTargetMm(int32_t *x_mm, int32_t *y_mm, int32_t *yaw_mm) {
  if (g_target_pending == 0U) {
    return false;
  }

  if (!x_mm || !y_mm || !yaw_mm) {
    return false;
  }

  *x_mm = g_target_x_mm;
  *y_mm = g_target_y_mm;
  *yaw_mm = g_target_yaw_mm;
  g_target_pending = 0U;
  return true;
}
