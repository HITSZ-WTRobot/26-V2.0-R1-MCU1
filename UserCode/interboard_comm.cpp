#include "interboard_comm.hpp"

#include "can.h"
#include "can_driver.h"

namespace {

constexpr uint32_t kInterboardRetreatCmdId = 0x168U;
constexpr uint8_t kMagic0 = 0xA5U;
constexpr uint8_t kMagic1 = 0x5AU;
constexpr uint32_t kRetreatCmdTimeoutMs = 80U;

volatile uint8_t g_retreat_enable = 0U;
volatile uint32_t g_last_rx_tick_ms = 0U;

void InterboardRxCallback(const CAN_HandleTypeDef* hcan,
                          const CAN_RxHeaderTypeDef* header,
                          const uint8_t* data) {
  (void)hcan;

  if (header == nullptr || data == nullptr) {
    return;
  }
  if (header->IDE != CAN_ID_STD || header->StdId != kInterboardRetreatCmdId) {
    return;
  }
  if (header->DLC < 3U) {
    return;
  }
  if (data[0] != kMagic0 || data[1] != kMagic1) {
    return;
  }

  g_retreat_enable = data[2] ? 1U : 0U;
  g_last_rx_tick_ms = HAL_GetTick();
}

} // namespace

void InterboardComm_Init(void) {
  CAN_RegisterCallback(&hcan1, InterboardRxCallback);
  CAN_RegisterCallback(&hcan2, InterboardRxCallback);
}

void InterboardComm_SendRetreatCommand(bool enable) { (void)enable; }

bool InterboardComm_IsRetreatRequested(void) {
  if (g_retreat_enable == 0U) {
    return false;
  }
  const uint32_t now = HAL_GetTick();
  return ((uint32_t)(now - g_last_rx_tick_ms) <= kRetreatCmdTimeoutMs);
}
