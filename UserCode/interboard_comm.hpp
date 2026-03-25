#pragma once

#include <stdbool.h>
#include <stdint.h>

void InterboardComm_Init(void);
void InterboardComm_SendRetreatCommand(bool enable);
bool InterboardComm_IsRetreatRequested(void);
void InterboardComm_OnUartByte(uint8_t byte);

void InterboardComm_SendTargetMm(int32_t x_mm, int32_t y_mm, int32_t yaw_mm);
bool InterboardComm_TryConsumeTargetMm(int32_t *x_mm, int32_t *y_mm, int32_t *yaw_mm);
