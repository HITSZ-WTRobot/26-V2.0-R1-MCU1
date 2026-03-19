#pragma once

#include <stdbool.h>

void InterboardComm_Init(void);
void InterboardComm_SendRetreatCommand(bool enable);
bool InterboardComm_IsRetreatRequested(void);
