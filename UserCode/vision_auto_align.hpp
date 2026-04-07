#ifndef __VISION_AUTO_ALIGN_HPP__
#define __VISION_AUTO_ALIGN_HPP__

#include <stdbool.h>
#include <stdint.h>
#include "chassis.hpp"

void VisionAutoAlign_OnModeEnter(void);
void VisionAutoAlign_ResetState(void);

void VisionAutoAlign_RunMode(uint32_t button_status,
                             bool button8_pressed,
                             float *target_x,
                             float *target_y,
                             float *target_yaw,
                             Control_Mode *chassis_control_mode,
                             Chassis_Velocity_t *chassis_v,
                             uint8_t *auto_mode);

#endif
