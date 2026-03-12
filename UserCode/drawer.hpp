#pragma once

#include "cmsis_os2.h"
#include "device.hpp"
#include "eventflags.hpp"
#include "motor_pos_controller.hpp"
#include "motor_trajectory.hpp"
#include "motor_vel_controller.hpp"

#include <array>
#include <cstdint>

typedef struct
{
    float    drawer_pushout_speed_abs;
    float    drawer_pushout_start_vel_th;
    float    drawer_pushout_stall_vel_th;
    float    drawer_pushout_stall_out_th;
    uint32_t drawer_pushout_min_run_ms;
    uint32_t drawer_pushout_stall_confirm_ms;
} DRAWER_PUSHOUT_PARAMS_T;

typedef enum
{
    MODE_UP = 0,
    MODE_RESET_DOWN
} RUN_MODE_E;

typedef enum
{
    DRAWER_IN = 0,
    DRAWER_OUT,
    DRAWER_UP,
    DRAWER_RUNNING,
    DRAWER_UNKNOWN
} DRAWER_STATUS_E;

class Drawer
{
public:
    Drawer();

    void update_1kHz();
    void update_100Hz();
    void softTIM();

private:
    static constexpr float RESET_NEG_BIG_TARGET   = -100000.0f;
    static constexpr uint32_t RESET_MIN_RUN_MS       = 800u;
    static constexpr uint32_t RESET_STALL_CONFIRM_MS = 100u;
    static constexpr float RESET_STALL_VEL_TH     = 8.0f;
    static constexpr float RESET_STALL_OUT_TH     = 2000.0f;
    static constexpr float UP_DISTANCE            = 4200.0f;

    static constexpr DRAWER_PUSHOUT_PARAMS_T drawer_pushout_params = {
        60.0f,
        15.0f,
        8.0f,
        4000.0f,
        800u,
        1000u
    };

    bool target_up         = false;
    bool target_out        = false;
    bool target_out_enable = true;

    uint8_t  stall_cond  = 0;
    uint32_t now         = 0;
    uint32_t stall_start = 0;

    controllers::MotorVelController vel_drawer_1;
    controllers::MotorVelController vel_drawer_2;
    controllers::MotorVelController vel_drawer_3;
    controllers::MotorVelController vel_drawer_4;
    controllers::MotorVelController vel_drawer_out;
    controllers::MotorPosController pos_drawer_out;

    controllers::MotorVelController* up_down_ctrls[4];
    trajectory::MotorTrajectory<4>   up;
    trajectory::MotorTrajectory<4>   down;

    uint32_t drawerup_count  = 0;
    uint32_t drawerout_count = 0;

    int             flagup_s_update = 0;
    DRAWER_STATUS_E drawer_status   = DRAWER_UNKNOWN;

    uint8_t                  reset_running  = 0;
    uint32_t                 reset_start_ms = 0;
    std::array<uint8_t, 4>   reset_stall_seen{ 0, 0, 0, 0 };
    std::array<uint32_t, 4>  reset_stall_t0{ 0, 0, 0, 0 };
    bool                     up_running   = false;
    bool                     down_running = false;

    osThreadId_t drawerUPHandle  = nullptr;
    osThreadId_t drawerOUTHandle = nullptr;

    static void DrawerUP_Control(void* argument);
    static void DrawerOUT_Control(void* argument);

    void DrawerUP_Control_Loop();
    void DrawerOUT_Control_Loop();

    void Drawer_TIM_Callback();
    void force_zero_all();
    void PushUp_DoneDetect_Step();
    bool reset_any_stalled() const;
    [[maybe_unused]] bool reset_all_stalled() const;
    void reset_angle_all();
    void ResetDown_Step();
    void drawer_pushout_move_to_end(int dir);
    void ResetDown_Start();
};

