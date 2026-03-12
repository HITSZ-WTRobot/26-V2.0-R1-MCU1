#include "drawer.hpp"

#include "main.h"

#include <cmath>

namespace
{

controllers::MotorVelController::Config make_drawer_vel_ctrl_cfg()
{
    controllers::MotorVelController::Config cfg{};
    cfg.pid.Kp             = 25.0f;
    cfg.pid.Ki             = 0.08f;
    cfg.pid.Kd             = 5.0f;
    cfg.pid.abs_output_max = 8000.0f;
    return cfg;
}

controllers::MotorPosController::Config make_drawer_pos_ctrl_cfg()
{
    controllers::MotorPosController::Config cfg{};

    cfg.velocity_pid.Kp             = 25.0f;
    cfg.velocity_pid.Ki             = 0.08f;
    cfg.velocity_pid.Kd             = 5.0f;
    cfg.velocity_pid.abs_output_max = 4500.0f;

    cfg.position_pid.Kp             = 8.0f;
    cfg.position_pid.Ki             = 0.0065f;
    cfg.position_pid.Kd             = 15.0f;
    cfg.position_pid.abs_output_max = 500.0f;

    cfg.pos_vel_freq_ratio = 1;
    return cfg;
}

velocity_profile::SCurveProfile::Config make_up_profile_cfg()
{
    velocity_profile::SCurveProfile::Config cfg{};
    cfg.max_spd  = 240.0f;
    cfg.max_acc  = 60.0f;
    cfg.max_jerk = 120.0f;
    return cfg;
}

velocity_profile::SCurveProfile::Config make_down_profile_cfg()
{
    velocity_profile::SCurveProfile::Config cfg{};
    cfg.max_spd  = 240.0f;
    cfg.max_acc  = 30.0f;
    cfg.max_jerk = 40.0f;
    return cfg;
}

const PD::Config up_error_pd_cfg = [] {
    PD::Config cfg{};
    cfg.Kp             = 50.0f;
    cfg.Kd             = 0.5f;
    cfg.abs_output_max = 60.0f;
    return cfg;
}();

const PD::Config down_error_pd_cfg = [] {
    PD::Config cfg{};
    cfg.Kp             = 50.0f;
    cfg.Kd             = 0.5f;
    cfg.abs_output_max = 600.0f;
    return cfg;
}();

const osThreadAttr_t drawerUP_attributes = [] {
    osThreadAttr_t attr{};
    attr.name       = "drawerUP";
    attr.stack_size = 128 * 8;
    attr.priority   = static_cast<osPriority_t>(osPriorityHigh);
    return attr;
}();

const osThreadAttr_t drawerOUT_attributes = [] {
    osThreadAttr_t attr{};
    attr.name       = "drawerOUT";
    attr.stack_size = 128 * 8;
    attr.priority   = static_cast<osPriority_t>(osPriorityHigh);
    return attr;
}();

} // namespace

Drawer::Drawer() :
    vel_drawer_1(motor_drawer_1, make_drawer_vel_ctrl_cfg()),
    vel_drawer_2(motor_drawer_2, make_drawer_vel_ctrl_cfg()),
    vel_drawer_3(motor_drawer_3, make_drawer_vel_ctrl_cfg()),
    vel_drawer_4(motor_drawer_4, make_drawer_vel_ctrl_cfg()),
    vel_drawer_out(motor_drawer_out, make_drawer_vel_ctrl_cfg()),
    pos_drawer_out(motor_drawer_out, make_drawer_pos_ctrl_cfg()),
    up_down_ctrls{ &vel_drawer_1, &vel_drawer_2, &vel_drawer_3, &vel_drawer_4 },
    up(up_down_ctrls, make_up_profile_cfg(), up_error_pd_cfg),
    down(up_down_ctrls, make_down_profile_cfg(), down_error_pd_cfg)
{
    vel_drawer_1.disable();
    vel_drawer_2.disable();
    vel_drawer_3.disable();
    vel_drawer_4.disable();
    vel_drawer_out.disable();
    pos_drawer_out.disable();

    up.disable();
    down.disable();
    drawer_status = DRAWER_UNKNOWN;

    drawerUPHandle  = osThreadNew(DrawerUP_Control, this, &drawerUP_attributes);
    drawerOUTHandle = osThreadNew(DrawerOUT_Control, this, &drawerOUT_attributes);
}

void Drawer::update_1kHz()
{
    Drawer_TIM_Callback();
}

void Drawer::update_100Hz()
{
}

void Drawer::softTIM()
{
    if ((osEventFlagsWait(flags_id, 0x00000001U, osFlagsWaitAny, 0) & 0xFF000001U) == 0x00000001U)
    {
        target_out = !target_out;
    }
    if ((osEventFlagsWait(flags_id, 0x00000002U, osFlagsWaitAny, 0) & 0xFF000002U) == 0x00000002U)
    {
        target_up = !target_up;
    }
}

void Drawer::force_zero_all()
{
    vel_drawer_1.setRef(0.0f);
    vel_drawer_1.getPID().reset();
    vel_drawer_2.setRef(0.0f);
    vel_drawer_2.getPID().reset();
    vel_drawer_3.setRef(0.0f);
    vel_drawer_3.getPID().reset();
    vel_drawer_4.setRef(0.0f);
    vel_drawer_4.getPID().reset();
}

void Drawer::PushUp_DoneDetect_Step()
{
    if (up_running && up.isFinished())
    {
        drawer_status = DRAWER_UP;
    }
}

bool Drawer::reset_any_stalled() const
{
    return reset_stall_seen[0] || reset_stall_seen[1] || reset_stall_seen[2] || reset_stall_seen[3];
}

bool Drawer::reset_all_stalled() const
{
    return reset_stall_seen[0] && reset_stall_seen[1] && reset_stall_seen[2] && reset_stall_seen[3];
}

void Drawer::reset_angle_all()
{
    if (motor_drawer_1) motor_drawer_1->resetAngle();
    if (motor_drawer_2) motor_drawer_2->resetAngle();
    if (motor_drawer_3) motor_drawer_3->resetAngle();
    if (motor_drawer_4) motor_drawer_4->resetAngle();
}

void Drawer::ResetDown_Step()
{
    if (!reset_running)
        return;

    const uint32_t tick_now = HAL_GetTick();
    if ((tick_now - reset_start_ms) < RESET_MIN_RUN_MS)
    {
        for (size_t i = 0; i < 4; i++)
            reset_stall_t0[i] = 0;
        return;
    }

    motors::DJIMotor*                 m[4] = { motor_drawer_1, motor_drawer_2, motor_drawer_3, motor_drawer_4 };
    controllers::MotorVelController*  c[4] = { &vel_drawer_1, &vel_drawer_2, &vel_drawer_3, &vel_drawer_4 };

    for (size_t i = 0; i < 4; i++)
    {
        if (reset_stall_seen[i])
            continue;
        if (m[i] == nullptr)
            continue;

        const bool cond = (std::fabs(c[i]->getPID().getOutput()) >= RESET_STALL_OUT_TH) &&
                          (std::fabs(m[i]->getVelocity()) <= RESET_STALL_VEL_TH);

        if (cond)
        {
            if (reset_stall_t0[i] == 0)
                reset_stall_t0[i] = tick_now;

            if ((tick_now - reset_stall_t0[i]) >= RESET_STALL_CONFIRM_MS)
            {
                reset_stall_seen[i] = 1;
                reset_stall_t0[i]   = 0;
            }
        }
        else
        {
            reset_stall_t0[i] = 0;
        }
    }

    if (reset_any_stalled())
    {
        flagup_s_update = 0;

        down.stop();
        down.disable();
        down_running = false;

        force_zero_all();
        reset_angle_all();
        reset_running = 0;

        if (up.enable())
        {
            (void)up.setTarget(0.0f);
            up.stop();
        }
        up_running = false;

        drawer_status = DRAWER_OUT;
    }
}

void Drawer::drawer_pushout_move_to_end(const int dir)
{
    const float vref = (dir >= 0) ? std::fabs(drawer_pushout_params.drawer_pushout_speed_abs)
                                  : -std::fabs(drawer_pushout_params.drawer_pushout_speed_abs);

    pos_drawer_out.disable();
    vel_drawer_out.enable();
    vel_drawer_out.setRef(vref);
    drawer_status = DRAWER_RUNNING;

    const uint32_t t0 = HAL_GetTick();
    stall_start       = HAL_GetTick();

    while (motor_drawer_out &&
           (std::fabs(motor_drawer_out->getVelocity()) <= drawer_pushout_params.drawer_pushout_start_vel_th))
    {
        osDelay(1);
    }

    while (motor_drawer_out)
    {
        now = HAL_GetTick();
        if ((now - t0) < drawer_pushout_params.drawer_pushout_min_run_ms)
        {
            osDelay(1);
            continue;
        }

        stall_cond = (std::fabs(vel_drawer_out.getPID().getOutput()) >=
                      drawer_pushout_params.drawer_pushout_stall_out_th) &&
                     (std::fabs(motor_drawer_out->getVelocity()) <=
                      drawer_pushout_params.drawer_pushout_stall_vel_th);

        if (!stall_cond)
            stall_start = now;

        if (stall_cond)
        {
            if ((now - stall_start) >= drawer_pushout_params.drawer_pushout_stall_confirm_ms)
                break;
        }
        osDelay(1);
    }

    if (dir <= -1)
    {
        drawer_status = DRAWER_OUT;
    }
    else
    {
        drawer_status = DRAWER_IN;
    }

    vel_drawer_out.disable();
    vel_drawer_out.setRef(0.0f);
    vel_drawer_out.getPID().reset();

    pos_drawer_out.enable();
    if (motor_drawer_out)
        pos_drawer_out.setRef(motor_drawer_out->getAngle());
}

void Drawer::Drawer_TIM_Callback()
{
    if (flagup_s_update == 1)
    {
        if (reset_running == 0)
        {
            if (up_running)
            {
                up.profileUpdate(0.001f);
                up.errorUpdate();
            }
        }
        else
        {
            if (down_running)
            {
                down.profileUpdate(0.001f);
                down.errorUpdate();
            }
        }
    }
    else
    {
        force_zero_all();
    }

    vel_drawer_1.update();
    vel_drawer_2.update();
    vel_drawer_3.update();
    vel_drawer_4.update();
    vel_drawer_out.update();
    pos_drawer_out.update();

    PushUp_DoneDetect_Step();
    ResetDown_Step();
}

void Drawer::ResetDown_Start()
{
    flagup_s_update = 0;

    for (size_t i = 0; i < 4; i++)
    {
        reset_stall_seen[i] = 0;
        reset_stall_t0[i]   = 0;
    }

    reset_running  = 1;
    reset_start_ms = HAL_GetTick();

    up.stop();
    up.disable();
    up_running = false;

    if (down.enable())
        down_running = down.setTarget(RESET_NEG_BIG_TARGET);
    else
        down_running = false;

    drawer_status   = DRAWER_RUNNING;
    flagup_s_update = down_running ? 1 : 0;
}

void Drawer::DrawerUP_Control(void* argument)
{
    if (argument == nullptr)
    {
        osThreadExit();
        return;
    }
    static_cast<Drawer*>(argument)->DrawerUP_Control_Loop();
}

void Drawer::DrawerUP_Control_Loop()
{
    for (;;)
    {
        drawerup_count++;

        if (target_up && drawer_status == DRAWER_OUT && !reset_running)
        {
            drawer_status = DRAWER_RUNNING;

            down.stop();
            down.disable();
            down_running = false;

            if (up.enable())
                up_running = up.setTarget(UP_DISTANCE);
            else
                up_running = false;

            osDelay(500);
            flagup_s_update = up_running ? 1 : 0;
        }

        if (!target_up && (drawer_status == DRAWER_UP) && !reset_running)
        {
            ResetDown_Start();
        }
        osDelay(10);
    }
}

void Drawer::DrawerOUT_Control(void* argument)
{
    if (argument == nullptr)
    {
        osThreadExit();
        return;
    }
    static_cast<Drawer*>(argument)->DrawerOUT_Control_Loop();
}

void Drawer::DrawerOUT_Control_Loop()
{
    for (;;)
    {
        drawerout_count++;
        if (target_out_enable)
        {
            if (target_out && (drawer_status == DRAWER_IN || drawer_status == DRAWER_UNKNOWN))
            {
                drawer_pushout_move_to_end(-1);
            }
            if (!target_out && drawer_status == DRAWER_OUT)
            {
                drawer_pushout_move_to_end(+1);
            }
        }
        osDelay(10);
    }
}

