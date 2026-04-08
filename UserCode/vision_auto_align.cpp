#include "vision_auto_align.hpp"

#include "interboard_comm.hpp"
#include "vision_receive.hpp"
#include <cmath>

constexpr uint32_t kVisionLostCycleThreshold        = 10U; // controller_task 10ms周期，约100ms
constexpr uint32_t kAutoAlignWindowCapacity           = 20U;  // 滤波窗口容量
constexpr float    kAutoAlignOutlierThresholdM        = 0.05f; // 位置离群值阈值，单位米
constexpr float    kAutoAlignOutlierThresholdDeg      = 2.0f; // 朝向离群值阈值，单位度
constexpr float    kInterboardRetreatDistanceM        = 0.20f;
constexpr float    kMmToMScale                 = 0.001f; // 板间通信的目标单位为毫米，这个常量用于转换为米以供自动对齐使用

//滤波系数和限幅值，自动对齐时每周期（10ms）允许的最大调整量，过大可能导致震荡，过小可能导致响应迟钝
constexpr float    kVisionLpfAlpha             = 0.85f; 
constexpr float    kVisionMaxStepPerCycleM     = 0.03f;
constexpr float    kVisionMaxStepPerCycleDeg   = 12.0f;
constexpr float    kVisionPosDeadbandM         = 0.03f;
constexpr float    kVisionYawLpfAlpha          = 0.60f;
constexpr float    kVisionYawDeadbandDeg       = 0.8f;
constexpr float    kAutoAlignYawLockDeg        = 2.0f;

static uint32_t g_vision_last_update_seq       = 0U;
static uint32_t g_vision_stale_cycles          = 0U;
static bool     g_auto_align_pos_executed_once = false;
static uint32_t g_auto_align_window_count      = 0U;  // 窗口中当前有效样本数
static uint32_t g_auto_align_last_sample_seq   = 0U;
static float    g_auto_align_window_x          = 0.0f; // 窗口内X坐标累加和
static float    g_auto_align_window_y          = 0.0f; // 窗口内Y坐标累加和
static float    g_auto_align_window_yaw        = 0.0f; // 窗口内朝向累加和
static bool     g_step_cmd_active              = false; 
static bool     g_interboard_retreat_active    = false; // 板间通信的后退命令生效中标志
static bool     g_interboard_retreat_last_req  = false; // 上一周期板间通信的后退命令请求状态
static bool     g_emergency_hold_active        = false; // 紧急停止状态，触发后立即停止底盘并禁止自动对齐流程，直到手动重置
static bool     g_wait_interboard_target       = false; // 是否在等待板间通信的新目标，若为true则暂不接受视觉输入以覆盖目标，直到收到新目标或超时
static bool     g_vision_filter_inited         = false; // 视觉输入滤波器是否已初始化，未初始化时直接将首个输入作为滤波器初始值
static float    g_target_x_filtered            = 0.0f;
static float    g_target_y_filtered            = 0.0f;

static inline float ClampFloat(float value, float min_value, float max_value)
{
    return value < min_value ? min_value : (value > max_value ? max_value : value);
}

//滤波控制，闭环位置坐标滤波
static void ApplyVisionTargetFilter(float raw_x, float raw_y, float* out_x, float* out_y)
{
    if (!out_x || !out_y)
    {
        return;
    }

    if (!g_vision_filter_inited)
    {
        g_target_x_filtered    = raw_x;
        g_target_y_filtered    = raw_y;
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

    if (fabsf(g_target_x_filtered) < kVisionPosDeadbandM)
    {
        g_target_x_filtered = 0.0f;
    }
    if (fabsf(g_target_y_filtered) < kVisionPosDeadbandM)
    {
        g_target_y_filtered = 0.0f;
    }

    *out_x = g_target_x_filtered;
    *out_y = g_target_y_filtered;
}

//滤波控制，闭环朝向滤波
static void ApplyYawTargetFilter(float raw_yaw, float* yaw)
{
    if (!yaw)
    {
        return;
    }

    const float dyaw =
            ClampFloat(raw_yaw - *yaw, -kVisionMaxStepPerCycleDeg, kVisionMaxStepPerCycleDeg);
    float yaw_output = *yaw + kVisionYawLpfAlpha * dyaw;

    if (fabsf(yaw_output) < kVisionYawDeadbandDeg)
    {
        yaw_output = 0.0f;
    }

    *yaw = yaw_output;
}

//监测机械臂自动流程控制的触发按键，避免与底盘控制按键冲突
static bool ArmActionKeyTriggered(uint32_t button_status)
{
    return ((button_status & (1U << 0)) != 0U) || ((button_status & (1U << 2)) != 0U) ||
           ((button_status & (1U << 6)) != 0U);
}

//监测紧急停止按键，触发后立即停止底盘并禁止自动对齐流程，直到手动重置
static void AbortAutoAlignAndStop(float*              target_x,
                                  float*              target_y,
                                  float*              target_yaw,
                                  Control_Mode*       chassis_control_mode,
                                  Chassis_Velocity_t* chassis_v,
                                  uint8_t*            auto_mode)
{
    if (!target_x || !target_y || !target_yaw || !chassis_control_mode || !chassis_v || !auto_mode)
    {
        return;
    }

    VisionAutoAlign_ResetState();
    g_step_cmd_active           = false;
    g_interboard_retreat_active = false;
    *auto_mode                  = 0;
    *target_x                   = 0.0f;
    *target_y                   = 0.0f;
    *target_yaw                 = 0.0f;
    *chassis_control_mode       = VEL_Control;
    chassis_v->vx               = 0.0f;
    chassis_v->vy               = 0.0f;
    chassis_v->wz               = 0.0f;
}

// 应用板间通信接收到的目标，优先级高于视觉输入
static bool TryApplyInterboardTarget(float*        target_x,
                                     float*        target_y,
                                     float*        target_yaw,
                                     Control_Mode* chassis_control_mode)
{
    int32_t x_mm   = 0;
    int32_t y_mm   = 0;
    int32_t yaw_mm = 0;
    if (!target_x || !target_y || !target_yaw || !chassis_control_mode)
    {
        return false;
    }

    if (!InterboardComm_TryConsumeTargetMm(&x_mm, &y_mm, &yaw_mm))
    {
        return false;
    }

    VisionAutoAlign_ResetState();
    g_step_cmd_active = false;
    // Keep retreat path latched until trajectory finishes, otherwise
    // AUTO_ALIGN vision branch may overwrite the freshly received target.
    g_interboard_retreat_active = true;

    *target_x             = (float)x_mm * kMmToMScale;
    *target_y             = (float)y_mm * kMmToMScale;
    *target_yaw           = (float)yaw_mm;
    *chassis_control_mode = POS_Control;
    return true;
}

// 应用板间通信的后退命令，优先级高于视觉输入和自动对齐流程
static void ApplyInterboardRetreatByPosition(float*              target_x,
                                             float*              target_y,
                                             float*              target_yaw,
                                             Control_Mode*       chassis_control_mode,
                                             Chassis_Velocity_t* chassis_v)
{
    if (!target_x || !target_y || !target_yaw || !chassis_control_mode || !chassis_v)
    {
        return;
    }

    VisionAutoAlign_ResetState();

    const bool retreat_req = InterboardComm_IsRetreatRequested();

    // 上升沿触发一次固定距离后退，距离定义在 MCU1 侧。
    if (retreat_req && !g_interboard_retreat_last_req)
    {
        g_interboard_retreat_active = true;
        g_step_cmd_active           = false;
        *target_x                   = -kInterboardRetreatDistanceM;
        *target_y                   = 0.0f;
        *target_yaw                 = 0.0f;
    }
    g_interboard_retreat_last_req = retreat_req;

    if (g_interboard_retreat_active)
    {
        *chassis_control_mode = POS_Control;
        if (chassis_ && chassis_->isTrajectoryFinished())
        {
            g_interboard_retreat_active = false;
            *target_x                   = 0.0f;
            *target_y                   = 0.0f;
            *target_yaw                 = 0.0f;
            *chassis_control_mode       = VEL_Control;
            chassis_v->vx               = 0.0f;
            chassis_v->vy               = 0.0f;
            chassis_v->wz               = 0.0f;
        }
    }
}


void VisionAutoAlign_ResetState(void)
{
    g_vision_last_update_seq       = 0U;
    g_vision_stale_cycles          = 0U;
    g_auto_align_pos_executed_once = false;
    g_auto_align_window_count      = 0U;
    g_auto_align_last_sample_seq   = 0U;
    g_auto_align_window_x          = 0.0f;
    g_auto_align_window_y          = 0.0f;
    g_auto_align_window_yaw        = 0.0f;
    g_step_cmd_active              = false;
    g_interboard_retreat_active    = false;
    g_interboard_retreat_last_req  = false;
    g_emergency_hold_active        = false;
    g_wait_interboard_target       = false;
    g_vision_filter_inited         = false;
    g_target_x_filtered            = 0.0f;
    g_target_y_filtered            = 0.0f;
}

void VisionAutoAlign_OnModeEnter(void)
{
    VisionAutoAlign_ResetState();
}

//应用视觉对齐坐标
static bool VisionAutoAlign_Apply(float*              target_x,
                                  float*              target_y,
                                  float*              target_yaw,
                                  Control_Mode*       chassis_control_mode,
                                  Chassis_Velocity_t* chassis_v,
                                  uint8_t*            auto_mode)
{
    if (!target_x || !target_y || !target_yaw || !chassis_control_mode || !chassis_v || !auto_mode)
    {
        return false;
    }

    chassis_v->vx = 0.0f;
    chassis_v->vy = 0.0f;
    chassis_v->wz = 0.0f;

    if (g_auto_align_pos_executed_once)
    {
        return true;
    }

    const uint32_t apriltag_seq = lr_apriltag_update_seq;
    const uint32_t detect_seq   = lr_detect_update_seq;
    const uint32_t latest_seq   = (apriltag_seq >= detect_seq) ? apriltag_seq : detect_seq;
    const int      has_apriltag = (apriltag_seq > 0U);
    const int      has_detect   = (detect_seq > 0U);
    if (!has_apriltag && !has_detect)
    {
        *auto_mode = 0;
        return false;
    }

    if (g_vision_last_update_seq == 0U || latest_seq != g_vision_last_update_seq)
    {
        g_vision_last_update_seq = latest_seq;
        g_vision_stale_cycles    = 0U;
    }
    else
    {
        if (g_vision_stale_cycles < 0xFFFFFFFFU)
        {
            g_vision_stale_cycles++;
        }
        if (g_vision_stale_cycles > kVisionLostCycleThreshold)
        {
            *auto_mode = 0;
            return false;
        }
    }

    const int use_apriltag = has_apriltag && (!has_detect || (apriltag_seq >= detect_seq));
    *auto_mode             = use_apriltag ? 2 : 1; // 2=apriltag, 1=detect
    g_step_cmd_active      = false;

    if (latest_seq == g_auto_align_last_sample_seq)
    {
        return true;
    }
    g_auto_align_last_sample_seq = latest_seq;

    LR_DataPacket src = { 0 };
    if (use_apriltag)
    {
        const int latest_idx = (lr_apriltag_write_idx + LR_DATA_MAX_NUM - 1) % LR_DATA_MAX_NUM;
        src                  = lr_apriltag_buffer[latest_idx];
    }
    else
    {
        const int latest_idx = (lr_detect_write_idx + LR_DATA_MAX_NUM - 1) % LR_DATA_MAX_NUM;
        src                  = lr_detect_buffer[latest_idx];
    }

    float sample_target_x   = 0.0f;
    float sample_target_y   = 0.0f;
    float sample_target_yaw = 0.0f;
    LR_Compute_Target(
            src.x, src.y, src.z, src.yaw, &sample_target_x, &sample_target_y, &sample_target_yaw);

    ApplyVisionTargetFilter(sample_target_x, sample_target_y, &sample_target_x, &sample_target_y);
    ApplyYawTargetFilter(sample_target_yaw, &sample_target_yaw);

    // 窗口滤波：检查新样本是否为离群值
    if (g_auto_align_window_count > 0U)
    {
        const float window_avg_x   = g_auto_align_window_x / (float)g_auto_align_window_count;
        const float window_avg_y   = g_auto_align_window_y / (float)g_auto_align_window_count;
        const float window_avg_yaw = g_auto_align_window_yaw / (float)g_auto_align_window_count;
        
        const float delta_x   = fabsf(sample_target_x - window_avg_x);
        const float delta_y   = fabsf(sample_target_y - window_avg_y);
        const float delta_yaw = fabsf(sample_target_yaw - window_avg_yaw);
        const float delta_pos = sqrtf(delta_x * delta_x + delta_y * delta_y);
        
        // 如果位置或朝向偏差过大则舍弃该样本
        if (delta_pos > kAutoAlignOutlierThresholdM || delta_yaw > kAutoAlignOutlierThresholdDeg)
        {
            return true; // 舍弃离群值，继续等待下一个样本
        }
    }

    // 样本有效，加入窗口
    g_auto_align_window_x += sample_target_x;
    g_auto_align_window_y += sample_target_y;
    g_auto_align_window_yaw += sample_target_yaw;
    g_auto_align_window_count++;

    if (g_auto_align_window_count < kAutoAlignWindowCapacity)
    {
        return true;
    }

    // 窗口满，计算平均值并应用
    const float inv_count          = 1.0f / (float)g_auto_align_window_count;
    *target_x                      = g_auto_align_window_x * inv_count;
    *target_y                      = g_auto_align_window_y * inv_count;
    *target_yaw                    = g_auto_align_window_yaw * inv_count;
    *chassis_control_mode          = POS_Control;
    g_auto_align_pos_executed_once = true;

    return true;
}

// 自动对齐遥控模式下的总逻辑流程，后续可以对应到按键进行更改
void VisionAutoAlign_RunMode(uint32_t            button_status,
                             bool                button8_pressed,
                             float*              target_x,
                             float*              target_y,
                             float*              target_yaw,
                             Control_Mode*       chassis_control_mode,
                             Chassis_Velocity_t* chassis_v,
                             uint8_t*            auto_mode)
{
    if (!target_x || !target_y || !target_yaw || !chassis_control_mode || !chassis_v || !auto_mode)
    {
        return;
    }

    if (ArmActionKeyTriggered(button_status))
    {
        g_wait_interboard_target = true;
    }

    if (button8_pressed || ((button_status & (1U << 8)) != 0U))
    {
        // 按下或触发按钮8都立即锁止，且按住期间持续锁止。
        g_emergency_hold_active = true;
    }

    if (g_emergency_hold_active)
    {
        AbortAutoAlignAndStop(
                target_x, target_y, target_yaw, chassis_control_mode, chassis_v, auto_mode);
        g_wait_interboard_target = false;
        if (!button8_pressed)
        {
            // 释放后退出锁止态，避免永久占用自动对齐流程。
            g_emergency_hold_active = false;
        }
    }
    else if (g_wait_interboard_target)
    {
        // 收到机械臂动作按键后立即停对齐，等待MCU2下发目标信息。
        AbortAutoAlignAndStop(
                target_x, target_y, target_yaw, chassis_control_mode, chassis_v, auto_mode);
        if (TryApplyInterboardTarget(target_x, target_y, target_yaw, chassis_control_mode))
        {
            g_wait_interboard_target = false;
        }
    }
    else if (InterboardComm_IsRetreatRequested())
    { // 远程请求后退优先级高于自动对齐，确保安全。
        ApplyInterboardRetreatByPosition(
                target_x, target_y, target_yaw, chassis_control_mode, chassis_v);
    }
    else if (g_interboard_retreat_active)
    { // 如果正在执行远程后退，则持续执行，直到完成。避免在后退过程中被自动对齐命令打断。
        ApplyInterboardRetreatByPosition(
                target_x, target_y, target_yaw, chassis_control_mode, chassis_v);
    }
    else
    {
        VisionAutoAlign_Apply(
                target_x, target_y, target_yaw, chassis_control_mode, chassis_v, auto_mode);
    }
}
