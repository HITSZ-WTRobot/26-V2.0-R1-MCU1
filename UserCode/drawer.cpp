#include "drawer.hpp"
#include "s_curve.hpp"

/* ===== 复位参数===== */
#define RESET_NEG_BIG_TARGET   (-100000.0f) // 复位曲线超级大的位置，永远达不到
#define RESET_MIN_RUN_MS       (800u)       // 起步后800ms不做堵转判定，因为刚开始运动时电流会比较大
#define RESET_STALL_CONFIRM_MS (100u)       // 堵转时间判定
#define RESET_STALL_VEL_TH     (8.0f)       // 堵转速度判定阈值
#define RESET_STALL_OUT_TH     (2000.0f)    // 堵转电流判定阈值
#define UP_DISTANCE            4200.0f      // 抬升距离

// 对外接口
bool target_up         = false; // 抬升目前目标是上面还是下面（1：目标为顶上||0：目标为底部）
bool target_out        = false; // 推出目前目标是推出还是收回（1：目标为推出||0：目标为收回）
bool target_out_enable = true;  // 推出使能

osTimerId_t drawer_timHandle;

uint8_t  stall_cond;
uint32_t now;
uint32_t stall_start;
// 电机控制器
Motor_VelCtrl_t vel_drawer_1;
Motor_VelCtrl_t vel_drawer_2;
Motor_VelCtrl_t vel_drawer_3;
Motor_VelCtrl_t vel_drawer_4;
Motor_VelCtrl_t vel_drawer_out;
Motor_PosCtrl_t pos_drawer_out;

// 用两条s曲线用以控制上升下降
SCurveTrajFollower_GroupItem_t items[4];
SCurveTrajFollower_Group_t     up;
SCurveTrajFollower_GroupItem_t down_items[4];
SCurveTrajFollower_Group_t     down;
SCurveTrajFollower_GroupItem_t items_out[1];
SCurveTrajFollower_Group_t     out;

// 测试用计数器
static uint32_t drawerup_count  = 0;
static uint32_t drawerout_count = 0;

static int             flagup_s_update = 0;              // 是否对s曲线更新（1：更新 || 0：不更新）
static DRAWER_STATUS_E drawer_status   = DRAWER_UNKNOWN; // 抽屉状态初始化

/* ===== 复位状态 ===== */
static uint8_t  reset_running       = 0;              // 复位是否正在进行
static uint32_t reset_start_ms      = 0;              // 复位起步开始时刻
static uint8_t  reset_stall_seen[4] = { 0, 0, 0, 0 }; // 堵转标志位
static uint32_t reset_stall_t0[4]   = { 0, 0, 0, 0 }; // 堵转开始时刻

static const DRAWER_PUSHOUT_PARAMS_T drawer_pushout_params = {
    .drawer_pushout_speed_abs        = 60.0f,   // 运行速度
    .drawer_pushout_start_vel_th     = 15.0f,   // 判定开始运动阈值
    .drawer_pushout_stall_vel_th     = 8.0f,    // 堵转速度判定阈值
    .drawer_pushout_stall_out_th     = 4000.0f, // 堵转电流判定阈值
    .drawer_pushout_min_run_ms       = 800u,    // 起步保护时间（起步后多少时间不进行堵转判定）
    .drawer_pushout_stall_confirm_ms = 1000u,   // 堵转确认时间
};

/* ===== 抽屉控制任务 =======*/
osThreadId_t         drawerUPHandle;
const osThreadAttr_t drawerUP_attributes = {
    .name       = "drawerUP",
    .stack_size = 128 * 8,
    .priority   = (osPriority_t) osPriorityHigh,
};

osThreadId_t         drawerOUTHandle;
const osThreadAttr_t drawerOUT_attributes = {
    .name       = "drawerOUT",
    .stack_size = 128 * 8,
    .priority   = (osPriority_t) osPriorityHigh,
};

// 抬升电机参数
const SCurveTrajFollower_GroupConfig_t config = {
    .item_count = 4,
    .items      = items,
    .item_configs =
            (SCurveTrajFollower_GroupItem_Config_t[]) {
                    {
                            .ctrl     = &vel_drawer_1,
                            .error_pd = { .Kp = 50.0f, .Kd = 0.5f, .abs_output_max = 60 },
                    },
                    {
                            .ctrl     = &vel_drawer_2,
                            .error_pd = { .Kp = 50.0f, .Kd = 0.5f, .abs_output_max = 60 },
                    },
                    {
                            .ctrl     = &vel_drawer_3,
                            .error_pd = { .Kp = 50.0f, .Kd = 0.5f, .abs_output_max = 60 },
                    },
                    {
                            .ctrl     = &vel_drawer_4,
                            .error_pd = { .Kp = 50.0f, .Kd = 0.5f, .abs_output_max = 60 },
                    },
            },
    .update_interval = 0.001f,
    .v_max           = 240.0f,
    .a_max           = 60.0f,
    .j_max           = 120.0f
};