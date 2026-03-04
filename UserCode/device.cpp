#include "device.hpp"
#include "can.h"
#include "dji.hpp"
#include "cmsis_os2.h"

motors::DJIMotor* drawer_1;
motors::DJIMotor* drawer_2;
motors::DJIMotor* drawer_3;
motors::DJIMotor* drawer_4;
motors::DJIMotor* motor_wheel[4];
motors::DJIMotor* drawer_out;

static void can_init()
{
    // CAN 初始化
    motors::DJIMotor::CAN_FilterInit(&hcan1, 0);
    CAN_RegisterCallback(&hcan1, motors::DJIMotor::CANBaseReceiveCallback);
    motors::DJIMotor::CAN_FilterInit(&hcan2, 14);
    CAN_RegisterCallback(&hcan2, motors::DJIMotor::CANBaseReceiveCallback);

    // 注册 CAN 主回调，并启动 CAN
    HAL_CAN_RegisterCallback(&hcan1, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, CAN_Fifo0ReceiveCallback);
    CAN_Start(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_RegisterCallback(&hcan2, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, CAN_Fifo0ReceiveCallback);
    CAN_Start(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

constexpr motors::DJIMotor::Config drawer_1_config = {
        .hcan    = &hcan2,
        .type    = motors::DJIMotor::Type::M3508_C620,
        .id1     = 1,
        .reverse = false,
};

constexpr motors::DJIMotor::Config drawer_2_config = {
        .hcan    = &hcan2,
        .type    = motors::DJIMotor::Type::M3508_C620,
        .id1     = 2,
        .reverse = false,
};

constexpr motors::DJIMotor::Config drawer_3_config = {
        .hcan    = &hcan2,
        .type    = motors::DJIMotor::Type::M3508_C620,
        .id1     = 3,
        .reverse = false,
};

constexpr motors::DJIMotor::Config drawer_4_config = {
        .hcan    = &hcan2,
        .type    = motors::DJIMotor::Type::M3508_C620,
        .id1     = 4,
        .reverse = false,
};

constexpr motors::DJIMotor::Config drawer_out_config = {
        .hcan    = &hcan2,
        .type    = motors::DJIMotor::Type::M3508_C620,
        .id1     = 5,
        .reverse = false,
};

constexpr motors::DJIMotor::Config motor_wheel_config[4] = {
    {
            .hcan    = &hcan1,
            .type    = motors::DJIMotor::Type::M3508_C620,
            .id1     = 1,
            .reverse = false,
    },
        {
            .hcan    = &hcan1,
            .type    = motors::DJIMotor::Type::M3508_C620,
            .id1     = 2,
            .reverse = false,
    },
        {
            .hcan    = &hcan1,
            .type    = motors::DJIMotor::Type::M3508_C620,
            .id1     = 3,
            .reverse = false,
    },
        {
            .hcan    = &hcan1,
            .type    = motors::DJIMotor::Type::M3508_C620,
            .id1     = 4,
            .reverse = false,
    }
};

static void wheel_motor_init()
{
    using namespace motors;
    for (size_t i = 0; i < 4; ++i)
        motor_wheel[i] = new DJIMotor(motor_wheel_config[i]);
}

static void drawer_motor_init()
{
    using namespace motors;
    drawer_1 = new DJIMotor(drawer_1_config);
    drawer_2 = new DJIMotor(drawer_2_config);
    drawer_3 = new DJIMotor(drawer_3_config);
    drawer_4 = new DJIMotor(drawer_4_config);
    drawer_out = new DJIMotor(drawer_out_config);
}

void APP_Device_Init()
{
    can_init();

    wheel_motor_init();
    drawer_motor_init();
}

void APP_Device_Update()
{
    motors::DJIMotor::SendIqCommand(&hcan1, motors::DJIMotor::IqSetCMDGroup::IqCMDGroup_1_4);
    motors::DJIMotor::SendIqCommand(&hcan2, motors::DJIMotor::IqSetCMDGroup::IqCMDGroup_5_8);
    motors::DJIMotor::SendIqCommand(&hcan2, motors::DJIMotor::IqSetCMDGroup::IqCMDGroup_5_8);
}

bool APP_Device_isAllConnected()
{
    bool all_connected = true;

    // check motors
    for (const auto& m : motor_wheel)
        all_connected &= m->isConnected();

    all_connected &= (drawer_1 != nullptr) && drawer_1->isConnected();
    all_connected &= (drawer_2 != nullptr) && drawer_2->isConnected();
    all_connected &= (drawer_3 != nullptr) && drawer_3->isConnected();
    all_connected &= (drawer_4 != nullptr) && drawer_4->isConnected();
    all_connected &= (drawer_out != nullptr) && drawer_out->isConnected();

    return all_connected;
}

void APP_Device_WaitConnections()
{
    while (!APP_Device_isAllConnected())
        osDelay(1);
}