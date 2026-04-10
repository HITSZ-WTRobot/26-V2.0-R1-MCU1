#include "can.h"
#include "chassis.hpp"
#include "cmsis_os2.h"
#include "controller_receive.hpp"
#include "device.hpp"
#include "drawer.hpp"
#include "eventflags.hpp"
#include "interboard_comm.hpp"
#include "stm32f4xx_hal_uart.h"
#include "tim.h"
#include "vision_receive.hpp"

Drawer*            drawer           = nullptr;
static osTimerId_t drawer_timHandle = nullptr;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    if (ControllerReceive_OnRxCplt(huart))
    {
        return;
    }
    // 板间通信串口接收回调
    if (InterboardComm_OnRxCplt(huart))
    {
        return;
    }
    // 视觉串口接收回调
    (void)CammeraReceive_OnRxCplt(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
    if (ControllerReceive_OnError(huart))
    {
        return;
    }

    if (InterboardComm_OnError(huart))
    {
        return;
    }

    (void)CammeraReceive_OnError(huart);
}

extern "C" void Drawer_softTIM(void* argument)
{
    (void)argument;
    if (drawer)
        drawer->softTIM();
}

void TIM_Callback_1kHz_1(TIM_HandleTypeDef* htim)
{
    (void)htim;

    static uint32_t tick_1kHz = 0;
    ++tick_1kHz;

    if (drawer)
        drawer->update_1kHz();
    APP_Chassis_Update_1kHz();

    APP_Device_Update();
    service::Watchdog::EatAll();
}

void APP_Drawer_BeforeUpdate()
{
    drawer = new Drawer();
}

extern "C" void Init(void* argument)
{
    (void)argument;

    flags_create();
    APP_Device_Init();
    InterboardComm_Init();
    Controller_receiver_Init();
    CammeraReceive_Init();

    APP_Chassis_BeforeUpdate();
    APP_Drawer_BeforeUpdate();

    if (service::Watchdog::isFull())
        Error_Handler();

    drawer_timHandle = osTimerNew(Drawer_softTIM, osTimerPeriodic, NULL, NULL);
    osTimerStart(drawer_timHandle, 10);

    HAL_TIM_RegisterCallback(&htim6, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM_Callback_1kHz_1);
    HAL_TIM_Base_Start_IT(&htim6);

    HAL_TIM_RegisterCallback(&htim10, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM10_Callback);
    HAL_TIM_Base_Start_IT(&htim10);

    // APP_Device_WaitConnections();
    osDelay(2000);

    APP_Chassis_Init();
    osThreadExit();
}
