#include "controller_receive.hpp"
#include "interboard_comm.hpp"
#include "vision_receive.hpp"

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    if (ControllerReceive_OnRxCplt(huart))
    {
        return;
    }

    if (InterboardComm_OnRxCplt(huart))
    {
        return;
    }

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
