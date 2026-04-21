/**
 * @file lower_receive.c
 * @author Mburn
 * @date 2026-03-10
 * @brief 下位机串口数据接收与解析实现（环形缓冲区版）
 *
 * 实现串口数据的分帧、解析、环形缓存、类型识别与回调。
 * 支持 detect（AA,1.0,2.0,3.0,4.0,BB）和 apriltag（AA,1.0,2.0,3.0,4.0,5.0,6.0,BB）两种格式。
 * 缓存满时自动覆盖最旧数据，避免数据丢失。
 *
 * 典型用法：
 * 1. 在HAL_UART_RxCpltCallback等中断回调内调用LR_Parse_And_Store(byte)。
 * 2. 通过LR_Set_DataType_Callback注册回调，区分数据类型并驱动LED等。
 * 3. 通过全局缓存访问解析后的数据。
 */

#include <cmath>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include "vision_receive.hpp"

static uint8_t             g_lr_uart2_rx_byte = 0;
static LR_DataTypeCallback g_datatype_cb      = NULL;

static osThreadId_t         vision_parse_thread_handle   = NULL;
static osThreadId_t         vision_request_thread_handle = NULL;
static const osThreadAttr_t vision_parse_thread_attr     = {
    .name       = "vision_parse",
    .stack_size = 256 * 8,
    .priority   = (osPriority_t)osPriorityAboveNormal,
};
static const osThreadAttr_t vision_request_thread_attr = {
    .name       = "vision_req",
    .stack_size = 128 * 8,
    .priority   = (osPriority_t)osPriorityNormal,
};

#define LR_RX_RING_SIZE 256U
static uint8_t           g_rx_ring[LR_RX_RING_SIZE] = { 0 };
static volatile uint16_t g_rx_ring_head             = 0U;
static volatile uint16_t g_rx_ring_tail             = 0U;
static volatile uint32_t g_rx_ring_overflow_cnt     = 0U;

// 相机id请求定时器回调中使用，默认请求id=1
static volatile uint8_t g_request_camera_id = 0x01U;

volatile uint32_t vision_rx_irq_cnt    = 0;
volatile uint32_t vision_rx_byte_cnt   = 0;
volatile uint32_t vision_fail_cnt      = 0;
volatile uint32_t vision_err_cnt       = 0;
volatile uint32_t vision_last_err_code = 0;

volatile uint32_t lr_diag_parse_ok_count                    = 0;
volatile uint32_t lr_diag_parse_fail_count                  = 0;
volatile float    lr_diag_last_x                            = 0.0f;
volatile float    lr_diag_last_y                            = 0.0f;
volatile float    lr_diag_last_z                            = 0.0f;
volatile float    lr_diag_last_yaw                          = 0.0f;
volatile uint8_t  lr_diag_last_type                         = 0;
volatile uint8_t  lr_diag_last_fail_stage                   = 0;
volatile uint32_t lr_diag_last_raw_len                      = 0;
volatile char     lr_diag_last_raw_frame[LR_RX_BUFFER_SIZE] = { 0 };
volatile uint8_t  lr_diag_last_status                       = 0;

// 环形缓冲区操作：推入一个字节，成功返回true，失败（满）返回false
static bool LR_RingPushByte(uint8_t byte)
{
    const uint16_t head      = g_rx_ring_head;
    const uint16_t next_head = (uint16_t)((head + 1U) % LR_RX_RING_SIZE);
    if (next_head == g_rx_ring_tail)
    {
        return false;
    }

    g_rx_ring[head] = byte;
    g_rx_ring_head  = next_head;
    return true;
}

// 环形缓冲区操作：弹出一个字节，成功返回true并通过out参数输出，失败（空）返回false
static bool LR_RingPopByte(uint8_t* out)
{
    if (!out)
    {
        return false;
    }

    const uint16_t tail = g_rx_ring_tail;
    if (tail == g_rx_ring_head)
    {
        return false;
    }

    *out           = g_rx_ring[tail];
    g_rx_ring_tail = (uint16_t)((tail + 1U) % LR_RX_RING_SIZE);
    return true;
}

// 串口接受线程：持续从环形缓冲区读取字节并解析
static void VisionParseTask(void* argument)
{
    (void)argument;
    for (;;)
    {
        uint8_t byte = 0U;
        if (LR_RingPopByte(&byte))
        {
            LR_Parse_And_Store(byte);
            continue;
        }

        osDelay(1);
    }
}

// 定时请求线程：每秒发送一次请求字节，请求相应的相机数据
static void VisionRequestTask(void* argument)
{
    (void)argument;
    for (;;)
    {
        const uint8_t req = g_request_camera_id;
        if (HAL_UART_Transmit(&huart2, (uint8_t*)&req, 1U, 10U) != HAL_OK)
        {
            vision_fail_cnt++;
        }
        osDelay(1000);
    }
}

// ======================== 接口实现 ========================
void CammeraReceive_Init(void)
{
    g_rx_ring_head         = 0U;
    g_rx_ring_tail         = 0U;
    g_rx_ring_overflow_cnt = 0U;

    if (!vision_parse_thread_handle)
    {
        vision_parse_thread_handle = osThreadNew(VisionParseTask, NULL, &vision_parse_thread_attr);
    }

    if (!vision_request_thread_handle)
    {
        vision_request_thread_handle =
                osThreadNew(VisionRequestTask, NULL, &vision_request_thread_attr);
    }

    if (HAL_UART_Receive_IT(&huart2, &g_lr_uart2_rx_byte, 1) != HAL_OK)
    {
        vision_fail_cnt++;
    }
}

bool CammeraReceive_OnRxCplt(UART_HandleTypeDef* huart)
{
    if (huart->Instance != USART2)
    {
        return false;
    }

    const uint8_t rx_byte = g_lr_uart2_rx_byte;
    vision_rx_irq_cnt++;
    vision_rx_byte_cnt++;

    // 先重启接收，尽量缩短无保护窗口，避免连续字节导致ORE。
    if (HAL_UART_Receive_IT(&huart2, &g_lr_uart2_rx_byte, 1) != HAL_OK)
    {
        vision_fail_cnt++;
        return true;
    }

    if (!LR_RingPushByte(rx_byte))
    {
        g_rx_ring_overflow_cnt++;
        vision_fail_cnt++;
    }

    return true;
}

bool CammeraReceive_OnError(UART_HandleTypeDef* huart)
{
    if (huart->Instance != USART2)
    {
        return false;
    }

    vision_err_cnt++;
    vision_last_err_code = huart->ErrorCode;

    // 清除常见UART错误标志，避免错误中断反复触发导致接收回调停滞。
    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);

    if (HAL_UART_Receive_IT(&huart2, &g_lr_uart2_rx_byte, 1) != HAL_OK)
    {
        vision_fail_cnt++;
    }

    return true;
}

// CRC8计算，输入为数据部分（不包含帧头AA和帧尾BB），输出为CRC8校验码
static uint8_t LR_CRC8_Payload(const uint8_t* data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x80)
            {
                crc = (uint8_t)((crc << 1) ^ 0x07U);
            }
            else
            {
                crc <<= 1;
            }
        }
    }
    return crc;
}

void LR_Set_RequestCameraId(uint8_t camera_id)
{
    g_request_camera_id = camera_id;
}

// 小端序浮点数编码/解码（IEEE 754单精度）
static void LR_EncodeFloatLE(float value, uint8_t* out4)
{
    uint32_t bits = 0;
    memcpy(&bits, &value, sizeof(bits));
    out4[0] = (uint8_t)(bits & 0xFFU);
    out4[1] = (uint8_t)((bits >> 8) & 0xFFU);
    out4[2] = (uint8_t)((bits >> 16) & 0xFFU);
    out4[3] = (uint8_t)((bits >> 24) & 0xFFU);
}

// 小端序浮点数解码（IEEE 754单精度）
static float LR_DecodeFloatLE(const uint8_t* in4)
{
    const uint32_t bits  = ((uint32_t)in4[0]) | ((uint32_t)in4[1] << 8) | ((uint32_t)in4[2] << 16) |
                           ((uint32_t)in4[3] << 24);
    float          value = 0.0f;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

// 将成功解析的数据包存入detect环形缓冲区，并更新相关计数和序号
static void LR_PushDetectPacket(const LR_DataPacket* pkt)
{
    if (!pkt)
    {
        return;
    }

    lr_detect_buffer[lr_detect_write_idx] = *pkt;
    lr_detect_write_idx                   = (lr_detect_write_idx + 1) % LR_DATA_MAX_NUM;
    if (lr_detect_count < LR_DATA_MAX_NUM)
    {
        lr_detect_count++;
    }
    lr_detect_update_seq++;

    if (g_datatype_cb)
    {
        g_datatype_cb(0);
    }
}

// ======================== 内部变量 ========================
static LR_Vector3 g_camera_to_body_offset = { 0.28f,
                                              0.0f,
                                              0.0f }; // 视觉坐标系（相机）到机器人身体坐标系的偏移
// 单位米x方向前正，y方向左正，z方向上正
static LR_Vector3 g_arm_to_body_offset = { 0.83f,
                                           0.35f,
                                           0.0f }; // 视觉坐标系（相机）到机器人身体坐标系的偏移
                                                   // 单位米x方向前正，y方向左正，z方向上正
static int yaw_camera_to_body_deg =
        0; // 视觉坐标系（相机）到机器人身体坐标系的偏移
           // 角度（单位度，正值表示相机坐标系相对于身体坐标系逆时针旋转）
static int yaw_arm_to_body_deg =
        0; // 机械臂坐标系到机器人身体坐标系的偏移
           // 角度（单位度，正值表示机械臂坐标系相对于身体坐标系逆时针旋转）
constexpr float PI = 3.14159265358979323846f;
// 已移除测试用全局诊断变量，生产/发布时请使用更轻量的日志或调试接口。

// 环形缓冲区变量（读/写索引 + 计数）
LR_DataPacket     lr_detect_buffer[LR_DATA_MAX_NUM];
int               lr_detect_count      = 0; // 当前有效数据量
int               lr_detect_write_idx  = 0; // 写索引（下一个要写入的位置）
volatile uint32_t lr_detect_update_seq = 0;

LR_DataPacket     lr_apriltag_buffer[LR_DATA_MAX_NUM];
int               lr_apriltag_count      = 0; // 当前有效数据量
int               lr_apriltag_write_idx  = 0; // 写索引（下一个要写入的位置）
volatile uint32_t lr_apriltag_update_seq = 0;

static uint8_t g_frame_buf[LR_VISION_FRAME_SIZE] = { 0 };
static uint8_t g_frame_pos                       = 0;

// ======================== 回调注册 ========================
void LR_Set_DataType_Callback(LR_DataTypeCallback cb)
{
    g_datatype_cb = cb;
}

void LR_Set_Camera_To_Body_Offset(float x, float y, float z)
{
    g_camera_to_body_offset.x = x;
    g_camera_to_body_offset.y = y;
    g_camera_to_body_offset.z = z;
}

void LR_Set_Arm_To_Body_Offset(float x, float y, float z)
{
    g_arm_to_body_offset.x = x;
    g_arm_to_body_offset.y = y;
    g_arm_to_body_offset.z = z;
}
// ======================== 主要功能实现 ========================
// target解算
void LR_Compute_Target(
        float x, float y, float z, float yaw, float* target_x, float* target_y, float* target_yaw)
{
    float camera_x   = 0.0f;
    float camera_y   = 0.0f;
    float camera_yaw = 0.0f;

    // 位置受到坐标轴颠倒和摄像头于机械臂位置关系的影响
    camera_x   = x;
    camera_y   = LR_CAMERA_REVERSED ? -y : y;
    camera_yaw = LR_CAMERA_REVERSED ? -yaw : yaw;

    // 先换算相机到车体坐标系
    float body_x = camera_x + g_camera_to_body_offset.x;
    float body_y = camera_y + g_camera_to_body_offset.y;

    float target_in_body_x = body_x - g_arm_to_body_offset.x * cosf(PI * camera_yaw / 180.0f) +
                             g_arm_to_body_offset.y * sinf(PI * camera_yaw / 180.0f);
    float target_in_body_y = body_y - g_arm_to_body_offset.x * sinf(PI * camera_yaw / 180.0f) -
                             g_arm_to_body_offset.y * cosf(PI * camera_yaw / 180.0f);

    *target_x   = target_in_body_x;
    *target_y   = target_in_body_y;
    *target_yaw = camera_yaw;
}
// 将相机坐标系下的点转换为车体坐标系下的点
LR_DataPacket LR_Convert_Packet_CameraToBody(const LR_DataPacket* cam_pkt)
{
    LR_DataPacket out = { 0 };
    if (!cam_pkt)
        return out;

    out = *cam_pkt;
    LR_Convert_CameraPoint_To_Body(cam_pkt->x, cam_pkt->y, cam_pkt->z, &out.x, &out.y, &out.z);
    return out;
}
// 将数据包中的位置从相机基准转换为机械臂基准（姿态字段保持不变）
LR_DataPacket LR_Convert_Packet_CameraToArm(const LR_DataPacket* cam_pkt)
{
    LR_DataPacket out = { 0 };
    if (!cam_pkt)
        return out;

    out = *cam_pkt;
    LR_Convert_CameraPoint_To_Arm(cam_pkt->x, cam_pkt->y, cam_pkt->z, &out.x, &out.y, &out.z);
    return out;
}

// ======================== 数据缓存清空 ========================
void LR_Clear_Data_Buffer(void)
{
    // 清空环形缓冲区所有状态
    lr_detect_count      = 0;
    lr_detect_write_idx  = 0;
    lr_detect_update_seq = 0;
    memset(lr_detect_buffer, 0, sizeof(lr_detect_buffer));

    lr_apriltag_count      = 0;
    lr_apriltag_write_idx  = 0;
    lr_apriltag_update_seq = 0;
    memset(lr_apriltag_buffer, 0, sizeof(lr_apriltag_buffer));

    g_frame_pos = 0;
    memset(g_frame_buf, 0, sizeof(g_frame_buf));

    g_rx_ring_head         = 0U;
    g_rx_ring_tail         = 0U;
    g_rx_ring_overflow_cnt = 0U;
    memset(g_rx_ring, 0, sizeof(g_rx_ring));
}

bool LR_Send_Frame(float x, float y, float yaw, uint8_t status)
{
    uint8_t frame[LR_VISION_FRAME_SIZE] = { 0 };

    frame[0] = LR_VISION_FRAME_HEADER;
    LR_EncodeFloatLE(x, &frame[1]);
    LR_EncodeFloatLE(y, &frame[5]);
    LR_EncodeFloatLE(yaw, &frame[9]);
    frame[13] = status;
    frame[14] = LR_CRC8_Payload(&frame[1], 13);

    return HAL_UART_Transmit(&huart2, frame, LR_VISION_FRAME_SIZE, 10) == HAL_OK;
}

// ======================== 串口接收入口 ========================
void LR_Parse_And_Store(uint8_t byte)
{
    if (g_frame_pos == 0U)
    {
        if (byte == LR_VISION_FRAME_HEADER)
        {
            g_frame_buf[0] = byte;
            g_frame_pos    = 1U;
        }
        return;
    }

    g_frame_buf[g_frame_pos++] = byte;
    if (g_frame_pos < LR_VISION_FRAME_SIZE)
    {
        return;
    }

    const uint8_t expected_crc = LR_CRC8_Payload(&g_frame_buf[1], 13);
    if (expected_crc != g_frame_buf[14])
    {
        lr_diag_parse_fail_count++;
        lr_diag_last_fail_stage = 4; // crc mismatch
        vision_fail_cnt++;

        if (byte == LR_VISION_FRAME_HEADER)
        {
            g_frame_buf[0] = byte;
            g_frame_pos    = 1U;
        }
        else
        {
            g_frame_pos = 0U;
        }
        return;
    }

    LR_DataPacket pkt = { 0 };
    pkt.x             = LR_DecodeFloatLE(&g_frame_buf[1]);
    pkt.y             = LR_DecodeFloatLE(&g_frame_buf[5]);
    pkt.z             = 0.0f;
    pkt.yaw           = LR_DecodeFloatLE(&g_frame_buf[9]);
    pkt.roll          = pkt.yaw;
    pkt.pitch         = 0.0f;
    pkt.has_rpy       = 0;

    lr_diag_parse_ok_count++;
    lr_diag_last_x       = pkt.x;
    lr_diag_last_y       = pkt.y;
    lr_diag_last_z       = pkt.z;
    lr_diag_last_yaw     = pkt.yaw;
    lr_diag_last_status  = g_frame_buf[13];
    lr_diag_last_type    = 0;
    lr_diag_last_raw_len = LR_VISION_FRAME_SIZE;

    LR_PushDetectPacket(&pkt);
    g_frame_pos = 0U;
}
