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

#include "vision_lower_receive.hpp"
#include <string.h>
#include <stdio.h>
#include <ctype.h>

static int LR_Parse_Floats(const char* text, float* out, int max_count)
{
    int count = 0;
    const char* p = text;

    while (*p != '\0' && count < max_count)
    {
        // 跳过前导空白
        while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n')
        {
            p++;
        }

        char* endptr = NULL;
        float v = strtof(p, &endptr);
        if (endptr == p)
        {
            break;
        }
        out[count++] = v;
        p = endptr;

        // 跳过数字后空白
        while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n')
        {
            p++;
        }

        if (*p == ',')
        {
            p++;
            continue;
        }
        if (*p == '\0')
        {
            break;
        }

        // 非逗号分隔，判为格式结束
        break;
    }

    return count;
}

// ======================== 内部变量 ========================
static LR_DataTypeCallback g_datatype_cb = NULL;
static LR_Vector3          g_camera_to_body_offset = { 0.0f, 0.48f, 0.0f };// 视觉坐标系（相机）到机器人身体坐标系的偏移 单位米
static LR_Vector3          g_arm_to_body_offset    = { -0.60f, 0.60f, 0.0f };// 视觉坐标系（相机）到机器人身体坐标系的偏移 单位米

volatile uint32_t lr_diag_parse_ok_count   = 0;
volatile uint32_t lr_diag_parse_fail_count = 0;
volatile float    lr_diag_last_x           = 0.0f;
volatile float    lr_diag_last_y           = 0.0f;
volatile float    lr_diag_last_z           = 0.0f;
volatile float    lr_diag_last_yaw         = 0.0f;
volatile uint8_t  lr_diag_last_type        = 0;
volatile uint8_t  lr_diag_last_fail_stage  = 0;
volatile uint32_t lr_diag_last_raw_len     = 0;
volatile char     lr_diag_last_raw_frame[LR_RX_BUFFER_SIZE] = { 0 };

// 环形缓冲区变量（读/写索引 + 计数）
LR_DataPacket lr_detect_buffer[LR_DATA_MAX_NUM];
int           lr_detect_count     = 0; // 当前有效数据量
int           lr_detect_write_idx = 0; // 写索引（下一个要写入的位置）

LR_DataPacket lr_apriltag_buffer[LR_DATA_MAX_NUM];
int           lr_apriltag_count     = 0; // 当前有效数据量
int           lr_apriltag_write_idx = 0; // 写索引（下一个要写入的位置）

static char lr_rx_line[LR_RX_BUFFER_SIZE];
static int  lr_rx_line_pos = 0;

// 帧头/帧尾：逗号分隔格式
#define LR_FRAME_HEAD "AA," // 帧头：AA,
#define LR_FRAME_TAIL ",BB" // 帧尾：,BB

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

LR_Vector3 LR_Get_Camera_To_Body_Offset(void)
{
    return g_camera_to_body_offset;
}

LR_Vector3 LR_Get_Arm_To_Body_Offset(void)
{
    return g_arm_to_body_offset;
}

void LR_Convert_CameraPoint_To_Body(float cam_x, float cam_y, float cam_z,
                                    float* body_x, float* body_y, float* body_z)
{
    if (body_x)
        *body_x = cam_x + g_camera_to_body_offset.x;
    if (body_y)
        *body_y = cam_y + g_camera_to_body_offset.y;
    if (body_z)
        *body_z = cam_z + g_camera_to_body_offset.z;
}

void LR_Convert_CameraPoint_To_Arm(float cam_x, float cam_y, float cam_z,
                                   float* arm_x, float* arm_y, float* arm_z)
{
    float body_x = 0.0f;
    float body_y = 0.0f;
    float body_z = 0.0f;

    LR_Convert_CameraPoint_To_Body(cam_x, cam_y, cam_z, &body_x, &body_y, &body_z);
    if (arm_x)
        *arm_x = body_x - g_arm_to_body_offset.x;
    if (arm_y)
        *arm_y = body_y - g_arm_to_body_offset.y;
    if (arm_z)
        *arm_z = body_z - g_arm_to_body_offset.z;
}

LR_DataPacket LR_Convert_Packet_CameraToBody(const LR_DataPacket* cam_pkt)
{
    LR_DataPacket out = { 0 };
    if (!cam_pkt)
        return out;

    out = *cam_pkt;
    LR_Convert_CameraPoint_To_Body(cam_pkt->x, cam_pkt->y, cam_pkt->z, &out.x, &out.y, &out.z);
    return out;
}

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
    lr_detect_count     = 0;
    lr_detect_write_idx = 0;
    memset(lr_detect_buffer, 0, sizeof(lr_detect_buffer));

    lr_apriltag_count     = 0;
    lr_apriltag_write_idx = 0;
    memset(lr_apriltag_buffer, 0, sizeof(lr_apriltag_buffer));
}

// ======================== 数据帧解析 ========================
static void LR_Parse_Frame(const char* frame)
{
    size_t raw_len = 0;
    while (raw_len < (size_t)(LR_RX_BUFFER_SIZE - 1) && frame[raw_len] != '\0')
    {
        raw_len++;
    }
    lr_diag_last_raw_len = (uint32_t)raw_len;
    memcpy((void*)lr_diag_last_raw_frame, frame, raw_len);
    lr_diag_last_raw_frame[raw_len] = '\0';

    char normalized[LR_RX_BUFFER_SIZE];
    int  norm_pos = 0;
    for (int i = 0; frame[i] != '\0' && norm_pos < LR_RX_BUFFER_SIZE - 1; ++i)
    {
        const unsigned char ch = (unsigned char)frame[i];
        if (ch == '\r' || ch == '\n' || ch == ' ' || ch == '\t')
        {
            continue;
        }
        normalized[norm_pos++] = (char)toupper(ch);
    }
    normalized[norm_pos] = '\0';

    const char* start = strstr(normalized, LR_FRAME_HEAD); // 查找帧头 AA,
    const char* end   = strstr(normalized, LR_FRAME_TAIL); // 查找帧尾 ,BB
    int has_head_tail = 1;// 默认认为有帧头帧尾，若找不到则降级解析纯数值串
    if (!start || !end || end <= start)
    {
        // 兜底：允许直接发送纯数值串 "x,y,z,yaw" 或 "x,y,z,roll,pitch,yaw"
        has_head_tail = 0;
        start = normalized;
        end   = normalized + strlen(normalized);
    }

    if (has_head_tail)
    {
        start += 3; // 跳过帧头 AA,
    }

    int len = end - start ;
    if (len <= 0 || len >= LR_RX_BUFFER_SIZE)
    {
        lr_diag_parse_fail_count++;
        lr_diag_last_fail_stage = 2;
        return; // 长度异常
    }
    char content[LR_RX_BUFFER_SIZE];
    strncpy(content, start, len);
    content[len] = '\0';

    // 逗号分隔解析数值（strtof手动解析，避免sscanf在嵌入式下不稳定）
    LR_DataPacket pkt = { 0 };
    float values[6] = { 0.0f };
    int n = LR_Parse_Floats(content, values, 6);

    if (n == 4)
    {
        pkt.x = values[0];
        pkt.y = values[1];
        pkt.z = values[2];
        pkt.roll = values[3];
        pkt.has_rpy = 0;
        pkt.yaw     = pkt.roll; // detect格式，只有yaw（第4个值）

        lr_diag_parse_ok_count++;
        lr_diag_last_type = 0;
        lr_diag_last_x    = pkt.x;
        lr_diag_last_y    = pkt.y;
        lr_diag_last_z    = pkt.z;
        lr_diag_last_yaw  = pkt.yaw;
        lr_diag_last_fail_stage = 0;

        // 写入当前写索引位置
        lr_detect_buffer[lr_detect_write_idx] = pkt;
        // 更新写索引（循环：0→1→...→MAX-1→0）
        lr_detect_write_idx = (lr_detect_write_idx + 1) % LR_DATA_MAX_NUM;
        // 有效计数不超过最大值
        if (lr_detect_count < LR_DATA_MAX_NUM)
        {
            lr_detect_count++;
        }
        // （满员时：count保持MAX，写索引循环覆盖最旧数据）

        if (g_datatype_cb)
        {
            g_datatype_cb(0); // detect类型回调
        }
    }
    else if (n == 6)
    {
        pkt.x = values[0];
        pkt.y = values[1];
        pkt.z = values[2];
        pkt.roll = values[3];
        pkt.pitch = values[4];
        pkt.yaw = values[5];
        pkt.has_rpy = 1; // apriltag格式，含roll/pitch/yaw（4/5/6值）

        lr_diag_parse_ok_count++;
        lr_diag_last_type = 1;
        lr_diag_last_x    = pkt.x;
        lr_diag_last_y    = pkt.y;
        lr_diag_last_z    = pkt.z;
        lr_diag_last_yaw  = pkt.yaw;
        lr_diag_last_fail_stage = 0;

        // 写入当前写索引位置
        lr_apriltag_buffer[lr_apriltag_write_idx] = pkt;
        // 更新写索引（循环）
        lr_apriltag_write_idx = (lr_apriltag_write_idx + 1) % LR_DATA_MAX_NUM;
        // 有效计数不超过最大值
        if (lr_apriltag_count < LR_DATA_MAX_NUM)
        {
            lr_apriltag_count++;
        }

        if (g_datatype_cb)
        {
            g_datatype_cb(1); // apriltag类型回调
        }
    }
    else
    {
        lr_diag_parse_fail_count++;
        lr_diag_last_fail_stage = (strstr(normalized, LR_FRAME_HEAD) && strstr(normalized, LR_FRAME_TAIL)) ? 3 : 1;
        return; // 解析失败
    }
}



// ======================== 串口接收入口 ========================
void LR_Parse_And_Store(uint8_t byte)
{
    // 丢弃NUL，避免C字符串在中间被提前截断。
    if (byte == '\0')
    {
        return;
    }

    // 丢弃起始的孤立换行/回车，避免空帧被提前触发解析（例如流开头有噪声"\n,..."）
    if (lr_rx_line_pos == 0 && (byte == '\n' || byte == '\r'))
    {
        return;
    }

    if (lr_rx_line_pos < LR_RX_BUFFER_SIZE - 1)
    {
        lr_rx_line[lr_rx_line_pos++] = byte;
    }
    // 帧尾检测：\n 或 ,BB
    int parse = 0;
    if (byte == '\n' || lr_rx_line_pos >= LR_RX_BUFFER_SIZE - 1)
        parse = 1;
    else if (lr_rx_line_pos >= 3 && lr_rx_line[lr_rx_line_pos - 3] == ',' &&
             lr_rx_line[lr_rx_line_pos - 2] == 'B' && lr_rx_line[lr_rx_line_pos - 1] == 'B')
        parse = 1; // 检测到,BB
    if (parse)
    {
        lr_rx_line[lr_rx_line_pos] = '\0';

        // 如果缓冲中去除空白后没有有效字符，则视为噪声空帧，丢弃。
        int trimmed = 0;
        for (int i = 0; i < lr_rx_line_pos; ++i)
        {
            unsigned char c = (unsigned char)lr_rx_line[i];
            if (!isspace(c) && c != '\0')
            {
                trimmed = 1;
                break;
            }
        }
        if (!trimmed)
        {
            lr_rx_line_pos = 0;
            memset(lr_rx_line, 0, sizeof(lr_rx_line));
            return;
        }

        LR_Parse_Frame(lr_rx_line);
        lr_rx_line_pos = 0;
        memset(lr_rx_line, 0, sizeof(lr_rx_line));
    }
}