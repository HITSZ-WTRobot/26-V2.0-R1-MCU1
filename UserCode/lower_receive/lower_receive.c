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

#include "lower_receive.h"
#include <string.h>
#include <stdio.h>

// ======================== 内部变量 ========================
static LR_DataTypeCallback g_datatype_cb = NULL;

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
    const char* start = strstr(frame, LR_FRAME_HEAD); // 查找帧头 AA,
    const char* end   = strstr(frame, LR_FRAME_TAIL); // 查找帧尾 ,BB
    if (!start || !end || end <= start)
        return;                     // 格式不符
    start += strlen(LR_FRAME_HEAD); // 跳过帧头 AA,
    int len = end - start;
    if (len <= 0 || len >= LR_RX_BUFFER_SIZE)
        return; // 长度异常
    char content[LR_RX_BUFFER_SIZE];
    strncpy(content, start, len);
    content[len] = '\0';

    // 逗号分隔解析数值
    LR_DataPacket pkt = { 0 };
    int           n   = sscanf(
            content, "%f,%f,%f,%f,%f,%f", &pkt.x, &pkt.y, &pkt.z, &pkt.roll, &pkt.pitch, &pkt.yaw);

    if (n == 4)
    {
        pkt.has_rpy = 0;
        pkt.yaw     = pkt.roll; // detect格式，只有yaw（第4个值）

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
        printf("Parsed detect: x=%.2f, y=%.2f, z=%.2f, yaw=%.2f\n", pkt.x, pkt.y, pkt.z, pkt.yaw);
    }
    else if (n == 6)
    {
        pkt.has_rpy = 1; // apriltag格式，含roll/pitch/yaw（4/5/6值）

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
        return; // 解析失败
    }
}

// ======================== 串口接收入口 ========================
void LR_Parse_And_Store(uint8_t byte)
{
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
        LR_Parse_Frame(lr_rx_line);
        lr_rx_line_pos = 0;
        memset(lr_rx_line, 0, sizeof(lr_rx_line));
    }
}