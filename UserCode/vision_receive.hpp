/**
 * @file lower_receive.h
 * @author Mburn
 * @date 2026-03-10
 * @brief 下位机串口数据收发与解析模块头文件
 *
 * 提供视觉串口固定二进制帧收发接口与目标缓存。
 * 帧格式：0xAA + float32(x) + float32(y) + float32(yaw) + uint8(status) + uint8(crc8)
 * float 按小端序发送，CRC8 覆盖 x/y/yaw/status（即 header 之后的 13 字节）。
 *
 * 用法：
 * 1. 在串口接收中断中调用 LR_Parse_And_Store(byte) 实现自动分帧与解析。
 * 2. 可通过 LR_Set_DataType_Callback 注册回调，区分数据类型。
 * 3. 通过全局缓存访问解析后的数据（始终保留最新的LR_DATA_MAX_NUM条）。
 */
#ifndef __LOWER_RECEIVE_H__
#define __LOWER_RECEIVE_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdlib.h> // 为 strtof
#include <ctype.h>  // 为 isspace
#include <stdint.h>
#include "gpio.h"

// ======================== 配置参数 ========================

// 串口接收缓冲区长度（仅用于诊断字符串缓存）
#define LR_RX_BUFFER_SIZE 128
// 可缓存的数据包最大数量（环形缓冲区大小）
#define LR_DATA_MAX_NUM 10
// 摄像头倒立正立，0=正立，1=倒立（影响坐标转换）
#define LR_CAMERA_REVERSED     1
#define LR_VISION_FRAME_HEADER 0xAAU
#define LR_VISION_FRAME_SIZE   15U

// ======================== 数据结构 ========================
/**
 * @brief 通用数据包结构体
 * @note 支持 detect（AA,1.0,2.0,3.0,4.0,BB）和 apriltag（AA,1.0,2.0,3.0,4.0,5.0,6.0,BB）两种格式
 * @param has_rpy 1:含roll/pitch/yaw，0:仅有yaw
 */
typedef struct
{
    float x, y, z;          // 位置
    float roll, pitch, yaw; // 姿态角（apriltag时有效）
    int   has_rpy;          // 1:有rpy, 0:只有yaw
} LR_DataPacket;

typedef struct
{
    float x;
    float y;
    float z;
} LR_Vector3;

// ======================== 全局变量 ========================
// detect格式环形缓存（AA,1.0,2.0,3.0,4.0,BB）
extern LR_DataPacket lr_detect_buffer[LR_DATA_MAX_NUM];
extern int           lr_detect_count;     // 当前有效数据量（≤LR_DATA_MAX_NUM）
extern int           lr_detect_write_idx; // 下一个写入位置的索引

// apriltag格式环形缓存（AA,1.0,2.0,3.0,4.0,5.0,6.0,BB）
extern LR_DataPacket lr_apriltag_buffer[LR_DATA_MAX_NUM];
extern int           lr_apriltag_count;     // 当前有效数据量（≤LR_DATA_MAX_NUM）
extern int           lr_apriltag_write_idx; // 下一个写入位置的索引

// 数据更新序号：每次对应类型成功解析后自增，可用于判断“最新到达的是哪一类数据”。
extern volatile uint32_t lr_detect_update_seq;
extern volatile uint32_t lr_apriltag_update_seq;

// 解析最小诊断：用于确认串口数据是否被正确解析入库
extern volatile uint32_t lr_diag_parse_ok_count;
extern volatile uint32_t lr_diag_parse_fail_count;
extern volatile float    lr_diag_last_x;
extern volatile float    lr_diag_last_y;
extern volatile float    lr_diag_last_z;
extern volatile float    lr_diag_last_yaw;
extern volatile uint8_t  lr_diag_last_type;       // 0=detect, 1=apriltag
extern volatile uint8_t  lr_diag_last_fail_stage; // 1=head/tail, 2=len, 3=number-parse
extern volatile uint32_t lr_diag_last_raw_len;
extern volatile char     lr_diag_last_raw_frame[LR_RX_BUFFER_SIZE];
extern volatile uint8_t  lr_diag_last_status; // 最近一次成功解析的状态位

// ======================== 接口函数 ========================

#include "usart.h"
#include "cmsis_os2.h"
#include <stdbool.h>
#include <stdint.h>

// USART2视觉串口最小诊断计数器（可在调试器Watch窗口直接观察）
extern volatile uint32_t vision_rx_irq_cnt;
extern volatile uint32_t vision_rx_byte_cnt;
extern volatile uint32_t vision_fail_cnt;
extern volatile uint32_t vision_err_cnt;
extern volatile uint32_t vision_last_err_code;

void CammeraReceive_Init(void);
bool CammeraReceive_OnRxCplt(UART_HandleTypeDef* huart);
bool CammeraReceive_OnError(UART_HandleTypeDef* huart);

/**
 * @brief 串口接收中断中调用，按固定二进制帧协议逐字节解析
 * @param byte 新接收到的字节
 * @note 推荐在HAL_UART_RxCpltCallback等中断回调内调用
 */
void LR_Parse_And_Store(uint8_t byte);

/**
 * @brief 按固定二进制帧协议发送一帧视觉数据
 * @param x 目标 x
 * @param y 目标 y
 * @param yaw 目标 yaw
 * @param status 8位状态位
 * @return true 发送调用成功；false 发送失败
 */
bool LR_Send_Frame(float x, float y, float yaw, uint8_t status);

/**
 * @brief 设置1Hz请求发送的相机ID字节
 */
void LR_Set_RequestCameraId(uint8_t camera_id);

/**
 * @brief 清空所有已接收并解析的数据包缓存
 * @note 清除detect和apriltag两类缓存、计数及写索引
 */
void LR_Clear_Data_Buffer(void);

/**
 * @brief 设置数据类型回调函数（0=detect, 1=apriltag）
 * @param cb 回调函数指针，参数type=0为detect，1为apriltag
 * @note 每次成功解析一帧数据后自动调用（无论是否覆盖旧数据）
 */
typedef void (*LR_DataTypeCallback)(int type);
void LR_Set_DataType_Callback(LR_DataTypeCallback cb);

/**
 * @brief 设置相机中心相对车体中心的位置（单位与解算结果一致）
 */
void LR_Set_Camera_To_Body_Offset(float x, float y, float z);

/**
 * @brief 设置机械臂基座相对车体中心的位置（单位与解算结果一致）
 */
void LR_Set_Arm_To_Body_Offset(float x, float y, float z);

/**
 * @brief 读取当前相机相对车体中心的位置参数
 */
LR_Vector3 LR_Get_Camera_To_Body_Offset(void);

/**
 * @brief 读取当前机械臂相对车体中心的位置参数
 */
LR_Vector3 LR_Get_Arm_To_Body_Offset(void);

/**
 * @brief 将相机坐标系下点转换为车体坐标系下点
 */
void LR_Convert_CameraPoint_To_Body(
        float cam_x, float cam_y, float cam_z, float* body_x, float* body_y, float* body_z);

/**
 * @brief 将相机坐标系下点转换为机械臂坐标系下点
 */
void LR_Convert_CameraPoint_To_Arm(
        float cam_x, float cam_y, float cam_z, float* arm_x, float* arm_y, float* arm_z);

/**
 * @brief 将相机坐标系下的yaw角转换为车体坐标系下的yaw角
 */
void LR_Convert_Camerayaw_To_Body(float cam_yaw_deg, float* body_yaw_deg);

/**
 * @brief 将相机坐标系下的yaw角转换为机械臂坐标系下的yaw角
 */
void LR_Convert_Camerayaw_To_Arm(float cam_yaw_deg, float* arm_yaw_deg);

/**
 @brief 解算位置后的目标位置
 */

void LR_Compute_Target(
        float x, float y, float z, float yaw, float* target_x, float* target_y, float* target_yaw);

/**
 * @brief 将数据包中的位置从相机基准转换为车体基准（姿态字段保持不变）
 */
LR_DataPacket LR_Convert_Packet_CameraToBody(const LR_DataPacket* cam_pkt);

/**
 * @brief 将数据包中的位置从相机基准转换为机械臂基准（姿态字段保持不变）
 */
LR_DataPacket LR_Convert_Packet_CameraToArm(const LR_DataPacket* cam_pkt);

void CammeraReceive_Init(void);

#ifdef __cplusplus
}
#endif
#endif