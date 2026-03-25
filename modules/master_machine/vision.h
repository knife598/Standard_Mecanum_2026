#ifndef VISION_H
#define VISION_H

#include "bsp_usart.h"
#include <stdint.h>

#pragma pack(1)
typedef struct __attribute__((packed))
{
    uint8_t head[2]; // 固定为'S','P'
    uint8_t mode; // 0: 不控制, 1: 控制云台但不开火，2: 控制云台且开火
    float yaw;
    float yaw_vel;
    float yaw_acc;
    float pitch;
    float pitch_vel;
    float pitch_acc;
    uint16_t crc16;
} VisionToGimbal_s;

typedef struct __attribute__((packed))
{
    uint8_t head[2]; // 固定为'S','P'
    uint8_t mode; // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
    float q[4];   // wxyz顺序
    float yaw;
    float yaw_vel;
    float pitch;
    float pitch_vel;
    float bullet_speed;
    uint16_t bullet_count; // 子弹累计发送次数, uint16_t按协议自然回绕
    uint16_t crc16;
} GimbalToVision_s;
#pragma pack()

// 兼容历史接口命名: Gimbal板接收来自Vision的数据,发送给Vision的数据
typedef VisionToGimbal_s Vision_Recv_s;
typedef GimbalToVision_s Vision_Send_s;

#define VISION_RECV_SIZE ((uint16_t)sizeof(VisionToGimbal_s))
#define VISION_SEND_SIZE ((uint16_t)sizeof(GimbalToVision_s))

_Static_assert(sizeof(GimbalToVision_s) <= 64, "GimbalToVision_s must be <=64 bytes");
_Static_assert(sizeof(VisionToGimbal_s) <= 64, "VisionToGimbal_s must be <=64 bytes");
// 64字节上限来自协议要求，需保证单帧长度不超过通信链路约束。

/**
 * @brief 调用此函数初始化和视觉的串口通信
 *
 * @param handle 用于和视觉通信的串口handle(C板上一般为USART1,丝印为USART2,4pin)
 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle);

/**
 * @brief 发送视觉数据
 *
 */
void VisionSend();

/**
 * @brief 设置发送数据的姿态部分
 *
 * @param yaw
 * @param pitch
 */
void VisionSetAltitude(float yaw, float pitch, float roll);

#endif // !VISION_H
