/**
 * @file master_process.c
 * @author neozng
 * @brief  module for recv&send vision data
 * @version beta
 * @date 2022-11-03
 * @todo 增加对串口调试助手协议的支持,包括vofa和serial debug
 * @copyright Copyright (c) 2022
 *
 */
#include "master_process.h"
#include "vision.h"
#include "daemon.h"
#include "bsp_log.h"
#include "robot_def.h"
#include "fifo.h"
#include "crc16.h"
#include <string.h>
#include <stddef.h>
static Vision_Recv_s recv_data;
static Vision_Send_s send_data;
static DaemonInstance *vision_daemon_instance;
static USARTInstance *vision_usart_instance;

static uint8_t VisionPacketIsValid(const uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len != sizeof(VisionToGimbal_s))
        return 0;

    if (buf[0] != 'S' || buf[1] != 'P')
        return 0;

    uint16_t expected_crc = crc_16(buf, offsetof(VisionToGimbal_s, crc16));
    uint16_t recv_crc = 0;
    memcpy(&recv_crc, &buf[offsetof(VisionToGimbal_s, crc16)], sizeof(uint16_t));
    return expected_crc == recv_crc;
}

static void VisionUpdateSendCrc(void)
{
    send_data.head[0] = 'S';
    send_data.head[1] = 'P';
    // 协议字段为uint16_t, 按协议自然回绕计数
    send_data.bullet_count++;
    send_data.crc16 = crc_16((uint8_t *)&send_data, offsetof(GimbalToVision_s, crc16));
}
/**
 * @brief 离线回调函数,将在daemon.c中被daemon task调用
 * @attention 由于HAL库的设计问题,串口开启DMA接收之后同时发送有概率出现__HAL_LOCK()导致的死锁,使得无法
 *            进入接收中断.通过daemon判断数据更新,重新调用服务启动函数以解决此问题.
 *
 * @param id vision_usart_instance的地址,此处没用.
 */
static void VisionOfflineCallback(void *id)
{
#ifdef VISION_USE_UART
    USARTServiceInit(vision_usart_instance);
#endif // !VISION_USE_UART
    LOGWARNING("[vision] vision offline, restart communication.");
    USBRefresh();
}

#ifdef VISION_USE_UART

#include "bsp_usart.h"



/**
 * @brief 接收解包回调函数,将在bsp_usart.c中被usart rx callback调用
 * @todo  1.提高可读性,将get_protocol_info的第四个参数增加一个float类型buffer
 *        2.添加标志位解码
 */
static void DecodeVision()
{
    DaemonReload(vision_daemon_instance); // 喂狗
    if (VisionPacketIsValid(vision_usart_instance->recv_buff, VISION_RECV_SIZE))
    {
        memcpy(&recv_data, vision_usart_instance->recv_buff, sizeof(VisionToGimbal_s));
    }
}

// Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
// {
//     USART_Init_Config_s conf;
//     conf.module_callback = DecodeVision;
//     conf.recv_buff_size = VISION_RECV_SIZE;
//     conf.usart_handle = _handle;
//     vision_usart_instance = USARTRegister(&conf);

//     // 为master process注册daemon,用于判断视觉通信是否离线
//     Daemon_Init_Config_s daemon_conf = {
//         .callback = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
//         .owner_id = vision_usart_instance,
//         .reload_count = 10,
//     };
//     vision_daemon_instance = DaemonRegister(&daemon_conf);

//     return &recv_data;
// }

/**
 * @brief 发送函数
 *
 * @param send 待发送数据
 *
 */
void VisionSend()
{
    VisionUpdateSendCrc();
    USARTSend(vision_usart_instance, (uint8_t *)&send_data, sizeof(GimbalToVision_s), USART_TRANSFER_DMA); // 和视觉通信使用DMA
    // 此处为HAL设计的缺陷,DMASTOP会停止发送和接收,导致再也无法进入接收中断.
    // 可在发送完成中断中重新启动DMA接收,但较为复杂.
    // 这里保持DMA发送,由daemon机制兜底离线重启.
}

#endif // VISION_USE_UART

#ifdef VISION_USE_VCP

#include "bsp_usb.h"
static uint8_t *vis_recv_buff;
int fifo_test;
static void DecodeVision(uint16_t recv_len)
{
    UNUSED(recv_len);
    if (VisionPacketIsValid(vis_recv_buff, VISION_RECV_SIZE))
    {
        memcpy(&recv_data, vis_recv_buff, sizeof(VisionToGimbal_s));
    }
}
// static void FIFO_WRITE(uint16_t recv_len){
//     WritePacketToFIFO(&usb_fifo,vis_recv_buff,11);
// }
/* 视觉通信初始化 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    UNUSED(_handle); // 仅为了消除警告
    //USB_Init_Config_s conf = {.rx_cbk = FIFO_WRITE};
    USB_Init_Config_s conf = {.rx_cbk = DecodeVision};
    vis_recv_buff = USBInit(conf);
    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id = NULL,
        .reload_count = 5, // 50ms
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

void VisionSend()
{
    VisionUpdateSendCrc();
    USBTransmit((uint8_t *)&send_data, sizeof(GimbalToVision_s));
}

#endif // VISION_USE_VCP
