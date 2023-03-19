#ifndef __GO_M8010_6_H
#define __GO_M8010_6_H

#include "main.h"
#include "crc_ccitt.h"
#include "usart.h"
#include "string.h"
#include <math.h>

/**
 * @brief 电机模式控制信息
 * 
 */
typedef struct __packed     // __attribute__((packed)) <- c++
{
    uint8_t id     :4;      // 电机ID: 0,1...,14 15表示向所有电机广播数据(此时无返回)
    uint8_t status :3;      // 工作模式: 0.锁定 1.FOC闭环 2.编码器校准 3.保留
    uint8_t none   :1;      // 保留位
} RIS_Mode_t /*__attribute__((packed))*/;   // 控制模式 1Byte



/**
 * @brief 电机状态控制信息
 * 
 */
typedef struct
{
    int16_t tor_des;        // 期望关节输出扭矩 unit: N.m     (q8)
    int16_t spd_des;        // 期望关节输出速度 unit: rad/s   (q7)
    int32_t pos_des;        // 期望关节输出位置 unit: rad     (q15)
    uint16_t  k_pos;        // 期望关节刚度系数 unit: 0.0-1.0 (q15)
    uint16_t  k_spd;        // 期望关节阻尼系数 unit: 0.0-1.0 (q15)
    
} RIS_Comd_t;   // 控制参数 12Byte




/**
 * @brief 控制数据包格式
 * 
 */
typedef struct
{
    uint8_t head[2];    // 包头         2Byte
    RIS_Mode_t mode;    // 电机控制模式  1Byte
    RIS_Comd_t comd;    // 电机期望数据 12Byte
    uint16_t   CRC16;   // CRC          2Byte

} ControlData_t;    // 主机控制命令     17Byte




typedef struct {              
        unsigned short id;              // 电机ID，0xBB代表全部电机
        unsigned short mode;            // 0:空闲, 5:开环转动, 10:闭环FOC控制
        uint16_t correct;               // 接收数据是否完整 （1 完整，0不完整）
        int MError;                     // 错误码 0.正常 1.过热 2.过流 3.过压 4.编码器故障
        int Temp;                       // 温度
        float tar_pos;                  // target position 
        float tar_w;                    // target speed
        float T;                        // 当前实际电机输出力矩
        float W;                        // 当前实际电机速度（高速）
        float Pos;                      // 当前电机位置
        int footForce;                  // They dont even know what 7 is this so we dont update this
        uint8_t buffer[17];
        uint8_t Rec_buffer[16];
        ControlData_t  motor_send_data;  
}GO_Motorfield;

void GO_M8010_init ();
// GO_Motorfield GO_M8010_init (uint8_t id, uint8_t mode);

void GO_M8010_send_data(UART_HandleTypeDef *huart,int id, int rev,float T,float W,float Pos,float K_P,float K_W);

void uartTxCB(UART_HandleTypeDef *huart);

void GO_M8010_recv_data(uint8_t* Temp_buffer);

void basic_ForceControl (UART_HandleTypeDef *huart, int id, float bias, float length, float mass, 
                        float tar_pos, float tar_w, float K_P,float K_W);

extern GO_Motorfield GO_motor_info[4];





#endif /*__GO_M8010_6_H */