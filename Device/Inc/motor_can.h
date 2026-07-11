#ifndef MOTOR_CAN_H
#define MOTOR_CAN_H

#include "fdcan.h"
#include <stdint.h>

/*
 * MIT电机协议核心配置
 * 所有参数通过统一结构体管理，避免宏定义碎片化
 */
typedef struct {
    float pos_min;    // 位置最小值(弧度)
    float pos_max;    // 位置最大值(弧度)
    float vel_min;    // 速度最小值(rad/s)
    float vel_max;    // 速度最大值(rad/s)
    float tor_min;    // 扭矩最小值(Nm)
    float tor_max;    // 扭矩最大值(Nm)
    float kp_min;     // 位置增益最小值
    float kp_max;     // 位置增益最大값
    float kd_min;     // 速度增益最小값
    float kd_max;     // 速度增益最大값
} MotorLimits_t;

/*
 * 电机状态结构体
 * 包含实时状态和范围限制
 */
typedef struct {
    float pos;      // 位置（弧度）
    float vel;      // 速度（弧度/秒）
    float tor;      // 扭矩（Nm）
    MotorLimits_t limits; // 范围限制
} MotorState_t;

/*
 * 协议常量定义
 */
#define MIT_PROTOCOL_BASE_ID 0x200
#define MIT_PROTOCOL_FEEDBACK_ID 0x780

/*
 * 电机ID定义
 */
#define MOTOR_ID_LEFT  0x02
#define MOTOR_ID_RIGHT 0x01

/*
 * 电机基本命令帧ID
 * 用于使能/失能/设置零点等基础操作
 */
#define MOTOR_BASIC_CMD_BASE_ID 0x00

/*
 * 电机基本命令字节
 */
#define MOTOR_CMD_ENABLE   0xFC
#define MOTOR_CMD_DISABLE  0xFD
#define MOTOR_CMD_SET_ZERO 0xFE

/*
 * 数值转换函数（简化版，直接传入范围和位宽）
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits);
uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits);

/*
 * 全局设备状态
 */
extern MotorState_t devicesState[2];

/*
 * 电机控制接口
 */
void MotorCAN_Init(void);
void MotorCAN_Enable(uint8_t nodeID);
void MotorCAN_Disable(uint8_t nodeID);
void MotorCAN_SetPositionZero(uint8_t nodeID);
void MotorState_callback(uint8_t nodeID, uint8_t *data);
void MotorCAN_SetMotorTorque(uint8_t nodeID, float torque);

/*
 * 设置MIT控制参数（位置+速度+力矩控制）
 * @param nodeID 电机节点ID (0x01 或 0x02)
 * @param pos 位置指令 (rad)
 * @param vel 速度指令 (rad/s)
 * @param Kp 位置比例系数
 * @param Kd 速度阻尼系数
 * @param torque 前馈力矩 (Nm)
 */
void MotorCAN_SetMITControl(uint8_t nodeID, float pos, float vel, float Kp, float Kd, float torque);

#endif /* MOTOR_CAN_H */
