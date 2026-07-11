#include "motor_can.h"
#include "ws2812.h"
#include <stdint.h>

// 默认电机参数范围（0x01）
static const MotorLimits_t default_limits01 = {
    .pos_min = -12.56f,  // ±π 弧度
    .pos_max = 12.56f,
    .vel_min = -45.0f,   // ±45 rad/s
    .vel_max = 45.0f,
    .tor_min = -18.0f,   // ±18 Nm
    .tor_max = 18.0f,
    .kp_min = 0.0f,      // 位置增益最小值
    .kp_max = 500.0f,    // 位置增益最大值
    .kd_min = 0.0f,      // 速度增益最小值
    .kd_max = 5.0f       // 速度增益最大值
};

// 默认电机参数范围（0x02）
static const MotorLimits_t default_limits02 = {
    .pos_min = -3.14f,  // ±π 弧度
    .pos_max = 3.14f,
    .vel_min = -45.0f,   // ±45 rad/s
    .vel_max = 45.0f,
    .tor_min = -18.0f,   // ±18 Nm
    .tor_max = 18.0f,
    .kp_min = 0.0f,      // 位置增益最小值
    .kp_max = 500.0f,    // 位置增益最大值
    .kd_min = 0.0f,      // 速度增益最小值
    .kd_max = 5.0f       // 速度增益最大值
};

// 全局设备状态数组
MotorState_t devicesState[2] = {
    { .limits = default_limits01 },
    { .limits = default_limits02 }
};

// MIT发送队列错误计数
uint32_t mit_tx_error_count = 0;

// 整数到浮点数转换
float uint_to_float(int x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

// 浮点数到整数转换
uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)(((x - offset) * ((float)((1 << bits) - 1))) / span);
}




// 模块初始化
void MotorCAN_Init(void) {

    FDCAN_FilterTypeDef sFilterConfig;
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = MIT_PROTOCOL_FEEDBACK_ID + 0x01;  
    sFilterConfig.FilterID2 = MIT_PROTOCOL_FEEDBACK_ID + 0x02;  
    
    // 配置FDCAN1过滤器
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }

    // 配置FDCAN2过滤器，同样接收ID为0x781和0x782的消息
    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }

    // 激活FDCAN1接收中断
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    
    // 激活FDCAN2接收中断
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    // 启动FDCAN1
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        Error_Handler();
    }
    // 启动FDCAN2
    if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK) {
        Error_Handler();
    }
}

// 电机状态反馈回调函数
void MotorState_callback(uint8_t nodeID, uint8_t *data){
    uint32_t posInt = (data[1] << 8) | data[2];
    uint32_t velInt = (data[3] << 4) | ((data[4] >> 4) & 0xF);
    uint32_t torInt = ((data[4] & 0xF) << 8) | data[5];

    uint8_t index = nodeID - 1;
    if(index < 2) {
        devicesState[index].pos = uint_to_float(posInt, devicesState[index].limits.pos_min, devicesState[index].limits.pos_max, 16);
        devicesState[index].vel = uint_to_float(velInt, devicesState[index].limits.vel_min, devicesState[index].limits.vel_max, 12);
        devicesState[index].tor = uint_to_float(torInt, devicesState[index].limits.tor_min, devicesState[index].limits.tor_max, 12);
    }
}

// 实现 HAL_FDCAN_RxFifo0Callback 弱符号函数，中断处理
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFrameStatus)
{
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t data[8]; // 添加 data 数组声明

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, data) == HAL_OK)
    {
        // 正确提取 NodeID（反馈ID = MIT_PROTOCOL_FEEDBACK_ID + nodeID）
        uint8_t nodeId = (uint8_t)(RxHeader.Identifier - MIT_PROTOCOL_FEEDBACK_ID); // 作差值

        // 调用状态解析回调，正确传递参数
        MotorState_callback(nodeId, data);
    }
}




// 发送基础命令的通用函数
static int motorcan_send_basic_cmd(uint8_t nodeID, uint8_t cmd_byte)
{
    FDCAN_TxHeaderTypeDef txh = {0};
    uint8_t data[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, cmd_byte};

    // 基础命令帧ID = MOTOR_BASIC_CMD_BASE_ID + nodeID
    txh.Identifier = MOTOR_BASIC_CMD_BASE_ID + nodeID;
    txh.IdType = FDCAN_STANDARD_ID;
    txh.TxFrameType = FDCAN_DATA_FRAME;
    txh.DataLength = FDCAN_DLC_BYTES_8;
    txh.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txh.BitRateSwitch = FDCAN_BRS_OFF;
    txh.FDFormat = FDCAN_CLASSIC_CAN;

    // 选择对应的CAN控制器
    FDCAN_HandleTypeDef* hfdcan = (nodeID == MOTOR_ID_RIGHT) ? &hfdcan1 : &hfdcan2;
    
    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txh, data) != HAL_OK)
        return -1;

    return 0;
}

// 使能电机
void MotorCAN_Enable(uint8_t nodeID)
{
    if (nodeID != MOTOR_ID_LEFT && nodeID != MOTOR_ID_RIGHT) return;
    int n=4;
    while(n--){
        motorcan_send_basic_cmd(nodeID, MOTOR_CMD_ENABLE);
        HAL_Delay(10);
    }
}

// 失能电机
void MotorCAN_Disable(uint8_t nodeID)
{
    if (nodeID != MOTOR_ID_LEFT && nodeID != MOTOR_ID_RIGHT) return;
    motorcan_send_basic_cmd(nodeID, MOTOR_CMD_DISABLE);
}

// 设置当前位置为零点
void MotorCAN_SetPositionZero(uint8_t nodeID)
{
    if (nodeID != MOTOR_ID_LEFT && nodeID != MOTOR_ID_RIGHT) return;
    motorcan_send_basic_cmd(nodeID, MOTOR_CMD_SET_ZERO);
}




// 设置MIT控制模式（位置+速度+力矩）
void MotorCAN_SetMITControl(uint8_t nodeID, float pos, float vel, float Kp, float Kd, float torque) {
    // 参数验证
    if (nodeID != MOTOR_ID_LEFT && nodeID != MOTOR_ID_RIGHT) return;

    // 获取目标电机状态
    MotorState_t* motor = &devicesState[nodeID-1];
    
    // 将物理量转换为协议所需的整数
    uint32_t p_des = float_to_uint(pos, motor->limits.pos_min, motor->limits.pos_max, 16);
    uint32_t v_des = float_to_uint(vel, motor->limits.vel_min, motor->limits.vel_max, 12);
    uint32_t kp_val = float_to_uint(Kp, motor->limits.kp_min, motor->limits.kp_max, 12);
    uint32_t kd_val = float_to_uint(Kd, motor->limits.kd_min, motor->limits.kd_max, 12);
    uint32_t t_ff   = float_to_uint(torque, motor->limits.tor_min, motor->limits.tor_max, 12);

    // 配置CAN消息头
    FDCAN_TxHeaderTypeDef txHeader = {
        .Identifier = MIT_PROTOCOL_BASE_ID + nodeID,
        .IdType = FDCAN_STANDARD_ID,
        .TxFrameType = FDCAN_DATA_FRAME,
        .DataLength = FDCAN_DLC_BYTES_8,
        .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
        .BitRateSwitch = FDCAN_BRS_OFF,
        .FDFormat = FDCAN_CLASSIC_CAN
    };

    // 构建数据帧，严格遵循MIT协议规范
    uint8_t txData[8] = {0};
    txData[0] = (p_des >> 8) & 0xFF;          // p_des[15:8]
    txData[1] = p_des & 0xFF;                 // p_des[7:0]
    txData[2] = (v_des >> 4) & 0xFF;          // v_des[11:4]
    txData[3] = ((v_des & 0x0F) << 4) | ((kp_val >> 8) & 0x0F); // v_des[3:0] | Kp[11:8]
    txData[4] = kp_val & 0xFF;                // Kp[7:0]
    txData[5] = (kd_val >> 4) & 0xFF;         // Kd[11:4]
    txData[6] = ((kd_val & 0x0F) << 4) | ((t_ff >> 8) & 0x0F); // Kd[3:0] | t_ff[11:8]
    txData[7] = t_ff & 0xFF;                  // t_ff[7:0]

    // 根据电机ID选择对应的CAN控制器
    FDCAN_HandleTypeDef* hfdcan = (nodeID == MOTOR_ID_RIGHT) ? &hfdcan1 : &hfdcan2;
    
    // 发送消息
    if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, txData) != HAL_OK){
        mit_tx_error_count++;
    }
}

// 设置电机力矩（MIT模式）
void MotorCAN_SetMotorTorque(uint8_t nodeID, float torque) {
    // 参数验证
    if (nodeID != MOTOR_ID_LEFT && nodeID != MOTOR_ID_RIGHT) return;
    
    // 获取目标电机
    MotorState_t* motor = &devicesState[nodeID-1];
    
    // 将目标力矩转换为12位整数
    uint16_t t_ff = float_to_uint(torque, motor->limits.tor_min, motor->limits.tor_max, 12);
    
    // 配置CAN消息头
    FDCAN_TxHeaderTypeDef txHeader = {
        .Identifier = MIT_PROTOCOL_BASE_ID + nodeID,
        .IdType = FDCAN_STANDARD_ID,
        .TxFrameType = FDCAN_DATA_FRAME,
        .DataLength = FDCAN_DLC_BYTES_8,
        .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
        .BitRateSwitch = FDCAN_BRS_OFF,
        .FDFormat = FDCAN_CLASSIC_CAN
    };
    
    // 构建数据帧
    uint8_t txData[8] = {0};
    txData[6] = (t_ff >> 8) & 0x0F;  // t_ff[11:8]
    txData[7] = t_ff & 0xFF;          // t_ff[7:0]
    
    // 选择对应的CAN控制器
    FDCAN_HandleTypeDef* hfdcan = (nodeID == MOTOR_ID_RIGHT) ? &hfdcan1 : &hfdcan2;
    
    // 发送消息
    if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, txData) != HAL_OK){
        mit_tx_error_count++;
    }
}

