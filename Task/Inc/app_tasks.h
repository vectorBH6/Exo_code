#ifndef APP_TASKS_H
#define APP_TASKS_H

#include <stdint.h>

// 实时左右关节角度（Motor_Task和Vofa_Task共用）
extern float q_r_processed;  // 右腿处理后角度
extern float q_l_processed;  // 左腿处理后角度
extern float tor_r;          // 右腿力矩
extern float tor_l;          // 左腿力矩

// 电机原始数据
extern float q_r_raw;  // 右腿原始角度
extern float q_l_raw;  // 左腿原始角度

void CAN_Task(void const *argument);
void Motor_Task(void const *argument);
void Vofa_Task(void const *argument);

#endif /* APP_TASKS_H */
