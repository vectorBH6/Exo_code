#include "app_tasks.h"
#include "ao_filter.h"
#include "motor_can.h"
#include "ws2812.h"
#include "cmsis_os.h"
#include "fdcan.h"
#include "usbd_cdc_if.h"
#include "vofa.h"

// ============ 宏定义 ============
#define POS_BUFFER_SIZE 100
#define AO_LEAD_TIME    0.02f  // 论文 Δt (秒) —— 提前 20ms, 抵消控制/通讯链路延迟
#define AO_KAPPA        5.0f   // 论文 §4.2.1: κ = 5
#define LAMBDA_L        0.5f  // 左腿机械摩擦补偿
#define LAMBDA_R        0.5f  // 右腿机械摩擦补偿
#define USE_AO_MODE     1      // 1=AO 预测(论文 §2.3 式7), 0=旧固定延迟(buffer算法)

// ============ 全局变量定义 ============
// 实时左右关节角度（Motor_Task和Vofa_Task共用）
float q_r_processed = 0.0f;  // 右腿处理后角度
float q_l_processed = 0.0f;  // 左腿处理后角度
float tor_r = 0.0f;          // 右腿力矩
float tor_l = 0.0f;          // 左腿力矩

// 电机原始数据
float q_r_raw = 0.0f;  // 右腿原始角度
float q_l_raw = 0.0f;  // 左腿原始角度

// 算法相关参数
float pos1[POS_BUFFER_SIZE] = {0};
float pos2[POS_BUFFER_SIZE] = {0};
float tor_send[2] = {0};
int index_pos = 0;
int delta_index = 50;  // 时延几个周期

// 自适应振荡器：左右腿各自独立 AO（论文 §2.3：τ_i = λ_i·κ·sin(φ_i)）
static AO_HandleTypeDef ao_L;
static AO_HandleTypeDef ao_R;
float phi_pred_l;
float phi_pred_r;

// 控制参数
const float max_torque = 0.0f;     // 最大输出力矩限制（单位：Nm）
const uint32_t control_period = 10;  // 控制周期10ms

void Motor_Task(void const *argument)
{
  (void)argument;

  osDelay(500);

  for (;;)
  {
    // 9. 发送力矩指令给电机
    MotorCAN_SetMotorTorque(MOTOR_ID_RIGHT, tor_send[0]); // 右脚电机 (ID:0x01)
    MotorCAN_SetMotorTorque(MOTOR_ID_LEFT, tor_send[1]);  // 左脚电机 (ID:0x02)
    osDelay(control_period);
  }

}

void Exo_Task(void const *argument)
{
  (void)argument;

  // 先设零
  MotorCAN_SetPositionZero(MOTOR_ID_RIGHT);
  osDelay(500);
  MotorCAN_SetPositionZero(MOTOR_ID_LEFT);
  osDelay(500);

  // 右腿初始偏置角度（硬件问题，右腿电机回传值始终比实际有误）
  const float q_r_init = -0.00001f;

  // 循环发送零数据，更新电机信息
  for (int i = 0; i < 5; i++)
  {
    MotorCAN_SetMotorTorque(MOTOR_ID_LEFT, 0);
    MotorCAN_SetMotorTorque(MOTOR_ID_RIGHT, 0);
    osDelay(5);
  }

  // 1. AO Init 时同时把 ν 注入到结构体里（频率/相位/幅值学习率）
  AO_Init(&ao_L, 0.5f);
  AO_Init(&ao_R, 0.5f);
  ao_L.nu_omega = 0.2f;
  ao_L.nu_phi   = 0.2f;
  ao_L.nu_alpha = 0.4f;
  ao_R.nu_omega = 0.2f;
  ao_R.nu_phi   = 0.2f;
  ao_R.nu_alpha = 0.4f;

  for (;;)
  {
    // 1. 获取电机角度（原始值和处理后值）
    q_r_raw = devicesState[0].pos;        // 右脚电机原始 (ID:0x01)
    q_l_raw = devicesState[1].pos;        // 左脚电机原始 (ID:0x02)

    // ─── 测试数据：单腿一阶谐波 + 左右反相，对齐论文 §3 髋关节屈伸 ───
    //   步频 3 步/秒(双步频) → 单腿频率 1.5 Hz → 单腿周期 T = 2/3 s
    //   单腿角频率 ω = 2π·1.5 = 3π ≈ 9.42 rad/s（注释保留论文原值）
    //   左腿：-5° ~ +40°  →  偏置 α₀ = +17.5° = +0.3054 rad，幅值 A = 22.5° = 0.3927 rad
    //   右腿：-40° ~ +5°  →  偏置 α₀ = -17.5° = -0.3054 rad，幅值 A = 22.5° = 0.3927 rad
    //   左右反相：右腿相位 +π，即 q_r = -q_l（绕 0 镜像），叠加反号偏置后等价于
    //            q_r = TEST_BIAS_R - TEST_AMP * sinf(phase)
    static float test_t = 0.0f;
    const float TEST_OMEGA  = 3.14f;                    // 单腿角频率 (rad/s)
    const float TEST_AMP    = 0.3927f;                   // 22.5°
    const float TEST_BIAS_L =  0.3054f;                  // 左腿偏置 +17.5°
    const float TEST_BIAS_R = -0.3054f;                  // 右腿偏置 -17.5°
    float phase = TEST_OMEGA * test_t;
    phase -= TWO_PI_F * (int)(phase / TWO_PI_F);             // fmodf 等价，但保整数版
    q_l_processed = TEST_BIAS_L + TEST_AMP * sinf(phase);      // 左腿真值
    q_r_processed = TEST_BIAS_R - TEST_AMP * sinf(phase);      // 右腿真值（与左腿反相 π）
    test_t += 0.005f;

    // 处理后的角度（用于VOFA显示）
    q_r_processed = q_r_raw - q_r_init;  // 右腿处理后角度
    q_l_processed = q_l_raw;              // 左腿处理后角度

    // 2. 存储位置数据用于延迟控制（旧算法 buffer）
    pos1[index_pos] = q_r_processed;
    pos2[index_pos] = q_l_processed;
    // 2. 自适应振荡器更新：左右腿各自独立迭代
    //   左腿：q_l_processed 直接喂入（伸=0, 屈=正，符合论文坐标系）
    //   右腿：取反让 AO 看到同样的"伸=0, 屈=正"，避免双 AO 反相抵消
    AO_Update(&ao_L, q_l_processed, 0.005f);             // dt=5ms
    AO_Update(&ao_R, q_r_processed, 0.005f);             // 右腿 AO，独立迭代

    // 3. 算延迟索引（旧固定延迟算法使用）
    int past_index = 0;
    if(index_pos < delta_index){
        past_index = (POS_BUFFER_SIZE - 1) + index_pos - delta_index;
    } else {
        past_index = index_pos - delta_index;
    }
    (void)past_index; // 仅 USE_AO_MODE==0 时使用

#if USE_AO_MODE
    // 3a. AO 模式：各自预测 Δt 之后的相位（论文式 6）：φ_pred = φ1 + ω·Δt
    phi_pred_r = ao_R.phi[1] + ao_R.omega * AO_LEAD_TIME;
    phi_pred_l = ao_L.phi[1] + ao_L.omega * AO_LEAD_TIME;
    // 4a. 助力矩按论文式 (7)(8)：τ_i = λ_i · κ · sin(φ_i)
    //     右腿：右 AO 用的是 -q_r，所以 τ_r = λ_R·κ·sin(φ_r) 已经"反相"了回正
    //     左腿：左 AO 直接用 q_l，所以 τ_l = λ_L·κ·sin(φ_l) 同相
    tor_send[0] = LAMBDA_R * AO_KAPPA * sinf(phi_pred_r);   // 右腿电机
    tor_send[1] = LAMBDA_L * AO_KAPPA * sinf(phi_pred_l);   // 左腿电机
#else
    // 3b/4b. 旧算法：用过去 buffer 里的位置差算力矩
    tor_send[0] = -5.0f * (pos1[past_index] + pos2[past_index]);
    tor_send[1] = tor_send[0];
#endif

    // 5. 小力矩时设为零（死区）
    if(tor_send[0] > -2.7f && tor_send[0] < 2.7f)
      tor_send[0] = 0.0f;
    if(tor_send[1] > -2.7f && tor_send[1] < 2.7f)
      tor_send[1] = 0.0f;

    // 6. 向下摆腿不需要出大力（重力补偿简化）
    if(tor_send[0] > 0.0f)
      tor_send[0] = 0.2f * tor_send[0];
    if(tor_send[1] < 0.0f)
      tor_send[1] = 0.2f * tor_send[1];

    // 7. 力矩限幅处理
    if (tor_send[0] > max_torque) tor_send[0] = max_torque;
    if (tor_send[0] < -max_torque) tor_send[0] = -max_torque;
    if (tor_send[1] > max_torque) tor_send[1] = max_torque;
    if (tor_send[1] < -max_torque) tor_send[1] = -max_torque;

    // 8. 更新VOFA显示的力矩数据
    tor_r = tor_send[0];
    tor_l = tor_send[1];

    // 10. 更新位置缓冲区索引
    index_pos++;
    index_pos = index_pos % (POS_BUFFER_SIZE - 1);

    // 11. 延时，计算循环周期
    osDelay(5);  //200hz计算
  }
}

// ============ VOFA 数据发送任务 ============
void Vofa_Task(void const *argument)
{
  (void)argument;

  osDelay(1000);

  for (;;)
  {
    // Call the function to store the data in the buffer
    // AO 预测角度（送上位机看拟合效果；论文式 θ_pre = α₀ + Σ αᵢ·sin(φᵢ + i·ω·Δt)）
    vofa_send_data(0, q_r_processed);
    vofa_send_data(1, q_l_processed);
    vofa_send_data(2, phi_pred_r);
    vofa_send_data(3, phi_pred_l);
    vofa_send_data(4,  AO_Predict(&ao_R, AO_LEAD_TIME));  // 右腿 AO 预测角度(rad)
    vofa_send_data(5, AO_Predict(&ao_L, AO_LEAD_TIME));  // 左腿 AO 预测角度(rad)
    vofa_send_data(6, ao_R.omega);                // 右腿频率 (rad/s)
    vofa_send_data(7, ao_L.omega);                // 左腿频率 (rad/s)

    // Call the function to send the frame tail
    vofa_sendframetail();
  }
}

// ============ Led 循环任务 ================
void Led_Task(void const *argument)
{
  (void)argument;

  osDelay(1000);

  for (;;)
  {
    WS2812_Ctrl(0, 255, 0);
    osDelay(200);
    WS2812_Ctrl(0, 0, 255);
    osDelay(200);
  }

}



