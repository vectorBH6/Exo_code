/**
 * @brief  自适应振荡器 (Adaptive Oscillator, AO) —— 对应论文 §2.1.2 式(3)-(6)
 *
 * 论文出处：
 *   徐铃辉 等. 基于运动预测的髋关节外骨骼实时助力控制. 机器人, 2021, 43(4): 473-483.
 *   式(3)：AO 模型状态更新微分方程
 *        φ̇ᵢ = ω·i + νφ · (|e|/Σαⱼ) · cos(ϕᵢ)
 *        ẇ   = νω  · (|e|/Σαⱼ) · cos(ϕ₁)
 *        α̇ᵢ = νη · e · sin(ϕᵢ)              ← 这里没有除法项
 *        α̇₀ = νη · e                       ← 这里也没有
 *        Σαⱼ 是各次谐波幅值绝对值之和，工程上加 1e-3 防起步除零；
 *        分母作用是"幅值归一化"：αᵢ 未建立时放大学习信号迫使振荡器快速开张，
 *        收敛后把学习信号拉回到 O(1) 量级使收敛变平顺。
 *   式(4)：θ̂(t) = α₀ + Σᵢ αᵢ·sin(i·ω·t + ϕᵢ)
 *   式(5)：θ̂(t+Δt)  提前 Δt 的角度预测
 *   式(6)：φ₁(t+Δt) = ω·(t+Δt) + ϕ₁  提前 Δt 的相位预测
 *
 * 设计说明：
 *   - AO_M = 2：保留基波 + 2 次谐波，对髋关节屈伸角度这种近似正弦信号已经够用；
 *     论文中 M 也是可调，这里与代码现状保持一致。
 *   - alpha[0] 是直流偏置，alpha[1..M] 是各次谐波幅值，phi[1..M] 是各次谐波相位。
 *   - 论文对 ω 没有额外平滑，迭代公式直接使用当前 ω。
 */

#ifndef __AO_FILTER_H
#define __AO_FILTER_H

#include <stdint.h>

#define AO_M              3                  // 谐波阶数 M (论文公式中的 i=1..M)
#define TWO_PI_F          6.28318530718f      // 2π，用于相位卷绕到 [0, 2π)

typedef struct {
    float omega;
    float phi[AO_M + 1];
    float alpha[AO_M + 1];
    float nu_omega;
    float nu_phi;
    float nu_alpha;
} AO_HandleTypeDef;

void AO_Init(AO_HandleTypeDef *ao, float init_omega);
void AO_Update(AO_HandleTypeDef *ao, float theta_obs, float dt);
float AO_Predict(AO_HandleTypeDef *ao, float delta_t);

#endif