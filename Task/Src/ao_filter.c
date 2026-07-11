#include "ao_filter.h"
#include <math.h>

/**
 * @brief  AO 模型初始化
 *
 * 对应论文式(3)(4)：把状态参数 αᵢ、ϕᵢ 置零，把偏置 α₀ 设为初始猜测，
 * ω 用外部传入的 init_omega（一般取稍大的值让它从可能的真实频率附近开始收敛）。
 *
 * νω/νφ/να 这里先给一个保守值（论文 §4.1.1 中提到 νω,νφ,να 由"实时估计步态周期"获得，
 * 最优收敛速度见 Ronsse 2013[27]；本工程在 app_tasks.c 里按上位机口径覆盖成 (2.6, 4.6, 1.3)）。
 */
void AO_Init(AO_HandleTypeDef *ao, float init_omega) {
    ao->omega    = init_omega;  // ω —— 基波角频率 (rad/s), 论文式(3) 第一项

    // 学习率初始化为保守值；外层任务会在 AO_Init 之后按"在线估计步态周期"结果覆盖
    ao->nu_omega = 0.5f;         // νω —— 基频学习率, 论文式(3) 第二项
    ao->nu_phi   = 0.5f;         // νφ —— 相位学习率, 论文式(3) 第一项
    ao->nu_alpha = 0.5f;         // να —— 幅值学习率, 论文式(3) 第三、四项

    // ϕᵢ 与 αᵢ (i=1..M) 全部清零；AO 是状态观测器，从零起步也能收敛
    for (int i = 0; i <= AO_M; i++) {
        ao->phi[i]   = 0.0f;     // ϕᵢ —— 第 i 阶谐波相位
        ao->alpha[i] = 0.0f;     // αᵢ —— 第 i 阶谐波幅值
    }
    ao->alpha[0] = 0.0f;        // α₀ —— 偏置项初始猜测 (≈ 起始姿态角)
}

/**
 * @brief  AO 模型一步状态更新 —— 对应论文式(3)(4) 的前向欧拉离散化
 *
 * 调用频率 = 1/dt。每周期用最新的角度观测 theta_obs 驱动状态参数
 * (ω, ϕᵢ, αᵢ, α₀) 向真实步态信号逼近。
 *
 * @param  ao        AO 模型实例
 * @param  theta_obs 当前滤波后髋关节角度 θ_filter (rad)
 * @param  dt        控制周期 (s)，本工程为 0.005 (=5ms, 200Hz，见 app_tasks.c)
 */
void AO_Update(AO_HandleTypeDef *ao, float theta_obs, float dt) {
    // ─── ① 估计当前 AO 输出 θ̂(t) —— 论文式(4) ───
    // θ̂(t) = α₀ + Σᵢ₌₁..M αᵢ·sin(ϕᵢ)
    float y_est = ao->alpha[0];
    for (int i = 1; i < AO_M; i++) {
        y_est += ao->alpha[i] * sinf(ao->phi[i]);
    }

    // ─── ② 计算跟踪误差 e(t) = θ_obs(t) - θ̂(t) ───
    // 这是把"真实步态"和"AO 内部振荡器"的差当作学习信号，
    // 用来反向调整 ω/ϕᵢ/αᵢ，让 θ̂ 逼近 θ_obs。
    float e = theta_obs - y_est;

    // ─── ③ 论文式(3) 里的"幅值求和"归一化项 ───
    // 论文写法：φ̇ᵢ 和 ẇ 的学习项前面都有  e / Σ αᵢ  这一项（不带绝对值）。
    // Σαᵢ 是 1..M 阶谐波幅值之和；工程上给 Σαᵢ 加 ε 起步防除零，
    // 一旦 αᵢ 建立起来 ε 的影响可忽略。
    // 物理意义：αᵢ 未建立时(Σαᵢ→0)分母小→学习信号放大，迫使振荡器快速"开张"；
    //           收敛后 Σαᵢ 趋稳 → 学习信号被归一化到 O(1) 量级，收敛平顺。
    float amp_sum = 1e-2f;
    for (int i = 1; i < AO_M; i++) {
        amp_sum += ao->alpha[i];
    }
    float e_norm = e / amp_sum;

    // ─── ④ 基频 ω 更新 —— 论文式(3) 第二行 ───
    // ẇ = νω · (e/Σαᵢ) · cos(ϕ₁)  → 离散化：ω += dt·νω·(e/Σαᵢ)·cos(ϕ₁)
    ao->omega += ao->nu_omega * e_norm * cosf(ao->phi[1]) * dt;
    // 工程保护：ω 不能 ≤ 1（否则相位被锁死），也不能太大（避免震荡）
    if (ao->omega < 0.5f)  ao->omega = 0.5f;   // ω_min ≈ 0.5 rad/s
    if (ao->omega > 15.0f) ao->omega = 15.0f;  // ω_max = 15 rad/s

    // ─── ⑤ 各次谐波的相位 ϕᵢ 与幅值 αᵢ 更新 —— 论文式(3) 第一、三行 ───
    // ϕ̇ᵢ = i·ω + νφ · (|e|/Σαᵢ) · cos(ϕᵢ)    ← 论文原式，带幅值归一化
    // α̇ᵢ = να · e · sin(ϕᵢ)                    ← 论文里没除的项，保持原样
    for (int i = 1; i < AO_M; i++) {
        ao->phi[i] += ((float)i * ao->omega + ao->nu_phi * e_norm * cosf(ao->phi[i])) * dt;
        // 相位卷绕到 [0, 2π)：避免长时间运行后 phi 数值过大导致 sinf 精度退化
        while (ao->phi[i] >= TWO_PI_F) ao->phi[i] -= TWO_PI_F;
        while (ao->phi[i] < 0.0f)      ao->phi[i] += TWO_PI_F;
        ao->alpha[i] += ao->nu_alpha * e * sinf(ao->phi[i]) * dt;
    }
    // ─── ⑥ 偏置 α₀ 更新 —— 论文式(3) 第四行 ───
    // α̇₀ = να·e  
    ao->alpha[0] += ao->nu_alpha * e * dt;
}

/**
 * @brief  预测 Δt 之后的关节角度 —— 对应论文式(5)
 *
 *   θ̂(t + Δt) = α₀ + Σᵢ αᵢ·sin(ϕᵢ + i·ω·Δt)
 *
 * 在 Exo_Task 里用 AO_LEAD_TIME = 0.02s 作为 Δt，刚好抵消滤波 + 控制链路总延时。
 * 这样电机在 t+Δt 真正落地时，参考角度就是"当下"的，而不是 Δt 之前的。
 *
 * @param  ao      AO 模型实例
 * @param  delta_t 提前预测时长 Δt (s)
 * @return         预测的关节角度 (rad)
 */
float AO_Predict(AO_HandleTypeDef *ao, float delta_t) {
    float theta_pre = ao->alpha[0];
    for (int i = 1; i < AO_M; i++) {
        theta_pre += ao->alpha[i] * sinf(ao->phi[i] + (float)i * ao->omega * delta_t);
    }
    return theta_pre;
}