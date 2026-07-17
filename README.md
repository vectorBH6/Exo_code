# Exo_code

# 基于运动预测的髋关节外骨骼实时助力控制 - AO算法实现

## 📖 项目简介

本项目为论文《基于运动预测的髋关节外骨骼实时助力控制》中 **自适应振荡器（Adaptive Oscillator，AO）** 算法部分的代码实现。

仓库仅包含 **AO 模型的核心逻辑**，主要用于：

- 步态相位预测（Gait Phase Prediction）
- 系统延时补偿（Delay Compensation）

> **注意：** 本项目仅复现论文中的 AO 算法模块，不包含完整的混合振荡器（Hybrid Oscillator，HO）系统及上层决策策略，**中间穿插了魔改小N的算法，以及虚拟步态数据（用于算法简单验证），需要加上注释才能使用实机步态，最后vofa可以看到预测步态，相位变换和第一级预测频率，可以根据预测频率来动态修改魔改小N的算法延迟部分，以匹配不同频率的步态，不过因时间原因该部分暂未实现**。

## 🛠️ 硬件平台

- **开发板：** STM32 DM MC02
- **电机：** 8108 24V 无刷电机（BLDC Motor）

## 📚 参考论文

> 徐铃辉, 杨巍, 杨灿军, 等. 基于运动预测的髋关节外骨骼实时助力控制[J]. 机器人, 2021, 43(4): 473-483. DOI: 10.13973/j.cnki.robot.200557

# Real-Time Assistive Control of Hip Exoskeleton Based on Motion Prediction - AO Algorithm Implementation

## 📖 Project Overview

This repository provides the implementation of the **Adaptive Oscillator (AO)** algorithm from the paper:

The repository contains only **the core implementation of the AO model**, which is mainly used for:

- Gait phase prediction
- System delay compensation

> **Note:** This repository reproduces only the AO algorithm described in the paper. The complete Hybrid Oscillator (HO) framework and high-level assistive control strategies are not included, **The algorithm incorporates a modified Xiao-N algorithm, along with virtual gait data (used for preliminary algorithm validation). // must be added to the code before real-robot gait data can be utilized. In VOFA, you can visualize the predicted gait, phase transformation, and the first-stage predicted frequency. Based on this predicted frequency, the delay component of the modified Xiao-N algorithm could be dynamically adjusted to accommodate gaits at varying frequencies; however, due to time constraints, this feature has not yet been implemented.**.

## 🛠️ Hardware Platform

- **Development Board:** STM32 DM MC02
- **Motor:** 8108 24V BLDC Motor

## 📚 Reference

> 徐铃辉, 杨巍, 杨灿军, 等. 基于运动预测的髋关节外骨骼实时助力控制[J]. 机器人, 2021, 43(4): 473-483. DOI: 10.13973/j.cnki.robot.200557

