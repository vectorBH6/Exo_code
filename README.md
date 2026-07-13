# Exo_code

# 基于运动预测的髋关节外骨骼实时助力控制 - AO算法实现

## 📖 项目简介

本项目为论文《基于运动预测的髋关节外骨骼实时助力控制》中 **自适应振荡器（Adaptive Oscillator，AO）** 算法部分的代码实现。

仓库仅包含 **AO 模型的核心逻辑**，主要用于：

- 步态相位预测（Gait Phase Prediction）
- 系统延时补偿（Delay Compensation）

> **注意：** 本项目仅复现论文中的 AO 算法模块，不包含完整的混合振荡器（Hybrid Oscillator，HO）系统及上层决策策略。

## 🛠️ 硬件平台

- **开发板：** STM32 DM MC02
- **电机：** 8108 24V 无刷电机（BLDC Motor）

# Real-Time Assistive Control of Hip Exoskeleton Based on Motion Prediction - AO Algorithm Implementation

## 📖 Project Overview

This repository provides the implementation of the **Adaptive Oscillator (AO)** algorithm from the paper:

> **Real-Time Assistive Control of Hip Exoskeleton Based on Motion Prediction**

The repository contains only the core implementation of the AO model, which is mainly used for:

- Gait phase prediction
- System delay compensation

> **Note:** This repository reproduces only the AO algorithm described in the paper. The complete Hybrid Oscillator (HO) framework and high-level assistive control strategies are **not included**.

## 🛠️ Hardware Platform

- **Development Board:** STM32 DM MC02
- **Motor:** 8108 24V BLDC Motor



