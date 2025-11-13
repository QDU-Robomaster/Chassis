# Chassis 模块

## 模块描述
`Chassis` 模块提供了一个通用的、可扩展的底盘控制框架。它采用模板元编程技术，定义了一个顶层包装类 `Chassis<ChassisType, MotorType>`。该类将具体的底盘运动学实现（如 `Mecanum`, `Omni`, `Helm`）和电机驱动实现（如 [RMMotorContainer](file:///home/leo/Documents/bsp-dev-c/Modules/Motor/RMMotorContainer.hpp#L56-L519)）作为模板参数。

这种设计实现了上层控制逻辑（如速度环、姿态环）与底层运动学解算和硬件驱动的完全解耦。`Chassis` 类本身负责高层状态管理和控制循环，并将具体的运动学计算委托给其内部的 `ChassisType` 实例，从而实现了零成本的编译时抽象。

支持的底盘类型包括：
- `Mecanum`: 麦克纳姆轮底盘
- `Omni`: 全向轮底盘
- `Helm`: 舵轮底盘

## 依赖
*   **硬件**:
    *   遥控器 (DR16)
    *   电机与电调 (Motor)
    *   CAN 总线
    *   IMU (BMI088)
*   **软件模块**:
    *   `CMD`: 用于接收和解析控制指令。
    *   `Motor`: 用于驱动底盘电机。
    *   `app_framework`: LibXR 核心框架。
    *   `pid`: 用于内置的速度和姿态闭环控制。

## 构造参数
`Chassis<ChassisType, MotorType>` 类的构造函数参数如下：

*   `hw`: `LibXR::HardwareContainer&`，硬件容器的引用。
*   `app`: `LibXR::ApplicationManager&`，应用管理器的引用。
*   `cmd`: `CMD&`，命令模块的引用，用于获取控制输入。
*   `motor_wheel_0` 至 `motor_wheel_3`: 各个驱动轮的电机引用。
*   `motor_steer_0` 至 `motor_steer_3`: 各个舵向电机引用（适用于舵轮底盘）。
*   `task_stack_depth`:uint32_t，为底盘控制任务分配的堆栈大小。
*   `wheel_radius`: 轮子半径。
*   `wheel_to_center`: 轮子到中心的距离。
*   `gravity_height`: 重心高度。
*   `wheel_resistance`: 轮子阻力。
*   `error_compensation`: 误差补偿系数。
*   `pid_...`: 一系列 `LibXR::PID<float>::Param` 结构体，用于配置各个控制环的 PID 参数，包括：
    *   `pid_velocity_x_`, `pid_velocity_y_`: 底盘 X/Y 轴速度环。
    *   `pid_omega_`: 底盘旋转角速度环。
    *   `pid_wheel_angle_...`: 轮速环（特定底盘模式使用）。
    *   `pid_steer_angle_...`: 舵向角环（舵轮底盘使用）。

## 主要功能
*   **编译时多态**: 通过C++模板实现，允许在不修改代码的情况下切换不同的底盘运动学方案（麦克纳姆轮、全向轮、舵轮等）。
*   **统一接口**: 为所有不同类型的底盘提供了一致的控制API。
*   **闭环控制**: 内置基于 PID 的速度环和扭矩环，可实现精确的运动控制。
*   **多种底盘支持**: 支持麦克纳姆轮、全向轮和舵轮三种常见的机器人底盘类型。
*   **易于扩展**: 支持新型底盘只需创建一个符合接口规范的新运动学类，无需修改 `Chassis` 包装类。

## 核心类与结构体
*   `Chassis<ChassisType, MotorType>`: 顶层模板包装类。它包含一个 `ChassisType` 的实例，并将运动学解算等操作转发给它。
*   `ChassisType`: 模板参数，代表具体的底盘运动学实现，例如 `Mecanum`、`Omni`、`Helm`。
*   `MotorType`: 模板参数，代表具体的电机驱动实现，例如 RMMotorContainer

支持的底盘类型：
*   `Mecanum<MotorType>`: 麦克纳姆轮底盘实现，使用四个麦克纳姆轮。
*   `Omni<MotorType>`: 全向轮底盘实现。
*   `Helm<MotorType>`: 舵轮底盘实现，支持独立的驱动轮和舵向电机控制。

## 设计原则与工作方式
`Chassis` 模块遵循经典的**策略模式 (Strategy Pattern)** 和 **包装器模式 (Wrapper Pattern)**，并在编译时完成组合。

1.  **实例化**: 当你声明一个底盘实例 `Chassis<Mecanum<RMMotorContainer>, RMMotorContainer> chassis(...)` 时，`Chassis` 类内部会创建一个 `Mecanum` 类型的成员变量 `chassis_`。
2.  **构造转发**: `Chassis` 的构造函数会将其接收到的参数部分转发给 `chassis_` 的构造函数。
3.  **方法委托**: 在 `Chassis` 的主控制循环中，各种运动学计算和控制算法由具体的底盘实现类处理。

每种底盘类型都有其专门的运动学解算和控制算法：
- `Mecanum`: 使用标准麦克纳姆轮运动学模型
- `Omni`: 为全向轮优化的运动学模型
- `Helm`: 支持舵轮控制，包括舵向角度优化算法

## 如何使用
```bash
# 添加 Chassis 模块实例
xrobot_add_mod Chassis --instance-id chassis
# 生成主应用程序入口
xrobot_gen_main
