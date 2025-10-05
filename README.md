# Chassis 模块

## 模块描述
`Chassis` 模块提供了一个通用的、可扩展的底盘控制框架。它与 `Motor` 模块的设计哲学类似，采用模板元编程技术，定义了一个顶层包装类 `Chassis<ChassisType, MotorType>`。该类将具体的底盘运动学实现（如 `Mecanum`, `Omni`, `Helm`）和电机驱动实现（如 `RMMotorContainer`）作为模板参数。

这种设计实现了上层控制逻辑（如速度环、姿态环）与底层运动学解算和硬件驱动的完全解耦。`Chassis` 类本身负责高层状态管理和控制循环，并将具体的运动学计算委托给其内部的 `ChassisType` 实例，从而实现了零成本的编译时抽象。

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
*   `motor_can1`, `motor_can2`: `Motor<MotorType>&`，电机模块的引用。
*   `task_stack_depth`: `uint32_t`，为底盘控制任务分配的堆栈大小。
*   `pid_...`: 一系列 `LibXR::PID<float>::Param` 结构体，用于配置各个控制环的 PID 参数，包括：
    *   `pid_velocity_x_`, `pid_velocity_y_`: 底盘 X/Y 轴速度环。
    *   `pid_omega_`: 底盘旋转角速度环（小陀螺模式）。
    *   `pid_wheel_angle_...`: 轮速环（特定底盘模式使用）。
    *   `pid_steer_angle_...`: 舵向角环（全向轮/舵轮底盘使用）。

## 主要功能
*   **编译时多态**: 通过C++模板实现，允许在不修改代码的情况下切换不同的底盘运动学方案（麦克纳姆轮、全向轮、舵轮等）。
*   **统一接口**: 为所有不同类型的底盘提供了一致的控制API。
*   **闭环控制**: 内置基于 PID 的速度环和扭矩环，可实现精确的运动控制。
*   **易于扩展**: 支持新型底盘只需创建一个符合接口规范的新运动学类，无需修改 `Chassis` 包装类。

## 核心类与结构体
*   `Chassis<ChassisType, MotorType>`: 顶层模板包装类。它包含一个 `ChassisType` 的实例，并将运动学解算等操作转发给它。
*   `ChassisType`: 模板参数，代表具体的底盘运动学实现，例如 `Mecanum`、`Omni`、`Helm`。
*   `MotorType`: 模板参数，代表具体的电机驱动实现，例如 `RMMotorContainer`。

## 设计原则与工作方式
`Chassis` 模块遵循经典的**策略模式 (Strategy Pattern)** 和 **包装器模式 (Wrapper Pattern)**，并在编译时完成组合。

1.  **实例化**: 当你声明一个底盘实例 `Chassis<Mecanum<RMMotorContainer>, RMMotorContainer> chassis(...)` 时，`Chassis` 类内部会创建一个 `Mecanum` 类型的成员变量 `chassis_`。
2.  **构造转发**: `Chassis` 的构造函数会将其接收到的参数部分转发给 `chassis_` 的构造函数。
3.  **方法委托**: 在 `Chassis` 的主控制循环中，当需要根据目标速度计算每个车轮的期望输出时，它会调用 `chassis_.Solve(...)` 方法。`Mecanum`、`Omni` 或 `Helm` 类会各自实现自己的 `Solve` 方法，执行不同的运动学解算。

这个设计的核心优势是，`Chassis` 类中的主控制逻辑（如从 `CMD` 获取输入、执行 PID 计算）是通用的，而将与具体底盘构型相关的复杂计算（运动学正/逆解）委托给了 `ChassisType` 策略类。

## 如何使用
```bash
# 添加 Chassis 模块实例
xrobot_add_mod.exe Chassis --instance-id mecanum_chassis
# 生成主应用程序入口
xrobot_gen_main.exe
```
然后在 `User/xrobot.yaml` 中配置 `Chassis` 实例的模板参数和构造函数参数，以选择具体的底盘类型和电机。

## 主要方法
`Chassis<ChassisType, MotorType>` 类提供了以下公共方法用于控制和交互：

*   `void SetVelocity(float vx, float vy, float omega)`: 设置底盘的目标速度。`vx` 和 `vy` 是底盘坐标系下的线速度 (m/s)，`omega` 是角速度 (rad/s)。
*   `void SetFollow(bool follow)`: 设置底盘是否进入小陀螺（底盘跟随云台）模式。
*   `void SetPowerLimit(float limit)`: 设置底盘的功率限制。
*   `float GetPowerLimit()`: 获取当前底盘的功率限制。
*   `float GetPower()`: 获取底盘瞬时功率。
*   `void SetMaxSpeed(float max_speed)`: 设置底盘最大速度。
*   `float GetMaxSpeed()`: 获取底盘最大速度。
