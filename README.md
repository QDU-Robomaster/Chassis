# Chassis

底盘总控模块。`Chassis<ChassisType>` 是一个模板外壳：模板参数选择具体底盘实现
（`Mecanum` 麦轮、`Omni` 全向轮、`Helm` 舵轮，分别在本仓库的 `Mecanum.hpp`、`Omni.hpp`、
`Helm.hpp` 中），外壳统一构造参数并通过 `GetEvent()` 提供模式切换事件。

## 工作方式

- 每种底盘各自创建一个控制线程（`MecanumChassisThread` / `OmniChassisThread` /
  `HelmChassisThread`），栈深为 `task_stack_depth`，优先级为 `param.thread_priority`，周期 2 ms。
- 模式切换：`GetEvent()` 返回的 `LibXR::Event` 上注册了 `ChassisType::ChassisMode` 的各个值，
  激活对应事件 ID 即切换模式并复位速度相关 PID（通常由 `EventBinder` 把遥控器事件绑定过来）。
  `Mecanum` / `Omni` 在 CMD 的 `CMD_EVENT_LOST_CTRL` 与 `CMD_EVENT_START_CTRL` 时切到 `RELAX`；
  `Helm` 只在 `CMD_EVENT_LOST_CTRL` 时对全部电机调用一次 `Relax()`，不改变模式。
- 控制链（`Mecanum` / `Omni`）：由 `chassis_cmd` 的 x / y / z 得到目标速度 → 逆运动学得到各轮
  目标角速度 → 轮速 PID 加上速度环 / 角速度环 PID 得到的前馈力 → 功率控制限幅 → 以
  `MODE_TORQUE` 下发到轮电机（限幅 ±6 N·m）。`Helm` 由目标速度解算各轮舵角与轮速，舵向为
  角度环 + 速度环，轮为速度环，同样以 `MODE_TORQUE` 下发（轮 ±4.4、舵 ±2.5 N·m）。
  `RELAX` 模式下所有电机 `Relax()`。
- 功率上限：裁判系统 `chassis_ref` 在 1 s 内有数据且 `chassis_power_limit` > 1 W 时使用该值，
  否则使用默认值（`Mecanum` 70 W，`Omni` / `Helm` 100 W）。`chassis_cmd` 的 `self_define`
  为 `CMD::ChasStat::BOOST` 且超级电容在线时，按电容能量 > 0.25 / 0.5 / 0.8 分别额外增加
  100 / 200 / 300 W。限幅由 `PowerControl` 完成。
- 小陀螺（`ROTOR`）转速缩放（`Mecanum` / `Omni`）：平移输入越大越按 `rotor_speed_scale` 降低
  转速；另按裁判系统缓冲能量（在 `rotor_buffer_low_j`–`rotor_buffer_high_j` 之间线性映射）和
  功率受限程度动态缩放，下限 `rotor_omega_min_scale`，经系数为 `rotor_scale_lpf_alpha` 的一阶低通。

订阅的 Topic：

| Topic | 类型 | 用途 |
| --- | --- | --- |
| `chassis_cmd` | `CMD::ChassisCMD` | 底盘控制命令（CMD 发布） |
| `chassis_ref` | `Referee::ChassisPack` | 裁判系统功率上限与缓冲能量 |
| `yawmotor_angle` | `float` | 云台 yaw 电机相对零点的角度（Gimbal 发布），用于 FOLLOW 与坐标变换 |
| `gimbal_euler` | `LibXR::EulerAngle<float>` | 仅 `Omni`：云台姿态，用于姿态前馈 |
| `pitchmotor_angle` | `float` | 仅 `Omni`：云台 pitch 电机角度，用于姿态前馈 |

`Helm` 另外每周期发布 `delta_yaw`（`float`，`yawmotor_angle` 归一化后取反）。

## 底盘类型

| | `Mecanum` | `Omni` | `Helm` |
| --- | --- | --- | --- |
| 模式 (`ChassisMode`) | `RELAX`、`INDEPENDENT`、`ROTOR`、`FOLLOW`、`TRACK_START` | `RELAX`、`INDEPENDENT`、`ROTOR`、`FOLLOW` | `RELAX`、`INDEPENDENT`、`ROTOR`、`FOLLOW` |
| `motor_steer_0` | 可选的履带电机，不用时填 `nullptr` | 不使用，填 `nullptr` | 舵向电机 0，必须非空 |
| `motor_steer_1..3` | 不使用，填 `nullptr` | 不使用，填 `nullptr` | 舵向电机 1..3，必须非空 |
| 使用的 PID | follow、velocity_x/y、omega、wheel_speed_0..3；`pid_steer_angle_0` 用作履带速度 PID | follow、velocity_x/y、omega、wheel_speed_0..3 | follow、wheel_speed_0..3、steer_angle_0..3、steer_speed_0..3 |
| 使用的 `chassis_param` | `wheel_radius`、`wheel_to_center`、`reduction_ratio`、`rotor_*` | 同左，另加 `length`、`width`、`gravity_height`、`gravity`（姿态前馈） | `reduction_ratio` |
| 裁判系统 UI | 定时任务每 52 ms 绘制模式字符、辅助线、电容能量条 | 定时任务每 100 ms 绘制模式、AI 状态、电容能量 | 无（`referee` 参数不使用） |

说明：

- `Mecanum` 轮序（箭头为轮子正方向）：wheel0 左前、wheel1 左后、wheel2 右后、wheel3 右前。
  `TRACK_START` 模式由履带（`motor_steer_0`）负责前后，麦轮保留横移并提供辅助，履带目标
  速度带斜坡，功率分配时优先保证履带。
- `Helm` 的舵向零点写死在 `Helm.hpp` 的 `zero_` 数组中，更换机械结构时需要修改源码。
- 未使用的参数仍需填写（PID 可保持默认），指针依赖不用时填 `nullptr`。

## 依赖

- `QDU-Robomaster/Motor`：轮电机与舵向电机的抽象接口。
- `QDU-Robomaster/RMMotor`：包含其头文件，使用 RoboMaster 电机的量程常量。
- `QDU-Robomaster/CMD`：控制命令类型与 CMD 事件。
- `QDU-Robomaster/PowerControl`：功率限制与电容能量。
- `QDU-Robomaster/Referee`：裁判系统数据类型与 UI 绘制。
- `QDU-Robomaster/SuperPower`：`Mecanum.hpp` 包含其头文件。
- `xrobot-org/BMI088`、`xrobot-org/MadgwickAHRS`：列在 manifest 中，代码不直接包含；
  `Omni` 订阅的 `gimbal_euler` 通常由姿态解算实例以该名字发布。

无外部软件包，仅使用 LibXR。

## 构造接口

```cpp
template <typename ChassisType>
class Chassis;

Chassis(Motor& motor_wheel_0,
        Motor& motor_wheel_1,
        Motor& motor_wheel_2,
        Motor& motor_wheel_3,
        Motor* motor_steer_0,
        Motor* motor_steer_1,
        Motor* motor_steer_2,
        Motor* motor_steer_3,
        CMD& cmd,
        PowerControl& power_control,
        Referee& referee,
        uint32_t task_stack_depth = 1536,
        const Param& param = {...});
```

`param` 的默认值：`chassis_param` 与全部 PID 参数为 `{}`（值初始化），
`thread_priority = LibXR::Thread::Priority::HIGH`。

模板参数：

- `ChassisType`：`Mecanum`、`Omni` 或 `Helm`，见上表。

依赖：

- `motor_wheel_0..3`：`Motor`，四个驱动轮电机（通常是 `RMMotor` 实例）。
- `motor_steer_0..3`：`Motor*`，舵向电机或履带电机，按底盘类型填写或填 `nullptr`。
- `cmd`：`CMD` 实例，提供控制事件与控制模式。
- `power_control`：`PowerControl` 实例。
- `referee`：`Referee` 实例。

配置：

- `task_stack_depth`：控制线程栈深，默认 1536。
- `param.chassis_param`：
  - `wheel_radius`：轮半径 (m)；
  - `wheel_to_center`：轮心到底盘中心距离 (m)；
  - `gravity_height`、`gravity`、`length`、`width`：仅 `Omni` 姿态前馈使用；
  - `reduction_ratio`：轮电机减速比；
  - `wheel_resistance`、`error_compensation`：当前代码未使用；
  - `rotor_speed_scale`（默认 1.0）、`rotor_omega_min_scale`（默认 0.55）、
    `rotor_buffer_low_j`（默认 35 J）、`rotor_buffer_high_j`（默认 70 J）、
    `rotor_scale_lpf_alpha`（默认 0.2）：小陀螺转速缩放，见上文。
  - 除 `rotor_*` 外默认均为 0，需按实车填写。
- `param.pid_follow`、`pid_velocity_x`、`pid_velocity_y`、`pid_omega`、`pid_wheel_speed_0..3`、
  `pid_steer_angle_0..3`、`pid_steer_speed_0..3`：`LibXR::PID<float>::Param`，
  字段依次为 `k`、`p`、`i`、`d`、`i_limit`、`out_limit`、`cycle`；默认全为 LibXR 的默认值。
- `param.thread_priority`：控制线程优先级，默认 `HIGH`。

## 使用

```sh
xrobot module add QDU-Robomaster/Chassis
xrobot setup
xrobot instance add QDU-Robomaster/Chassis
```

`xrobot instance add` 在 `User/xrobot.yaml` 中写入一个实例，依赖项留空，默认值按源码写出；
填好依赖，并在 `template_args` 中选择底盘类型。下面是不带履带的麦轮底盘
（`chassis_param` 的 0 值与空 PID 需要按实车标定后替换）：

```yaml
modules:
  - module: QDU-Robomaster/Chassis
    id: chassis_0
    template_args:
      - Mecanum
    args:
      - motor_wheel_0: motor_wheel_0
      - motor_wheel_1: motor_wheel_1
      - motor_wheel_2: motor_wheel_2
      - motor_wheel_3: motor_wheel_3
      - motor_steer_0: nullptr
      - motor_steer_1: nullptr
      - motor_steer_2: nullptr
      - motor_steer_3: nullptr
      - cmd: cmd
      - power_control: power_control
      - referee: referee
      - task_stack_depth: '1536'
      - param:
          chassis_param:
            wheel_radius: 0.0f
            wheel_to_center: 0.0f
            gravity_height: 0.0f
            reduction_ratio: 0.0f
            wheel_resistance: 0.0f
            error_compensation: 0.0f
            gravity: 0.0f
            length: 0.0f
            width: 0.0f
            rotor_speed_scale: 1.0f
            rotor_omega_min_scale: 0.55f
            rotor_buffer_low_j: 35.0f
            rotor_buffer_high_j: 70.0f
            rotor_scale_lpf_alpha: 0.2f
          pid_follow: []
          pid_velocity_x: []
          pid_velocity_y: []
          pid_omega: []
          pid_wheel_speed_0: []
          pid_wheel_speed_1: []
          pid_wheel_speed_2: []
          pid_wheel_speed_3: []
          pid_steer_angle_0: []
          pid_steer_angle_1: []
          pid_steer_angle_2: []
          pid_steer_angle_3: []
          pid_steer_speed_0: []
          pid_steer_speed_1: []
          pid_steer_speed_2: []
          pid_steer_speed_3: []
          thread_priority: LibXR::Thread::Priority::HIGH
```

所有依赖都是其他模块实例的 id，须在本实例之前列出：`motor_wheel_0..3` 为
`QDU-Robomaster/RMMotor` 实例，`cmd` 为 `QDU-Robomaster/CMD` 实例，`power_control` 为
`QDU-Robomaster/PowerControl` 实例，`referee` 为 `QDU-Robomaster/Referee` 实例。
本例没有需要 BSP 用 `XR_REGISTER` 注册的对象。带履带时把 `motor_steer_0` 填为履带电机实例的
id；`Helm` 需要把 `motor_steer_0..3` 都填为舵向电机实例的 id。PID 可写成完整的 C++ 表达式，
例如 `pid_follow: '{1.0, 30.0, 0.0, 1.0, 0.0, 0.0, true}'`。

填好后再次运行 `xrobot setup`，生成 `User/xrobot_main.hpp`。

`xrobot module show .`（在本仓库中）或 `xrobot module show Modules/QDU-Robomaster/Chassis`
（在 BSP 中）打印当前的构造函数。
