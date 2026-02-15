# Chassis

## 1. 模块作用
底盘总控模块。通过模板参数选择 Omni、Mecanum 或 Helm 实现，并统一提供事件接口。

## 2. 主要函数说明
1. Chassis::GetEvent / EventHandler: 对外事件入口与模式切换代理。
2. Omni/Mecanum/Helm::ThreadFunction: 控制线程主循环。
3. Update / UpdateCMD / SetMode: 状态更新、命令解析与模式管理。
4. InverseKinematicsSolution / Helmcontrol / Output: 运动学解算与输出下发。
5. PowerControlUpdate / LostCtrl: 功率限制与失控保护。

## 3. 接入步骤
1. 添加模块并在 template_args 里选择 ChassisType。
2. 绑定轮电机、舵向电机、CMD、PowerControl。
3. 先验证 RELAX/FOLLOW/ROTOR 等模式切换，再联调整车。

标准命令流程：
    xrobot_add_mod Chassis --instance-id chassis
    xrobot_gen_main
    cube-cmake --build /home/leo/Documents/bsp-dev-c/build/debug --

## 4. 配置示例（YAML）
module: Chassis
entry_header: Modules/Chassis/Chassis.hpp
constructor_args:
  - motor_wheel_0: '@&motor_wheel_0'
  - motor_wheel_1: '@&motor_wheel_1'
  - motor_wheel_2: '@&motor_wheel_2'
  - motor_wheel_3: '@&motor_wheel_3'
  - motor_steer_0: '@&motor_steer_0'
  - motor_steer_1: '@&motor_steer_1'
  - motor_steer_2: '@&motor_steer_2'
  - motor_steer_3: '@&motor_steer_3'
  - cmd: '@&cmd'
  - power_control: '@&power_control'
  - task_stack_depth: 4096
  - ChassisParam:
      wheel_radius: 0.065
      wheel_to_center: 0.26
      gravity_height: 0.0
      reductionratio: 15.7647
      wheel_resistance: 0.0
      error_compensation: 0.0
      gravity: 83.692
  - pid_follow_:
      k: 1.0
      p: 20.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 30.0
      cycle: true
  - pid_velocity_x_:
      k: 1.0
      p: 20.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 30.0
      cycle: false
  - pid_velocity_y_:
      k: 1.0
      p: 20.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 30.0
      cycle: false
  - pid_omega_:
      k: 1.0
      p: 20.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 30.0
      cycle: false
  - pid_wheel_speed_0_:
      k: 1.0
      p: 0.3
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 5.0
      cycle: false
  - pid_wheel_speed_1_:
      k: 1.0
      p: 0.3
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 5.0
      cycle: false
  - pid_wheel_speed_2_:
      k: 1.0
      p: 0.3
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 5.0
      cycle: false
  - pid_wheel_speed_3_:
      k: 1.0
      p: 0.3
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 5.0
      cycle: false
  - pid_steer_angle_0_:
      k: 1.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - pid_steer_angle_1_:
      k: 1.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - pid_steer_angle_2_:
      k: 1.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - pid_steer_angle_3_:
      k: 1.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
template_args:
  - ChassisType: Omni

## 5. 依赖与硬件
Required Hardware:
  - dr16
  - motor
  - can
  - bmi088

Depends:
  - qdu-future/BMI088
  - qdu-future/RMMotor
  - qdu-future/CMD
  - xrobot-org/MadgwickAHRS

## 6. 代码入口
Modules/Chassis/Chassis.hpp
