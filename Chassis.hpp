#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: No description provided
depends:
- id: xrobot-org/BMI088
  ref: same-or-dev
- id: QDU-Robomaster/RMMotor
  ref: same-or-dev
- id: QDU-Robomaster/CMD
  ref: same-or-dev
- id: QDU-Robomaster/PowerControl
  ref: same-or-dev
- id: QDU-Robomaster/Referee
  ref: same-or-dev
- id: xrobot-org/MadgwickAHRS
  ref: same-or-dev
- id: QDU-Robomaster/Motor
  ref: same-or-dev
- id: QDU-Robomaster/SuperPower
  ref: same-or-dev
=== END MANIFEST === */
// clang-format on

#include <cstdint>
#include <type_traits>

#include "thread.hpp"

/* 功率控制数组按当前最大底盘需求预留 */
static constexpr int CHASSIS_POWER_CONTROL_MAX_MOTOR_COUNT = 6;

struct MotorData
{
  float output_current_3508[CHASSIS_POWER_CONTROL_MAX_MOTOR_COUNT] = {};
  float rotorspeed_rpm_3508[CHASSIS_POWER_CONTROL_MAX_MOTOR_COUNT] = {};

  float output_current_6020[CHASSIS_POWER_CONTROL_MAX_MOTOR_COUNT] = {};
  float rotorspeed_rpm_6020[CHASSIS_POWER_CONTROL_MAX_MOTOR_COUNT] = {};
};

#include "CMD.hpp"
#include "Helm.hpp"
#include "Mecanum.hpp"
#include "Motor.hpp"
#include "Omni.hpp"
#include "RMMotor.hpp"
#include "Referee.hpp"
#include "libxr_def.hpp"
#include "pid.hpp"

template <typename ChassisType>
class Chassis
{
 public:
  using ChassisMode = typename ChassisType::ChassisMode;
  struct ChassisParam
  {
    float wheel_radius = 0.0f;
    float wheel_to_center = 0.0f;
    float gravity_height = 0.0f;
    float reduction_ratio = 0.0f;
    float wheel_resistance = 0.0f;
    float error_compensation = 0.0f;
    float gravity = 0.0f;
    float length = 0.0f;
    float width = 0.0f;
    float rotor_speed_scale = 1.0f;
    float rotor_omega_min_scale = 0.55f;
    float rotor_buffer_low_j = 35.0f;
    float rotor_buffer_high_j = 70.0f;
    float rotor_scale_lpf_alpha = 0.2f;
  };

  Chassis(

      Motor* motor_wheel_0, Motor* motor_wheel_1, Motor* motor_wheel_2,
      Motor* motor_wheel_3, Motor* motor_steer_0, Motor* motor_steer_1,
      Motor* motor_steer_2, Motor* motor_steer_3, CMD* cmd, PowerControl* power_control,
      Referee* referee, uint32_t task_stack_depth, ChassisParam chassis_param = {},
      LibXR::PID<float>::Param pid_follow_ = {},
      LibXR::PID<float>::Param pid_velocity_x_ = {},
      LibXR::PID<float>::Param pid_velocity_y_ = {},
      LibXR::PID<float>::Param pid_omega_ = {},
      LibXR::PID<float>::Param pid_wheel_speed_0_ = {},
      LibXR::PID<float>::Param pid_wheel_speed_1_ = {},
      LibXR::PID<float>::Param pid_wheel_speed_2_ = {},
      LibXR::PID<float>::Param pid_wheel_speed_3_ = {},
      LibXR::PID<float>::Param pid_steer_angle_0_ = {},
      LibXR::PID<float>::Param pid_steer_angle_1_ = {},
      LibXR::PID<float>::Param pid_steer_angle_2_ = {},
      LibXR::PID<float>::Param pid_steer_angle_3_ = {},
      LibXR::PID<float>::Param pid_steer_speed_0_ = {},
      LibXR::PID<float>::Param pid_steer_speed_1_ = {},
      LibXR::PID<float>::Param pid_steer_speed_2_ = {},
      LibXR::PID<float>::Param pid_steer_speed_3_ = {},
      LibXR::Thread::Priority thread_priority = LibXR::Thread::Priority::HIGH)
      : chassis_(motor_wheel_0, motor_wheel_1, motor_wheel_2, motor_wheel_3,
                 motor_steer_0, motor_steer_1, motor_steer_2, motor_steer_3, cmd,
                 power_control, referee, task_stack_depth,
                 typename ChassisType::ChassisParam{
                     chassis_param.wheel_radius, chassis_param.wheel_to_center,
                     chassis_param.gravity_height, chassis_param.reduction_ratio,
                     chassis_param.wheel_resistance, chassis_param.error_compensation,
                     chassis_param.gravity, chassis_param.length, chassis_param.width,
                     chassis_param.rotor_speed_scale, chassis_param.rotor_omega_min_scale,
                     chassis_param.rotor_buffer_low_j, chassis_param.rotor_buffer_high_j,
                     chassis_param.rotor_scale_lpf_alpha},
                 pid_follow_, pid_velocity_x_, pid_velocity_y_, pid_omega_,
                 pid_wheel_speed_0_, pid_wheel_speed_1_, pid_wheel_speed_2_,
                 pid_wheel_speed_3_, pid_steer_angle_0_, pid_steer_angle_1_,
                 pid_steer_angle_2_, pid_steer_angle_3_, pid_steer_speed_0_,
                 pid_steer_speed_1_, pid_steer_speed_2_, pid_steer_speed_3_,
                 thread_priority),
        referee_(referee)
  {
    auto callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Chassis* chassis, uint32_t event_id)
        {
          UNUSED(in_isr);
          chassis->EventHandler(event_id);
        },
        this);

    chassis_event_.Register(static_cast<uint32_t>(ChassisMode::RELAX), callback);

    chassis_event_.Register(static_cast<uint32_t>(ChassisMode::INDEPENDENT), callback);

    chassis_event_.Register(static_cast<uint32_t>(ChassisMode::ROTOR), callback);
    chassis_event_.Register(static_cast<uint32_t>(ChassisMode::FOLLOW), callback);
    /*
     * TRACK_START 只属于麦轮
     * 编译期判断可以让 Omni 和 Helm 继续使用各自的枚举
     */
    if constexpr (std::is_same<ChassisType, Mecanum>::value)
    {
      chassis_event_.Register(static_cast<uint32_t>(ChassisMode::TRACK_START), callback);
    }
  }

  /**
   * @brief 获取底盘的事件处理器
   * @details 通过此事件处理器可以向底盘发送事件消息，控制底盘的行为模式
   * @return LibXR::Event& 事件处理器的引用
   */
  LibXR::Event& GetEvent() { return chassis_event_; }

  /**
   * @brief 事件处理器，根据传入的事件ID执行相应操作
   * @param event_id 触发的事件ID
   */
  void EventHandler(uint32_t event_id) { chassis_.SetMode(event_id); }

  void OnMonitor() {}

 private:
  ChassisType chassis_;
  LibXR::Event chassis_event_;
  Referee* referee_;
};
