#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: No description provided
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
      wheel_radius: 0.063
      wheel_to_center: 0.31
      gravity_height: 0.0
      reductionratio: 15.7647
      wheel_resistance: 0.0
      error_compensation: 0.0
  - pid_velocity_x_:
      k: 0.0015
      p: 14.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - pid_velocity_y_:
      k: 0.0015
      p: 14.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - pid_omega_:
      k: 0.0015
      p: 14.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - pid_wheel_angle_0_:
      k: 0.0001
      p: 0.02
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - pid_wheel_angle_1_:
      k: 0.0001
      p: 0.02
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - pid_wheel_angle_2_:
      k: 0.0001
      p: 0.02
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - pid_wheel_angle_3_:
      k: 0.0001
      p: 0.02
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
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
required_hardware:
  - dr16
  - motor
  - can
  - bmi088
depends:
  - qdu-future/BMI088
  - qdu-future/RMMotor
  - qdu-future/CMD
  - xrobot-org/MadgwickAHRS
=== END MANIFEST === */
// clang-format on

#include <cstdint>
struct MotorData {
  float output_current_3508[4] = {};
  float rotorspeed_rpm_3508[4] = {};
  float target_motor_omega_3508[4] = {};
  float current_motor_omega_3508[4] = {};

  float output_current_6020[4] = {};
  float rotorspeed_rpm_6020[4] = {};
  float target_motor_omega_6020[4] = {};
  float current_motor_omega_6020[4] = {};
};

#include "CMD.hpp"
#include "Helm.hpp"
#include "Mecanum.hpp"
#include "Omni.hpp"
#include "RMMotor.hpp"
#include "app_framework.hpp"
#include "libxr_def.hpp"
#include "pid.hpp"

enum class ChassisEvent : uint8_t {
  SET_MODE_RELAX,
  SET_MODE_FOLLOW,
  SET_MODE_ROTOR,
  SET_MODE_6020_FOLLOW,
  SET_MODE_INDEPENDENT,
};

template <typename ChassisType>
class Chassis : public LibXR::Application {
 public:
  struct ChassisParam {
    float wheel_radius = 0.0f;
    float wheel_to_center = 0.0f;
    float gravity_height = 0.0f;
    float reductionratio = 0.0f;
    float wheel_resistance = 0.0f;
    float error_compensation = 0.0f;
  };

  Chassis(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app,
          RMMotor *motor_wheel_0, RMMotor *motor_wheel_1,
          RMMotor *motor_wheel_2, RMMotor *motor_wheel_3,
          RMMotor *motor_steer_0, RMMotor *motor_steer_1,
          RMMotor *motor_steer_2, RMMotor *motor_steer_3, CMD *cmd,
          PowerControl *power_control, uint32_t task_stack_depth,
          ChassisParam chassis_param = {},
          LibXR::PID<float>::Param pid_follow_={},
          LibXR::PID<float>::Param pid_velocity_x_ = {},
          LibXR::PID<float>::Param pid_velocity_y_ = {},
          LibXR::PID<float>::Param pid_omega_ = {},
          LibXR::PID<float>::Param pid_wheel_angle_0_ = {},
          LibXR::PID<float>::Param pid_wheel_angle_1_ = {},
          LibXR::PID<float>::Param pid_wheel_angle_2_ = {},
          LibXR::PID<float>::Param pid_wheel_angle_3_ = {},
          LibXR::PID<float>::Param pid_steer_angle_0_ = {},
          LibXR::PID<float>::Param pid_steer_angle_1_ = {},
          LibXR::PID<float>::Param pid_steer_angle_2_ = {},
          LibXR::PID<float>::Param pid_steer_angle_3_ = {},
          LibXR::PID<float>::Param pid_steer_speed_0_ = {},
          LibXR::PID<float>::Param pid_steer_speed_1_ = {},
          LibXR::PID<float>::Param pid_steer_speed_2_ = {},
          LibXR::PID<float>::Param pid_steer_speed_3_ = {})
      : chassis_(hw, app, motor_wheel_0, motor_wheel_1, motor_wheel_2,
                 motor_wheel_3, motor_steer_0, motor_steer_1, motor_steer_2,
                 motor_steer_3, cmd,
                 power_control,
                 task_stack_depth,
                 typename ChassisType::ChassisParam{
                     chassis_param.wheel_radius, chassis_param.wheel_to_center,
                     chassis_param.gravity_height, chassis_param.reductionratio,
                     chassis_param.wheel_resistance,
                     chassis_param.error_compensation},
                 pid_follow_,pid_velocity_x_, pid_velocity_y_, pid_omega_,
                 pid_wheel_angle_0_, pid_wheel_angle_1_, pid_wheel_angle_2_,
                 pid_wheel_angle_3_, pid_steer_angle_0_, pid_steer_angle_1_,
                 pid_steer_angle_2_, pid_steer_angle_3_, pid_steer_speed_0_,
                 pid_steer_speed_1_, pid_steer_speed_2_, pid_steer_speed_3_) {
    auto callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Chassis *chassis, uint32_t event_id) {
          UNUSED(in_isr);
          chassis->EventHandler(event_id);
        },
        this);

    chassis_event_.Register(static_cast<uint32_t>(ChassisEvent::SET_MODE_RELAX),callback);

    chassis_event_.Register(static_cast<uint32_t>(ChassisEvent::SET_MODE_FOLLOW), callback);

    chassis_event_.Register(static_cast<uint32_t>(ChassisEvent::SET_MODE_ROTOR), callback);

    chassis_event_.Register(static_cast<uint32_t>(ChassisEvent::SET_MODE_INDEPENDENT), callback);
  }

  /**
   * @brief 获取底盘的事件处理器
   * @details 通过此事件处理器可以向底盘发送事件消息，控制底盘的行为模式
   * @return LibXR::Event& 事件处理器的引用
   */
  LibXR::Event &GetEvent() { return chassis_event_; }

  /**
   * @brief 事件处理器，根据传入的事件ID执行相应操作
   * @param event_id 触发的事件ID
   */
  void EventHandler(uint32_t event_id) {
    chassis_.SetMode(
        static_cast<uint32_t>(static_cast<ChassisEvent>(event_id)));
  }

  void OnMonitor() override {}

 private:
  ChassisType chassis_;
  LibXR::Event chassis_event_;
};
