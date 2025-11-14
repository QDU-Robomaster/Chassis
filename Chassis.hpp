#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: No description provided
constructor_args:
  - cmd: '@cmd'
  - motor_wheel_0: '@motor_can1.GetMotor(0)'
  - motor_wheel_1: '@motor_can1.GetMotor(1)'
  - motor_wheel_2: '@motor_can1.GetMotor(2)'
  - motor_wheel_3: '@motor_can1.GetMotor(3)'
  - motor_steer_0: '@motor_can1.GetMotor(4)'
  - motor_steer_1: '@motor_can1.GetMotor(5)'
  - motor_steer_2: '@motor_can1.GetMotor(6)'
  - motor_steer_3: '@motor_can1.GetMotor(7)'
  - task_stack_depth: 2048
  - ChassisParam:
      wheel_radius: 0.0
      wheel_to_center: 0.0
      gravity_height: 0.0
      reductionratio: 0.0
      wheel_resistance: 0.0
      error_compensation: 0.0
  - pid_velocity_x_:
      k: 1.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - pid_velocity_y_:
      k: 1.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - pid_omega_:
      k: 1.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - pid_wheel_angle_0_:
      k: 1.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - pid_wheel_angle_1_:
      k: 1.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - pid_wheel_angle_2_:
      k: 1.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - pid_wheel_angle_3_:
      k: 1.0
      p: 0.0
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
  - ChassisType: Omni<RMMotorContainer>
  - MotorType: RMMotorContainer
required_hardware:
  - dr16
  - motor
  - can
  - bmi088
depends:
  - qdu-future/CMD
  - qdu-future/BMI088
  - qdu-future/Motor
  - xrobot-org/MadgwickAHRS
=== END MANIFEST === */
// clang-format on

#include <cstdint>

#include "CMD.hpp"
#include "Helm.hpp"
#include "Mecanum.hpp"
#include "Motor.hpp"
#include "Omni.hpp"
#include "app_framework.hpp"
#include "pid.hpp"

enum class ChassisEvent : uint8_t {
  SET_MODE_RELAX,
  SET_MODE_FOLLOW,
  SET_MODE_ROTOR,
  SET_MODE_INDENPENDENT,
};

template <typename ChassisType, typename MotorType>
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
          CMD &cmd, typename MotorType::RMMotor *motor_wheel_0,
          typename MotorType::RMMotor *motor_wheel_1,
          typename MotorType::RMMotor *motor_wheel_2,
          typename MotorType::RMMotor *motor_wheel_3,
          typename MotorType::RMMotor *motor_steer_0,
          typename MotorType::RMMotor *motor_steer_1,
          typename MotorType::RMMotor *motor_steer_2,
          typename MotorType::RMMotor *motor_steer_3, uint32_t task_stack_depth,
          ChassisParam chassis_param = {},
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
          LibXR::PID<float>::Param pid_steer_angle_3_ = {})
      : chassis_(hw, app, cmd, motor_wheel_0, motor_wheel_1, motor_wheel_2,
                 motor_wheel_3, motor_steer_0, motor_steer_1, motor_steer_2,
                 motor_steer_3, task_stack_depth,
                 typename ChassisType::ChassisParam{
                     chassis_param.wheel_radius, chassis_param.wheel_to_center,
                     chassis_param.gravity_height, chassis_param.reductionratio,
                     chassis_param.wheel_resistance,
                     chassis_param.error_compensation},
                 pid_velocity_x_, pid_velocity_y_, pid_omega_,
                 pid_wheel_angle_0_, pid_wheel_angle_1_, pid_wheel_angle_2_,
                 pid_wheel_angle_3_, pid_steer_angle_0_, pid_steer_angle_1_,
                 pid_steer_angle_2_, pid_steer_angle_3_) {
    auto cb = [](bool in_isr, void *arg, uint32_t event_id) {
      UNUSED(in_isr);
      static_cast<Chassis *>(arg)->EventHandler(event_id);
    };

    using GenericCallbackPtr = void (*)(bool, void *, uint32_t);
    using TargetCallbackPtr = void (*)(bool, Chassis *, uint32_t);

    GenericCallbackPtr generic_ptr = cb;

    auto specific_ptr = reinterpret_cast<TargetCallbackPtr>(generic_ptr);

    auto callback = LibXR::Callback<uint32_t>::Create(specific_ptr, this);
    chassis_event_.Register(static_cast<uint32_t>(ChassisEvent::SET_MODE_RELAX),
                            callback);
    chassis_event_.Register(
        static_cast<uint32_t>(ChassisEvent::SET_MODE_FOLLOW), callback);
    chassis_event_.Register(static_cast<uint32_t>(ChassisEvent::SET_MODE_ROTOR),
                            callback);
    chassis_event_.Register(
        static_cast<uint32_t>(ChassisEvent::SET_MODE_INDENPENDENT), callback);
  }

  /**
   * @brief 获取底盘的事件处理器
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
